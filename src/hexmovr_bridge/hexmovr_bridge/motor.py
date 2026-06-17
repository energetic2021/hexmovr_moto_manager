from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field, replace
from threading import RLock
from typing import Optional

from .bus import CanBus, CanFrame
from .protocol import (
    AdvancedParam,
    ControlParam,
    CurrentUpdate,
    FastStateUpdate,
    FieldsUpdate,
    MITLimits,
    MitStateUpdate,
    Opcode,
    PositionUpdate,
    PositionType,
    StatusUpdate,
    VelocityUpdate,
    decode_reply,
    encode_absolute_position,
    encode_current_control,
    encode_mit_control,
    encode_position_max_speed,
    encode_position_filter,
    encode_relative_position,
    encode_simple_command,
    encode_trapezoid_position,
    encode_velocity_control,
    encode_write_advanced_param,
    encode_write_can_timeout,
    encode_write_control_param,
    encode_write_device_address,
    encode_write_mit_limits,
)


@dataclass(frozen=True)
class MotorState:
    has_value: bool
    can_id: int
    pos: float = 0.0
    vel: float = 0.0
    torq: float = 0.0
    q_current: float = 0.0
    t_mos: float = 0.0
    bus_voltage: float = 0.0
    bus_current: float = 0.0
    run_mode: int = 0
    status_code: int = 0
    model: str = ""
    last_update_s: float = 0.0
    last_feedback: str = ""
    extra: dict[str, object] = field(default_factory=dict)

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


class HexmovrMotor:
    def __init__(self, id: int, fb_id: int = 0, model: str = "", bus: Optional[CanBus] = None) -> None:
        motor_id = int(id)
        if motor_id < 1 or motor_id > 254:
            raise ValueError("id must be in [1, 254]")
        fb_id = int(fb_id)
        if fb_id not in (0, motor_id):
            raise ValueError("fb_id must be 0 or equal to motor_id")
        if bus is None:
            raise ValueError("bus is required")

        self.id = motor_id
        self.fb_id = fb_id
        self.model = model
        self._bus = bus
        self._lock = RLock()
        self._state: Optional[MotorState] = None
        self._mit_limits = MITLimits()

    def enable(self) -> None:
        """The Hexmovr protocol has no explicit enable opcode in this contract."""

    def disable(self) -> None:
        self._send_encoded(encode_simple_command(self.id, Opcode.FREE_MOTOR))

    def clear_error(self) -> None:
        self._send_encoded(encode_simple_command(self.id, Opcode.CLEAR_ERROR))

    def set_zero(self) -> None:
        self._send_encoded(encode_simple_command(self.id, Opcode.SET_ZERO))

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self._send_encoded(encode_mit_control(self.id, pos, vel, kp, kd, tau, self._mit_limits))

    def send_current(self, current_a: float) -> None:
        self._send_encoded(encode_current_control(self.id, current_a))

    def send_pos_vel(self, pos: float, vel: float) -> None:
        self._send_encoded(encode_position_max_speed(self.id, vel))
        self._send_encoded(encode_absolute_position(self.id, pos))

    def send_relative_pos(self, pos: float) -> None:
        self._send_encoded(encode_relative_position(self.id, pos))

    def send_trapezoid_pos(
        self,
        pos: float,
        position_type: PositionType = PositionType.ABSOLUTE,
    ) -> None:
        self._send_encoded(encode_trapezoid_position(self.id, pos, position_type))

    def send_position_filter(
        self,
        pos: float,
        position_type: PositionType = PositionType.ABSOLUTE,
    ) -> None:
        self._send_encoded(encode_position_filter(self.id, pos, position_type))

    def send_vel(self, vel: float) -> None:
        self._send_encoded(encode_velocity_control(self.id, vel))

    def return_to_zero(self) -> None:
        self._send_encoded(encode_simple_command(self.id, Opcode.RETURN_TO_ZERO))

    def set_control_param(self, param: ControlParam, value: float) -> None:
        self._send_encoded(encode_write_control_param(self.id, param, value))

    def set_advanced_param(self, param: AdvancedParam, value: float) -> None:
        self._send_encoded(encode_write_advanced_param(self.id, param, value))

    def set_can_timeout(self, enabled: bool, timeout_ms: int, action_flags: int) -> None:
        self._send_encoded(encode_write_can_timeout(self.id, enabled, timeout_ms, action_flags))

    def set_device_address(self, device_address: int) -> None:
        self._send_encoded(encode_write_device_address(self.id, device_address))

    def set_mit_limits(self, limits: MITLimits) -> None:
        self._mit_limits = limits
        self._send_encoded(encode_write_mit_limits(self.id, limits))

    def configure_mit_limits(self, limits: MITLimits) -> None:
        """Update local MIT encode/decode limits without writing motor parameters."""
        self._mit_limits = limits

    def request_feedback(self, opcode: int = int(Opcode.READ_FAST_STATE)) -> None:
        self._send_encoded(encode_simple_command(self.id, opcode))

    def accepts_frame(self, frame: CanFrame) -> bool:
        expected_id = self.id if self.fb_id == 0 else self.fb_id
        return int(frame.arbitration_id) == expected_id

    def process_feedback_frame(self, frame: CanFrame) -> bool:
        if not self.accepts_frame(frame):
            return False
        update = decode_reply(frame.data, can_id=int(frame.arbitration_id), limits=self._mit_limits)
        if update is None:
            return False

        now = time.time()
        with self._lock:
            state = self._state or MotorState(
                has_value=True,
                can_id=int(frame.arbitration_id),
                model=self.model,
                last_update_s=now,
            )
            if isinstance(update, StatusUpdate):
                state = replace(
                    state,
                    has_value=True,
                    can_id=update.can_id,
                    t_mos=update.temp,
                    bus_voltage=update.bus_voltage,
                    bus_current=update.bus_current,
                    run_mode=update.run_mode,
                    status_code=update.status_code,
                    last_update_s=now,
                    last_feedback="status",
                )
            elif isinstance(update, FastStateUpdate):
                state = replace(
                    state,
                    has_value=True,
                    can_id=update.can_id,
                    pos=update.pos,
                    vel=update.vel,
                    q_current=update.q_current,
                    t_mos=update.temp,
                    last_update_s=now,
                    last_feedback="fast_state",
                )
            elif isinstance(update, CurrentUpdate):
                state = replace(
                    state,
                    has_value=True,
                    can_id=update.can_id,
                    q_current=update.q_current,
                    last_update_s=now,
                    last_feedback="current",
                )
            elif isinstance(update, VelocityUpdate):
                state = replace(
                    state,
                    has_value=True,
                    can_id=update.can_id,
                    vel=update.vel,
                    last_update_s=now,
                    last_feedback="velocity",
                )
            elif isinstance(update, PositionUpdate):
                state = replace(
                    state,
                    has_value=True,
                    can_id=update.can_id,
                    pos=update.pos,
                    last_update_s=now,
                    last_feedback="position",
                )
            elif isinstance(update, MitStateUpdate):
                state = replace(
                    state,
                    has_value=True,
                    can_id=update.can_id,
                    pos=update.pos,
                    vel=update.vel,
                    torq=update.torq,
                    status_code=update.status_code,
                    last_update_s=now,
                    last_feedback="mit_state",
                )
            elif isinstance(update, FieldsUpdate):
                extra = dict(state.extra)
                extra.update(update.fields)
                state = replace(
                    state,
                    has_value=True,
                    can_id=update.can_id,
                    last_update_s=now,
                    last_feedback=update.name,
                    extra=extra,
                )
            self._state = state
        return True

    def latest_state(self) -> Optional[MotorState]:
        with self._lock:
            return replace(self._state) if self._state is not None else None

    def _send_encoded(self, encoded) -> None:
        self._bus.send(
            CanFrame(
                arbitration_id=encoded.arbitration_id,
                data=encoded.data,
                is_rx=False,
            )
        )
