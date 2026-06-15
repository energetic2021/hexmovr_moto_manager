from __future__ import annotations

import time
from dataclasses import asdict, dataclass, replace
from threading import RLock
from typing import Optional

from .bus import CanBus, CanFrame
from .protocol import (
    FastStateUpdate,
    MitStateUpdate,
    Opcode,
    PositionUpdate,
    StatusUpdate,
    VelocityUpdate,
    decode_reply,
    encode_absolute_position,
    encode_mit_control,
    encode_position_max_speed,
    encode_simple_command,
    encode_velocity_control,
)


@dataclass(frozen=True)
class MotorState:
    has_value: bool
    can_id: int
    pos: float = 0.0
    vel: float = 0.0
    torq: float = 0.0
    t_mos: float = 0.0
    status_code: int = 0
    model: str = ""
    last_update_s: float = 0.0
    last_feedback: str = ""

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

    def enable(self) -> None:
        """The Hexmovr protocol has no explicit enable opcode in this contract."""

    def disable(self) -> None:
        self._send_encoded(encode_simple_command(self.id, Opcode.FREE_MOTOR))

    def clear_error(self) -> None:
        self._send_encoded(encode_simple_command(self.id, Opcode.CLEAR_ERROR))

    def set_zero(self) -> None:
        self._send_encoded(encode_simple_command(self.id, Opcode.SET_ZERO))

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self._send_encoded(encode_mit_control(self.id, pos, vel, kp, kd, tau))

    def send_pos_vel(self, pos: float, vel: float) -> None:
        self._send_encoded(encode_position_max_speed(self.id, vel))
        self._send_encoded(encode_absolute_position(self.id, pos))

    def send_vel(self, vel: float) -> None:
        self._send_encoded(encode_velocity_control(self.id, vel))

    def request_feedback(self, opcode: int = int(Opcode.READ_FAST_STATE)) -> None:
        self._send_encoded(encode_simple_command(self.id, opcode))

    def accepts_frame(self, frame: CanFrame) -> bool:
        expected_id = self.id if self.fb_id == 0 else self.fb_id
        return int(frame.arbitration_id) == expected_id

    def process_feedback_frame(self, frame: CanFrame) -> bool:
        if not self.accepts_frame(frame):
            return False
        update = decode_reply(frame.data, can_id=int(frame.arbitration_id))
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
                    t_mos=update.temp,
                    last_update_s=now,
                    last_feedback="fast_state",
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
