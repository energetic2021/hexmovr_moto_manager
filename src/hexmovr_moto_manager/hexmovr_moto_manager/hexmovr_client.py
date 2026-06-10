from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from .can_transport import SocketCanTransport
from .hexmovr_protocol import (
    ADVANCED_PARAM_NAMES,
    CONTROL_PARAM_NAMES,
    AdvancedParam,
    BrakeCommand,
    Command,
    MITLimits,
    MotorSnapshot,
    OutboundCommand,
    ParsedReply,
    PositionType,
    decode_reply,
    encode_absolute_position_control,
    encode_brake_control,
    encode_current_control,
    encode_mit_control,
    encode_position_filter_control,
    encode_relative_position_control,
    encode_simple_command,
    encode_trapezoid_position_control,
    encode_velocity_control,
    encode_write_advanced_param,
    encode_write_can_timeout,
    encode_write_control_param,
    encode_write_device_address,
    encode_write_mit_limits,
)


ControlParamName = str
AdvancedParamName = str


@dataclass
class ManagedMotor:
    motor_id: int
    snapshot: MotorSnapshot
    limits: MITLimits = field(default_factory=MITLimits)
    last_seen: float = 0.0
    last_error: str = ""


class HexmovrClient:
    def __init__(self, interface: str, timeout_s: float = 0.03) -> None:
        self._transport = SocketCanTransport(interface, timeout_s)
        self._timeout_s = timeout_s

    def close(self) -> None:
        self._transport.close()

    def scan(self, start_id: int, end_id: int, timeout_s: Optional[float] = None) -> dict[int, MotorSnapshot]:
        found: dict[int, MotorSnapshot] = {}
        for motor_id in range(max(1, start_id), min(254, end_id) + 1):
            reply = self.send_command(
                motor_id,
                encode_simple_command(motor_id, Command.READ_FAST_STATE),
                timeout_s=timeout_s,
            )
            if reply is None:
                continue
            snapshot = MotorSnapshot(motor_id=motor_id)
            reply.apply(snapshot)
            found[motor_id] = snapshot
        return found

    def read_fast_state(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.READ_FAST_STATE),
            timeout_s=timeout_s,
        )

    def read_status(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.READ_STATUS),
            timeout_s=timeout_s,
        )

    def read_position(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.READ_POSITION),
            timeout_s=timeout_s,
        )

    def read_velocity(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.READ_VELOCITY),
            timeout_s=timeout_s,
        )

    def read_current(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.READ_CURRENT),
            timeout_s=timeout_s,
        )

    def read_version(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.READ_VERSION),
            timeout_s=timeout_s,
        )

    def read_motor_param(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.READ_MOTOR_PARAM),
            timeout_s=timeout_s,
        )

    def read_device_address(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.DEVICE_ADDRESS),
            timeout_s=timeout_s,
        )

    def read_can_timeout(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.CAN_TIMEOUT),
            timeout_s=timeout_s,
        )

    def read_mit_limits(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.MIT_LIMITS),
            timeout_s=timeout_s,
        )

    def read_mit_state(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.MIT_STATE),
            timeout_s=timeout_s,
        )

    def read_brake_state(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_brake_control(motor_id, BrakeCommand.READ),
            timeout_s=timeout_s,
        )

    def read_control_param(
        self, motor_id: int, param: ControlParamName, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, CONTROL_PARAM_NAMES[param]),
            timeout_s=timeout_s,
        )

    def read_advanced_param(
        self, motor_id: int, param: AdvancedParamName, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, ADVANCED_PARAM_NAMES[param]),
            timeout_s=timeout_s,
        )

    def refresh_motor(
        self, motor_id: int, timeout_s: Optional[float] = None, deep: bool = True
    ) -> MotorSnapshot:
        snapshot = MotorSnapshot(motor_id=motor_id)
        for reader in (
            self.read_fast_state,
            self.read_status,
            self.read_position,
            self.read_velocity,
            self.read_current,
        ):
            reply = reader(motor_id, timeout_s=timeout_s)
            if reply is not None:
                reply.apply(snapshot)
        if deep:
            for reader in (
                self.read_version,
                self.read_motor_param,
                self.read_device_address,
                self.read_can_timeout,
                self.read_brake_state,
                self.read_mit_limits,
                self.read_mit_state,
            ):
                reply = reader(motor_id, timeout_s=timeout_s)
                if reply is not None:
                    reply.apply(snapshot)
            for name in CONTROL_PARAM_NAMES:
                reply = self.read_control_param(motor_id, name, timeout_s=timeout_s)
                if reply is not None:
                    reply.apply(snapshot)
            for name in ADVANCED_PARAM_NAMES:
                reply = self.read_advanced_param(motor_id, name, timeout_s=timeout_s)
                if reply is not None:
                    reply.apply(snapshot)
        return snapshot

    def clear_error(self, motor_id: int) -> None:
        self.send_only(encode_simple_command(motor_id, Command.CLEAR_ERROR))

    def set_zero(self, motor_id: int) -> None:
        self.send_only(encode_simple_command(motor_id, Command.SET_ZERO))

    def return_to_zero(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.RETURN_TO_ZERO),
            timeout_s=timeout_s,
        )

    def free_motor(self, motor_id: int, timeout_s: Optional[float] = None) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_simple_command(motor_id, Command.FREE_MOTOR),
            timeout_s=timeout_s,
        )

    def set_brake(
        self, motor_id: int, closed: bool, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_brake_control(motor_id, BrakeCommand.CLOSE if closed else BrakeCommand.OPEN),
            timeout_s=timeout_s,
        )

    def set_current(
        self, motor_id: int, current_a: float, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_current_control(motor_id, current_a),
            timeout_s=timeout_s,
        )

    def set_velocity(
        self, motor_id: int, velocity_rad_s: float, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_velocity_control(motor_id, velocity_rad_s),
            timeout_s=timeout_s,
        )

    def set_absolute_position(
        self, motor_id: int, position_rad: float, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_absolute_position_control(motor_id, position_rad),
            timeout_s=timeout_s,
        )

    def set_relative_position(
        self, motor_id: int, position_rad: float, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_relative_position_control(motor_id, position_rad),
            timeout_s=timeout_s,
        )

    def set_trapezoid_position(
        self,
        motor_id: int,
        position_rad: float,
        position_type: PositionType = PositionType.ABSOLUTE,
        timeout_s: Optional[float] = None,
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_trapezoid_position_control(motor_id, position_type, position_rad),
            timeout_s=timeout_s,
        )

    def set_position_filter(
        self,
        motor_id: int,
        position_rad: float,
        position_type: PositionType = PositionType.ABSOLUTE,
        timeout_s: Optional[float] = None,
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_position_filter_control(motor_id, position_type, position_rad),
            timeout_s=timeout_s,
        )

    def set_mit_control(
        self,
        motor_id: int,
        position_rad: float,
        velocity_rad_s: float,
        stiffness: float,
        damping: float,
        torque_nm: float,
        limits: Optional[MITLimits] = None,
        timeout_s: Optional[float] = None,
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_mit_control(
                motor_id,
                position_rad,
                velocity_rad_s,
                stiffness,
                damping,
                torque_nm,
                limits or MITLimits(),
            ),
            timeout_s=timeout_s,
            limits=limits,
        )

    def write_control_param(
        self, motor_id: int, name: ControlParamName, value: float, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_write_control_param(motor_id, CONTROL_PARAM_NAMES[name], value),
            timeout_s=timeout_s,
        )

    def write_advanced_param(
        self, motor_id: int, name: AdvancedParamName, value: float, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_write_advanced_param(motor_id, ADVANCED_PARAM_NAMES[name], value),
            timeout_s=timeout_s,
        )

    def write_can_timeout(
        self,
        motor_id: int,
        enabled: bool,
        timeout_ms: int,
        action_flags: int,
        timeout_s: Optional[float] = None,
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_write_can_timeout(motor_id, enabled, timeout_ms, action_flags),
            timeout_s=timeout_s,
        )

    def write_device_address(
        self, motor_id: int, device_address: int, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_write_device_address(motor_id, device_address),
            timeout_s=timeout_s,
        )

    def write_mit_limits(
        self, motor_id: int, limits: MITLimits, timeout_s: Optional[float] = None
    ) -> Optional[ParsedReply]:
        return self.send_command(
            motor_id,
            encode_write_mit_limits(motor_id, limits),
            timeout_s=timeout_s,
        )

    def send_only(self, command: OutboundCommand) -> None:
        self._transport.send_frame(command.arbitration_id, command.payload)

    def send_command(
        self,
        motor_id: int,
        command: OutboundCommand,
        timeout_s: Optional[float] = None,
        limits: Optional[MITLimits] = None,
    ) -> Optional[ParsedReply]:
        frame = self._transport.request(
            command.arbitration_id,
            command.payload,
            predicate=lambda reply: self._matches_reply(reply.can_id, reply.data, command),
            timeout_s=timeout_s or self._timeout_s,
        )
        if frame is None:
            return None
        return decode_reply(motor_id, frame.data, limits=limits)

    @staticmethod
    def _matches_reply(reply_can_id: int, reply_data: bytes, command: OutboundCommand) -> bool:
        if reply_can_id != command.expected_reply_id or not reply_data:
            return False
        if command.expected_command is None:
            return True
        return reply_data[0] == command.expected_command
