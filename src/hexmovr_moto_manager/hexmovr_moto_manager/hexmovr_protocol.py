from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import IntEnum
from typing import Any, Optional

from hexmovr_bridge import protocol as bridge_protocol

HOST_COMMAND_OFFSET = bridge_protocol.HOST_CMD_OFFSET
MIT_COMMAND_OFFSET = bridge_protocol.MIT_CMD_OFFSET
COUNTS_PER_REVOLUTION = bridge_protocol.CPR
TWO_PI = bridge_protocol.TWO_PI
RPM_TO_RAD_S = bridge_protocol.RPM_TO_RAD_S
RAD_S_TO_RPM = bridge_protocol.RAD_S_TO_RPM

Command = bridge_protocol.Opcode
PositionType = bridge_protocol.PositionType
ControlParam = bridge_protocol.ControlParam
AdvancedParam = bridge_protocol.AdvancedParam
CONTROL_PARAM_NAMES = bridge_protocol.CONTROL_PARAM_NAMES
ADVANCED_PARAM_NAMES = bridge_protocol.ADVANCED_PARAM_NAMES


class RunMode(IntEnum):
    OFF = 0
    VOLTAGE = 1
    CURRENT = 2
    VELOCITY = 3
    POSITION = 4


class BrakeCommand(IntEnum):
    OPEN = 0x00
    CLOSE = 0x01
    READ = 0xFF


@dataclass
class MITLimits:
    position_max_rad: float = 95.5
    velocity_max_rad_s: float = 45.0
    torque_max_nm: float = 18.0

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass
class MotorSnapshot:
    motor_id: int
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    q_current_a: float = 0.0
    bus_voltage_v: float = 0.0
    bus_current_a: float = 0.0
    temperature_c: int = 0
    run_mode: int = int(RunMode.OFF)
    fault_code: int = 0
    can_timeout_enabled: bool = False
    can_timeout_ms: int = 0
    can_timeout_action_flags: int = 0
    brake_state: str = "disabled"
    boot_version: int = 0
    software_version: int = 0
    hardware_version: int = 0
    can_protocol_version: int = 0
    torque_constant_nm_per_a: float = 0.0
    position_max_speed_rad_s: float = 0.0
    max_q_current_a: float = 0.0
    current_slope_a_s: float = 0.0
    velocity_acceleration_rad_s2: float = 0.0
    position_kp: float = 0.0
    position_ki: float = 0.0
    velocity_kp: float = 0.0
    velocity_ki: float = 0.0
    trapezoid_acceleration_rad_s2: float = 0.0
    trapezoid_deceleration_rad_s2: float = 0.0
    position_filter_bandwidth_hz: int = 0
    position_filter_inertia_nm_per_turn_s2: float = 0.0
    position_filter_feedforward_current_a: float = 0.0
    configured_device_address: int = 0
    mit_position_max_rad: float = 95.5
    mit_velocity_max_rad_s: float = 45.0
    mit_torque_max_nm: float = 18.0
    in_mit_mode: bool = False
    valid: bool = False
    last_command: str = ""

    def update(self, values: dict[str, Any], command_name: str) -> None:
        for key, value in values.items():
            if hasattr(self, key):
                setattr(self, key, value)
        self.valid = True
        self.last_command = command_name

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class OutboundCommand:
    arbitration_id: int
    payload: bytes
    expected_reply_id: int
    expected_command: Optional[int]
    name: str


@dataclass(frozen=True)
class ParsedReply:
    motor_id: int
    command: int
    command_name: str
    fields: dict[str, Any]

    def apply(self, snapshot: MotorSnapshot) -> MotorSnapshot:
        snapshot.update(self.fields, self.command_name)
        return snapshot


def host_command_id(motor_id: int) -> int:
    return bridge_protocol.host_command_id(motor_id)


def mit_command_id(motor_id: int) -> int:
    return bridge_protocol.mit_command_id(motor_id)


def rad_to_count(rad: float) -> int:
    return bridge_protocol.rad_to_count(rad)


def count_to_rad(count: int) -> float:
    return bridge_protocol.count_to_rad(count)


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return bridge_protocol.uint_to_float(value, low, high, bits)


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    return bridge_protocol.float_to_uint(value, low, high, bits)


def _wrap_frame(
    frame: bridge_protocol.OutboundFrame,
    expected_command: Optional[int],
    name: str,
) -> OutboundCommand:
    return OutboundCommand(
        arbitration_id=frame.arbitration_id,
        payload=frame.data,
        expected_reply_id=int(frame.arbitration_id) & 0xFF,
        expected_command=expected_command,
        name=name,
    )


def encode_simple_command(motor_id: int, command: Command) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_simple_command(motor_id, command),
        int(command),
        Command(command).name.lower(),
    )


def encode_current_control(motor_id: int, current_a: float) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_current_control(motor_id, current_a),
        int(Command.CURRENT_CONTROL),
        "current_control",
    )


def encode_velocity_control(motor_id: int, velocity_rad_s: float) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_velocity_control(motor_id, velocity_rad_s),
        int(Command.VELOCITY_CONTROL),
        "velocity_control",
    )


def encode_absolute_position_control(motor_id: int, position_rad: float) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_absolute_position(motor_id, position_rad),
        int(Command.ABSOLUTE_POSITION_CONTROL),
        "absolute_position_control",
    )


def encode_relative_position_control(motor_id: int, position_rad: float) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_relative_position(motor_id, position_rad),
        int(Command.RELATIVE_POSITION_CONTROL),
        "relative_position_control",
    )


def encode_trapezoid_position_control(
    motor_id: int,
    position_type: PositionType,
    position_rad: float,
) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_trapezoid_position(motor_id, position_rad, position_type),
        int(Command.TRAPEZOID_POSITION_CONTROL),
        "trapezoid_position_control",
    )


def encode_position_filter_control(
    motor_id: int,
    position_type: PositionType,
    position_rad: float,
) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_position_filter(motor_id, position_rad, position_type),
        int(Command.POSITION_FILTER_CONTROL),
        "position_filter_control",
    )


def encode_mit_control(
    motor_id: int,
    position_rad: float,
    velocity_rad_s: float,
    stiffness: float,
    damping: float,
    torque_nm: float,
    limits: MITLimits,
) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_mit_control(
            motor_id,
            position_rad,
            velocity_rad_s,
            stiffness,
            damping,
            torque_nm,
            limits,
        ),
        int(Command.MIT_STATE),
        "mit_control",
    )


def encode_write_control_param(motor_id: int, param: ControlParam, value: float) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_write_control_param(motor_id, param, value),
        int(param),
        f"write_{param.name.lower()}",
    )


def encode_write_advanced_param(motor_id: int, param: AdvancedParam, value: float) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_write_advanced_param(motor_id, param, value),
        int(param),
        f"write_{param.name.lower()}",
    )


def encode_write_can_timeout(
    motor_id: int,
    enabled: bool,
    timeout_ms: int,
    action_flags: int,
) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_write_can_timeout(motor_id, enabled, timeout_ms, action_flags),
        int(Command.CAN_TIMEOUT),
        "write_can_timeout",
    )


def encode_write_device_address(motor_id: int, device_address: int) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_write_device_address(motor_id, device_address),
        int(Command.DEVICE_ADDRESS),
        "write_device_address",
    )


def encode_write_mit_limits(motor_id: int, limits: MITLimits) -> OutboundCommand:
    return _wrap_frame(
        bridge_protocol.encode_write_mit_limits(motor_id, limits),
        int(Command.MIT_LIMITS),
        "write_mit_limits",
    )


def encode_brake_control(motor_id: int, brake_command: BrakeCommand) -> OutboundCommand:
    del motor_id, brake_command
    raise NotImplementedError("brake control is disabled")


def decode_reply(
    motor_id: int,
    data: bytes,
    limits: Optional[MITLimits] = None,
) -> Optional[ParsedReply]:
    from .hexmovr_client import _decode_reply

    return _decode_reply(motor_id, data, limits=limits)
