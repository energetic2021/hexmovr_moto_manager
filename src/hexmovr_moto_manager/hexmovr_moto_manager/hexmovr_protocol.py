import math
import struct
from dataclasses import asdict, dataclass
from enum import IntEnum
from typing import Any, Optional


HOST_COMMAND_OFFSET = 0x100
MIT_COMMAND_OFFSET = 0x400
COUNTS_PER_REVOLUTION = 16384.0
TWO_PI = 2.0 * math.pi
RPM_TO_RAD_S = TWO_PI / 60.0
RAD_S_TO_RPM = 60.0 / TWO_PI


class Command(IntEnum):
    REBOOT = 0x00
    READ_VERSION = 0xA0
    READ_CURRENT = 0xA1
    READ_VELOCITY = 0xA2
    READ_POSITION = 0xA3
    READ_FAST_STATE = 0xA4
    READ_STATUS = 0xAE
    CLEAR_ERROR = 0xAF
    READ_MOTOR_PARAM = 0xB0
    SET_ZERO = 0xB1
    POSITION_MAX_SPEED = 0xB2
    MAX_Q_CURRENT = 0xB3
    CURRENT_SLOPE = 0xB4
    VELOCITY_ACCELERATION = 0xB5
    POSITION_KP = 0xB6
    POSITION_KI = 0xB7
    VELOCITY_KP = 0xB8
    VELOCITY_KI = 0xB9
    DEVICE_ADDRESS = 0xBA
    CURRENT_CONTROL = 0xC0
    VELOCITY_CONTROL = 0xC1
    ABSOLUTE_POSITION_CONTROL = 0xC2
    RELATIVE_POSITION_CONTROL = 0xC3
    RETURN_TO_ZERO = 0xC4
    CAN_TIMEOUT = 0xCD
    BRAKE_CONTROL = 0xCE
    FREE_MOTOR = 0xCF
    TRAPEZOID_ACCELERATION = 0xD0
    TRAPEZOID_DECELERATION = 0xD1
    POSITION_FILTER_BANDWIDTH = 0xD5
    POSITION_FILTER_INERTIA = 0xD6
    POSITION_FILTER_FEEDFORWARD_CURRENT = 0xD7
    TRAPEZOID_POSITION_CONTROL = 0xDA
    POSITION_FILTER_CONTROL = 0xDC
    MIT_LIMITS = 0xF0
    MIT_STATE = 0xF1


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


class PositionType(IntEnum):
    ABSOLUTE = 0x00
    RELATIVE = 0x01


class ControlParam(IntEnum):
    POSITION_MAX_SPEED = 0xB2
    MAX_Q_CURRENT = 0xB3
    CURRENT_SLOPE = 0xB4
    VELOCITY_ACCELERATION = 0xB5
    POSITION_KP = 0xB6
    POSITION_KI = 0xB7
    VELOCITY_KP = 0xB8
    VELOCITY_KI = 0xB9


class AdvancedParam(IntEnum):
    TRAPEZOID_ACCELERATION = 0xD0
    TRAPEZOID_DECELERATION = 0xD1
    POSITION_FILTER_BANDWIDTH = 0xD5
    POSITION_FILTER_INERTIA = 0xD6
    POSITION_FILTER_FEEDFORWARD_CURRENT = 0xD7


CONTROL_PARAM_NAMES = {
    "position_max_speed": ControlParam.POSITION_MAX_SPEED,
    "max_q_current": ControlParam.MAX_Q_CURRENT,
    "current_slope": ControlParam.CURRENT_SLOPE,
    "velocity_acceleration": ControlParam.VELOCITY_ACCELERATION,
    "position_kp": ControlParam.POSITION_KP,
    "position_ki": ControlParam.POSITION_KI,
    "velocity_kp": ControlParam.VELOCITY_KP,
    "velocity_ki": ControlParam.VELOCITY_KI,
}

ADVANCED_PARAM_NAMES = {
    "trapezoid_acceleration": AdvancedParam.TRAPEZOID_ACCELERATION,
    "trapezoid_deceleration": AdvancedParam.TRAPEZOID_DECELERATION,
    "position_filter_bandwidth": AdvancedParam.POSITION_FILTER_BANDWIDTH,
    "position_filter_inertia": AdvancedParam.POSITION_FILTER_INERTIA,
    "position_filter_feedforward_current": AdvancedParam.POSITION_FILTER_FEEDFORWARD_CURRENT,
}


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
    brake_state: str = "unknown"
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
    return HOST_COMMAND_OFFSET | motor_id


def mit_command_id(motor_id: int) -> int:
    return MIT_COMMAND_OFFSET | host_command_id(motor_id)


def rad_to_count(rad: float) -> int:
    return int(round(rad * COUNTS_PER_REVOLUTION / TWO_PI))


def count_to_rad(count: int) -> float:
    return float(count) * TWO_PI / COUNTS_PER_REVOLUTION


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    span = high - low
    return (float(value) / float((1 << bits) - 1)) * span + low


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    clamped = min(max(value, low), high)
    normalized = (clamped - low) / (high - low)
    return int(normalized * ((1 << bits) - 1))


def encode_simple_command(motor_id: int, command: Command) -> OutboundCommand:
    return OutboundCommand(
        arbitration_id=host_command_id(motor_id),
        payload=bytes([int(command)]),
        expected_reply_id=motor_id,
        expected_command=int(command),
        name=command.name.lower(),
    )


def encode_current_control(motor_id: int, current_a: float) -> OutboundCommand:
    payload = bytes([Command.CURRENT_CONTROL]) + struct.pack("<i", int(round(current_a * 1000.0)))
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.CURRENT_CONTROL),
        "current_control",
    )


def encode_velocity_control(motor_id: int, velocity_rad_s: float) -> OutboundCommand:
    raw = int(round(velocity_rad_s * RAD_S_TO_RPM * 100.0))
    payload = bytes([Command.VELOCITY_CONTROL]) + struct.pack("<i", raw)
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.VELOCITY_CONTROL),
        "velocity_control",
    )


def encode_absolute_position_control(motor_id: int, position_rad: float) -> OutboundCommand:
    payload = bytes([Command.ABSOLUTE_POSITION_CONTROL]) + struct.pack("<i", rad_to_count(position_rad))
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.ABSOLUTE_POSITION_CONTROL),
        "absolute_position_control",
    )


def encode_relative_position_control(motor_id: int, position_rad: float) -> OutboundCommand:
    payload = bytes([Command.RELATIVE_POSITION_CONTROL]) + struct.pack("<i", rad_to_count(position_rad))
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.RELATIVE_POSITION_CONTROL),
        "relative_position_control",
    )


def encode_trapezoid_position_control(
    motor_id: int, position_type: PositionType, position_rad: float
) -> OutboundCommand:
    payload = (
        bytes([Command.TRAPEZOID_POSITION_CONTROL, int(position_type)])
        + struct.pack("<i", rad_to_count(position_rad))
    )
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.TRAPEZOID_POSITION_CONTROL),
        "trapezoid_position_control",
    )


def encode_position_filter_control(
    motor_id: int, position_type: PositionType, position_rad: float
) -> OutboundCommand:
    payload = (
        bytes([Command.POSITION_FILTER_CONTROL, int(position_type)])
        + struct.pack("<i", rad_to_count(position_rad))
    )
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.POSITION_FILTER_CONTROL),
        "position_filter_control",
    )


def encode_write_control_param(
    motor_id: int, param: ControlParam, value: float
) -> OutboundCommand:
    payload = bytearray([int(param)])
    if param in (ControlParam.POSITION_MAX_SPEED, ControlParam.VELOCITY_ACCELERATION):
        payload.extend(struct.pack("<I", int(round(value * RAD_S_TO_RPM * 100.0))))
    elif param in (ControlParam.MAX_Q_CURRENT, ControlParam.CURRENT_SLOPE):
        payload.extend(struct.pack("<I", int(round(value * 1000.0))))
    else:
        payload.extend(struct.pack("<f", float(value)))
    return OutboundCommand(
        host_command_id(motor_id),
        bytes(payload),
        motor_id,
        int(param),
        f"write_{param.name.lower()}",
    )


def encode_write_advanced_param(
    motor_id: int, param: AdvancedParam, value: float
) -> OutboundCommand:
    payload = bytearray([int(param)])
    if param in (AdvancedParam.TRAPEZOID_ACCELERATION, AdvancedParam.TRAPEZOID_DECELERATION):
        payload.extend(struct.pack("<I", int(round(value * RAD_S_TO_RPM * 100.0))))
    elif param == AdvancedParam.POSITION_FILTER_BANDWIDTH:
        payload.extend(struct.pack("<H", int(round(value))))
    elif param == AdvancedParam.POSITION_FILTER_INERTIA:
        payload.extend(struct.pack("<f", float(value)))
    else:
        payload.extend(struct.pack("<I", int(round(value * 1000.0))))
    return OutboundCommand(
        host_command_id(motor_id),
        bytes(payload),
        motor_id,
        int(param),
        f"write_{param.name.lower()}",
    )


def encode_write_can_timeout(
    motor_id: int, enabled: bool, timeout_ms: int, action_flags: int
) -> OutboundCommand:
    payload = bytes([Command.CAN_TIMEOUT, 0x01 if enabled else 0x00]) + struct.pack(
        "<HB",
        int(timeout_ms),
        int(action_flags) & 0xFF,
    )
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.CAN_TIMEOUT),
        "write_can_timeout",
    )


def encode_brake_control(motor_id: int, brake_command: BrakeCommand) -> OutboundCommand:
    payload = bytes([Command.BRAKE_CONTROL, int(brake_command)])
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.BRAKE_CONTROL),
        "brake_control",
    )


def encode_write_device_address(motor_id: int, device_address: int) -> OutboundCommand:
    payload = bytes([Command.DEVICE_ADDRESS, device_address & 0xFF])
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.DEVICE_ADDRESS),
        "write_device_address",
    )


def encode_write_mit_limits(motor_id: int, limits: MITLimits) -> OutboundCommand:
    payload = bytes([Command.MIT_LIMITS]) + struct.pack(
        "<HHH",
        int(round(limits.position_max_rad / 0.1)),
        int(round(limits.velocity_max_rad_s / 0.01)),
        int(round(limits.torque_max_nm / 0.01)),
    )
    return OutboundCommand(
        host_command_id(motor_id),
        payload,
        motor_id,
        int(Command.MIT_LIMITS),
        "write_mit_limits",
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
    q_uint = float_to_uint(
        position_rad, -limits.position_max_rad, limits.position_max_rad, 16
    )
    dq_uint = float_to_uint(
        velocity_rad_s, -limits.velocity_max_rad_s, limits.velocity_max_rad_s, 12
    )
    kp_uint = float_to_uint(stiffness, 0.0, 500.0, 12)
    kd_uint = float_to_uint(damping, 0.0, 5.0, 12)
    tau_uint = float_to_uint(torque_nm, -limits.torque_max_nm, limits.torque_max_nm, 12)
    payload = bytes(
        [
            (q_uint >> 8) & 0xFF,
            q_uint & 0xFF,
            (dq_uint >> 4) & 0xFF,
            ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
            kp_uint & 0xFF,
            (kd_uint >> 4) & 0xFF,
            ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
            tau_uint & 0xFF,
        ]
    )
    return OutboundCommand(
        mit_command_id(motor_id),
        payload,
        motor_id,
        int(Command.MIT_STATE),
        "mit_control",
    )


def decode_reply(motor_id: int, data: bytes, limits: Optional[MITLimits] = None) -> Optional[ParsedReply]:
    if not data:
        return None
    limits = limits or MITLimits()
    command = data[0]
    fields: dict[str, Any]
    try:
        command_name = Command(command).name.lower()
    except ValueError:
        command_name = f"0x{command:02x}"

    if command == Command.READ_VERSION and len(data) >= 8:
        fields = {
            "boot_version": int.from_bytes(data[1:3], "little"),
            "software_version": int.from_bytes(data[3:5], "little"),
            "hardware_version": int.from_bytes(data[5:7], "little"),
            "can_protocol_version": data[7],
        }
    elif command in (Command.READ_CURRENT, Command.CURRENT_CONTROL) and len(data) >= 5:
        fields = {"q_current_a": struct.unpack("<i", data[1:5])[0] * 0.001}
    elif command in (Command.READ_VELOCITY, Command.VELOCITY_CONTROL) and len(data) >= 5:
        fields = {"velocity_rad_s": struct.unpack("<i", data[1:5])[0] * 0.01 * RPM_TO_RAD_S}
    elif command in (
        Command.READ_POSITION,
        Command.ABSOLUTE_POSITION_CONTROL,
        Command.RELATIVE_POSITION_CONTROL,
        Command.RETURN_TO_ZERO,
        Command.TRAPEZOID_POSITION_CONTROL,
        Command.POSITION_FILTER_CONTROL,
    ) and len(data) >= 7:
        fields = {"position_rad": count_to_rad(struct.unpack("<i", data[3:7])[0])}
    elif command == Command.READ_FAST_STATE and len(data) >= 8:
        fields = {
            "temperature_c": int(data[1]),
            "q_current_a": struct.unpack("<h", data[2:4])[0] * 0.001,
            "velocity_rad_s": struct.unpack("<h", data[4:6])[0] * 0.01 * RPM_TO_RAD_S,
            "position_rad": count_to_rad(struct.unpack("<H", data[6:8])[0]),
        }
    elif command in (Command.READ_STATUS, Command.FREE_MOTOR) and len(data) >= 8:
        fields = {
            "bus_voltage_v": struct.unpack("<H", data[1:3])[0] * 0.01,
            "bus_current_a": struct.unpack("<H", data[3:5])[0] * 0.01,
            "temperature_c": int(data[5]),
            "run_mode": int(data[6]),
            "fault_code": int(data[7]),
        }
    elif command == Command.READ_MOTOR_PARAM and len(data) >= 6:
        fields = {"torque_constant_nm_per_a": struct.unpack("<f", data[2:6])[0]}
    elif command in [int(item) for item in ControlParam] and len(data) >= 5:
        fields = _decode_control_param(command, data)
    elif command == Command.DEVICE_ADDRESS and len(data) >= 2:
        fields = {"configured_device_address": int(data[1])}
    elif command in [int(item) for item in AdvancedParam]:
        fields = _decode_advanced_param(command, data)
        if not fields:
            return None
    elif command == Command.CAN_TIMEOUT and len(data) >= 5:
        fields = {
            "can_timeout_enabled": bool(data[1]),
            "can_timeout_ms": struct.unpack("<H", data[2:4])[0],
            "can_timeout_action_flags": int(data[4]),
        }
    elif command == Command.BRAKE_CONTROL and len(data) >= 2:
        fields = {"brake_state": "closed" if data[1] == 0x01 else "open"}
    elif command == Command.MIT_LIMITS and len(data) >= 7:
        fields = {
            "mit_position_max_rad": struct.unpack("<H", data[1:3])[0] * 0.1,
            "mit_velocity_max_rad_s": struct.unpack("<H", data[3:5])[0] * 0.01,
            "mit_torque_max_nm": struct.unpack("<H", data[5:7])[0] * 0.01,
        }
    elif command == Command.MIT_STATE and len(data) >= 7:
        q_uint = (data[1] << 8) | data[2]
        dq_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        fields = {
            "position_rad": uint_to_float(
                q_uint, -limits.position_max_rad, limits.position_max_rad, 16
            ),
            "velocity_rad_s": uint_to_float(
                dq_uint, -limits.velocity_max_rad_s, limits.velocity_max_rad_s, 12
            ),
            "torque_nm": uint_to_float(
                tau_uint, -limits.torque_max_nm, limits.torque_max_nm, 12
            ),
            "in_mit_mode": bool(data[6] & 0x01),
            "fault_code": 1 if (data[6] & 0x02) else 0,
        }
    else:
        return None

    return ParsedReply(
        motor_id=motor_id,
        command=int(command),
        command_name=command_name,
        fields=fields,
    )


def _decode_control_param(command: int, data: bytes) -> dict[str, Any]:
    param = ControlParam(command)
    if param == ControlParam.POSITION_MAX_SPEED:
        return {
            "position_max_speed_rad_s": struct.unpack("<I", data[1:5])[0] * 0.01 * RAD_S_TO_RPM
        }
    if param == ControlParam.MAX_Q_CURRENT:
        return {"max_q_current_a": struct.unpack("<I", data[1:5])[0] * 0.001}
    if param == ControlParam.CURRENT_SLOPE:
        return {"current_slope_a_s": struct.unpack("<I", data[1:5])[0] * 0.001}
    if param == ControlParam.VELOCITY_ACCELERATION:
        return {
            "velocity_acceleration_rad_s2": struct.unpack("<I", data[1:5])[0]
            * 0.01
            * RAD_S_TO_RPM
        }
    if param == ControlParam.POSITION_KP:
        return {"position_kp": struct.unpack("<f", data[1:5])[0]}
    if param == ControlParam.POSITION_KI:
        return {"position_ki": struct.unpack("<f", data[1:5])[0]}
    if param == ControlParam.VELOCITY_KP:
        return {"velocity_kp": struct.unpack("<f", data[1:5])[0]}
    return {"velocity_ki": struct.unpack("<f", data[1:5])[0]}


def _decode_advanced_param(command: int, data: bytes) -> dict[str, Any]:
    param = AdvancedParam(command)
    if param == AdvancedParam.TRAPEZOID_ACCELERATION and len(data) >= 5:
        return {
            "trapezoid_acceleration_rad_s2": struct.unpack("<I", data[1:5])[0]
            * 0.01
            * RAD_S_TO_RPM
        }
    if param == AdvancedParam.TRAPEZOID_DECELERATION and len(data) >= 5:
        return {
            "trapezoid_deceleration_rad_s2": struct.unpack("<I", data[1:5])[0]
            * 0.01
            * RAD_S_TO_RPM
        }
    if param == AdvancedParam.POSITION_FILTER_BANDWIDTH and len(data) >= 3:
        return {"position_filter_bandwidth_hz": struct.unpack("<H", data[1:3])[0]}
    if param == AdvancedParam.POSITION_FILTER_INERTIA and len(data) >= 5:
        return {"position_filter_inertia_nm_per_turn_s2": struct.unpack("<f", data[1:5])[0]}
    if param == AdvancedParam.POSITION_FILTER_FEEDFORWARD_CURRENT and len(data) >= 5:
        return {
            "position_filter_feedforward_current_a": struct.unpack("<I", data[1:5])[0]
            * 0.001
        }
    return {}
