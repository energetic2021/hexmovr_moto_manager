from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Union

HOST_CMD_OFFSET = 0x100
MIT_CMD_OFFSET = 0x400
CPR = 16384.0
TWO_PI = 2.0 * math.pi
RPM_TO_RAD_S = TWO_PI / 60.0
RAD_S_TO_RPM = 60.0 / TWO_PI

MIT_POSITION_MIN = -95.5
MIT_POSITION_MAX = 95.5
MIT_VELOCITY_MIN = -45.0
MIT_VELOCITY_MAX = 45.0
MIT_KP_MIN = 0.0
MIT_KP_MAX = 500.0
MIT_KD_MIN = 0.0
MIT_KD_MAX = 5.0
MIT_TAU_MIN = -18.0
MIT_TAU_MAX = 18.0


class Opcode(IntEnum):
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


class PositionType(IntEnum):
    ABSOLUTE = 0x00
    RELATIVE = 0x01


class ControlParam(IntEnum):
    POSITION_MAX_SPEED = int(Opcode.POSITION_MAX_SPEED)
    MAX_Q_CURRENT = int(Opcode.MAX_Q_CURRENT)
    CURRENT_SLOPE = int(Opcode.CURRENT_SLOPE)
    VELOCITY_ACCELERATION = int(Opcode.VELOCITY_ACCELERATION)
    POSITION_KP = int(Opcode.POSITION_KP)
    POSITION_KI = int(Opcode.POSITION_KI)
    VELOCITY_KP = int(Opcode.VELOCITY_KP)
    VELOCITY_KI = int(Opcode.VELOCITY_KI)


class AdvancedParam(IntEnum):
    TRAPEZOID_ACCELERATION = int(Opcode.TRAPEZOID_ACCELERATION)
    TRAPEZOID_DECELERATION = int(Opcode.TRAPEZOID_DECELERATION)
    POSITION_FILTER_BANDWIDTH = int(Opcode.POSITION_FILTER_BANDWIDTH)
    POSITION_FILTER_INERTIA = int(Opcode.POSITION_FILTER_INERTIA)
    POSITION_FILTER_FEEDFORWARD_CURRENT = int(Opcode.POSITION_FILTER_FEEDFORWARD_CURRENT)


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


@dataclass(frozen=True)
class OutboundFrame:
    arbitration_id: int
    data: bytes


@dataclass(frozen=True)
class MITLimits:
    position_max_rad: float = 95.5
    velocity_max_rad_s: float = 45.0
    torque_max_nm: float = 18.0


@dataclass(frozen=True)
class StatusUpdate:
    can_id: int
    status_code: int
    temp: float
    opcode: int
    bus_voltage: float = 0.0
    bus_current: float = 0.0
    run_mode: int = 0


@dataclass(frozen=True)
class FastStateUpdate:
    can_id: int
    pos: float
    vel: float
    temp: float
    q_current: float = 0.0
    opcode: int = int(Opcode.READ_FAST_STATE)


@dataclass(frozen=True)
class PositionUpdate:
    can_id: int
    pos: float
    opcode: int


@dataclass(frozen=True)
class VelocityUpdate:
    can_id: int
    vel: float
    opcode: int


@dataclass(frozen=True)
class CurrentUpdate:
    can_id: int
    q_current: float
    opcode: int


@dataclass(frozen=True)
class MitStateUpdate:
    can_id: int
    pos: float
    vel: float
    torq: float
    status_code: int
    in_mit_mode: bool = False
    opcode: int = int(Opcode.MIT_STATE)


@dataclass(frozen=True)
class FieldsUpdate:
    can_id: int
    opcode: int
    name: str
    fields: dict[str, object]


FeedbackUpdate = Union[
    StatusUpdate,
    FastStateUpdate,
    PositionUpdate,
    VelocityUpdate,
    CurrentUpdate,
    MitStateUpdate,
    FieldsUpdate,
]


def _validate_motor_id(motor_id: int) -> int:
    motor_id = int(motor_id)
    if motor_id < 1 or motor_id > 254:
        raise ValueError("motor_id must be in [1, 254]")
    return motor_id


def host_command_id(motor_id: int) -> int:
    return HOST_CMD_OFFSET + _validate_motor_id(motor_id)


def mit_command_id(motor_id: int) -> int:
    return HOST_CMD_OFFSET + MIT_CMD_OFFSET + _validate_motor_id(motor_id)


def rad_to_count(rad: float) -> int:
    return int(round(float(rad) * CPR / TWO_PI))


def count_to_rad(count: int) -> float:
    return float(count) * TWO_PI / CPR


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    value = min(max(float(value), low), high)
    return int((value - low) / (high - low) * ((1 << bits) - 1))


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return float(value) / float((1 << bits) - 1) * (high - low) + low


def encode_simple_command(motor_id: int, opcode: int) -> OutboundFrame:
    return OutboundFrame(host_command_id(motor_id), bytes([int(opcode) & 0xFF]))


def encode_velocity_control(motor_id: int, rad_s: float) -> OutboundFrame:
    raw = int(round(float(rad_s) * RAD_S_TO_RPM * 100.0))
    return OutboundFrame(
        host_command_id(motor_id),
        bytes([int(Opcode.VELOCITY_CONTROL)]) + struct.pack("<i", raw),
    )


def encode_position_max_speed(motor_id: int, rad_s: float) -> OutboundFrame:
    raw = int(round(abs(float(rad_s)) * RAD_S_TO_RPM * 100.0))
    return OutboundFrame(
        host_command_id(motor_id),
        bytes([int(Opcode.POSITION_MAX_SPEED)]) + struct.pack("<I", raw),
    )


def encode_absolute_position(motor_id: int, rad: float) -> OutboundFrame:
    raw = rad_to_count(rad)
    return OutboundFrame(
        host_command_id(motor_id),
        bytes([int(Opcode.ABSOLUTE_POSITION_CONTROL)]) + struct.pack("<i", raw),
    )


def encode_current_control(motor_id: int, current_a: float) -> OutboundFrame:
    raw = int(round(float(current_a) * 1000.0))
    return OutboundFrame(
        host_command_id(motor_id),
        bytes([int(Opcode.CURRENT_CONTROL)]) + struct.pack("<i", raw),
    )


def encode_relative_position(motor_id: int, rad: float) -> OutboundFrame:
    raw = rad_to_count(rad)
    return OutboundFrame(
        host_command_id(motor_id),
        bytes([int(Opcode.RELATIVE_POSITION_CONTROL)]) + struct.pack("<i", raw),
    )


def encode_position_control_with_type(
    motor_id: int,
    opcode: int,
    position_type: PositionType,
    rad: float,
) -> OutboundFrame:
    if int(opcode) not in (
        int(Opcode.TRAPEZOID_POSITION_CONTROL),
        int(Opcode.POSITION_FILTER_CONTROL),
    ):
        raise ValueError("opcode must be TRAPEZOID_POSITION_CONTROL or POSITION_FILTER_CONTROL")
    return OutboundFrame(
        host_command_id(motor_id),
        bytes([int(opcode), int(position_type)]) + struct.pack("<i", rad_to_count(rad)),
    )


def encode_trapezoid_position(
    motor_id: int,
    rad: float,
    position_type: PositionType = PositionType.ABSOLUTE,
) -> OutboundFrame:
    return encode_position_control_with_type(
        motor_id,
        Opcode.TRAPEZOID_POSITION_CONTROL,
        position_type,
        rad,
    )


def encode_position_filter(
    motor_id: int,
    rad: float,
    position_type: PositionType = PositionType.ABSOLUTE,
) -> OutboundFrame:
    return encode_position_control_with_type(
        motor_id,
        Opcode.POSITION_FILTER_CONTROL,
        position_type,
        rad,
    )


def encode_write_control_param(motor_id: int, param: ControlParam, value: float) -> OutboundFrame:
    payload = bytearray([int(param)])
    if param in (ControlParam.POSITION_MAX_SPEED, ControlParam.VELOCITY_ACCELERATION):
        payload.extend(struct.pack("<I", int(round(float(value) * RAD_S_TO_RPM * 100.0))))
    elif param in (ControlParam.MAX_Q_CURRENT, ControlParam.CURRENT_SLOPE):
        payload.extend(struct.pack("<I", int(round(float(value) * 1000.0))))
    else:
        payload.extend(struct.pack("<f", float(value)))
    return OutboundFrame(host_command_id(motor_id), bytes(payload))


def encode_write_advanced_param(motor_id: int, param: AdvancedParam, value: float) -> OutboundFrame:
    payload = bytearray([int(param)])
    if param in (AdvancedParam.TRAPEZOID_ACCELERATION, AdvancedParam.TRAPEZOID_DECELERATION):
        payload.extend(struct.pack("<I", int(round(float(value) * RAD_S_TO_RPM * 100.0))))
    elif param == AdvancedParam.POSITION_FILTER_BANDWIDTH:
        payload.extend(struct.pack("<H", int(round(float(value)))))
    elif param == AdvancedParam.POSITION_FILTER_INERTIA:
        payload.extend(struct.pack("<f", float(value)))
    else:
        payload.extend(struct.pack("<I", int(round(float(value) * 1000.0))))
    return OutboundFrame(host_command_id(motor_id), bytes(payload))


def encode_write_can_timeout(
    motor_id: int,
    enabled: bool,
    timeout_ms: int,
    action_flags: int,
) -> OutboundFrame:
    payload = bytes([int(Opcode.CAN_TIMEOUT), 0x01 if enabled else 0x00])
    payload += struct.pack("<HB", int(timeout_ms), int(action_flags) & 0xFF)
    return OutboundFrame(host_command_id(motor_id), payload)


def encode_write_device_address(motor_id: int, device_address: int) -> OutboundFrame:
    return OutboundFrame(
        host_command_id(motor_id),
        bytes([int(Opcode.DEVICE_ADDRESS), int(device_address) & 0xFF]),
    )


def encode_write_mit_limits(motor_id: int, limits: MITLimits) -> OutboundFrame:
    payload = bytes([int(Opcode.MIT_LIMITS)]) + struct.pack(
        "<HHH",
        int(round(float(limits.position_max_rad) / 0.1)),
        int(round(float(limits.velocity_max_rad_s) / 0.01)),
        int(round(float(limits.torque_max_nm) / 0.01)),
    )
    return OutboundFrame(host_command_id(motor_id), payload)


def encode_mit_control(
    motor_id: int,
    pos: float,
    vel: float,
    kp: float,
    kd: float,
    tau: float,
    limits: Optional[MITLimits] = None,
) -> OutboundFrame:
    limits = limits or MITLimits()
    p_uint = float_to_uint(pos, -limits.position_max_rad, limits.position_max_rad, 16)
    v_uint = float_to_uint(vel, -limits.velocity_max_rad_s, limits.velocity_max_rad_s, 12)
    kp_uint = float_to_uint(kp, MIT_KP_MIN, MIT_KP_MAX, 12)
    kd_uint = float_to_uint(kd, MIT_KD_MIN, MIT_KD_MAX, 12)
    tau_uint = float_to_uint(tau, -limits.torque_max_nm, limits.torque_max_nm, 12)
    payload = bytes(
        [
            (p_uint >> 8) & 0xFF,
            p_uint & 0xFF,
            (v_uint >> 4) & 0xFF,
            ((v_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
            kp_uint & 0xFF,
            (kd_uint >> 4) & 0xFF,
            ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
            tau_uint & 0xFF,
        ]
    )
    return OutboundFrame(mit_command_id(motor_id), payload)


def decode_reply(
    data: bytes,
    can_id: int = 0,
    limits: Optional[MITLimits] = None,
) -> Optional[FeedbackUpdate]:
    if not data:
        return None
    limits = limits or MITLimits()

    opcode = data[0]
    name = _opcode_name(opcode)

    if opcode == int(Opcode.READ_VERSION) and len(data) >= 8:
        return FieldsUpdate(
            can_id=can_id,
            opcode=opcode,
            name=name,
            fields={
                "boot_version": int.from_bytes(data[1:3], "little"),
                "software_version": int.from_bytes(data[3:5], "little"),
                "hardware_version": int.from_bytes(data[5:7], "little"),
                "can_protocol_version": data[7],
            },
        )

    if opcode in (int(Opcode.READ_CURRENT), int(Opcode.CURRENT_CONTROL)) and len(data) >= 5:
        return CurrentUpdate(
            can_id=can_id,
            q_current=struct.unpack("<i", data[1:5])[0] * 0.001,
            opcode=opcode,
        )

    if opcode in (int(Opcode.READ_STATUS), int(Opcode.FREE_MOTOR)) and len(data) >= 8:
        return StatusUpdate(
            can_id=can_id,
            status_code=int(data[7]),
            temp=float(data[5]),
            opcode=opcode,
            bus_voltage=struct.unpack("<H", data[1:3])[0] * 0.01,
            bus_current=struct.unpack("<H", data[3:5])[0] * 0.01,
            run_mode=int(data[6]),
        )

    if opcode == int(Opcode.READ_FAST_STATE) and len(data) >= 8:
        pos_count = struct.unpack("<h", data[6:8])[0]
        vel_rpm_x100 = struct.unpack("<h", data[4:6])[0]
        q_current_ma = struct.unpack("<h", data[2:4])[0]
        return FastStateUpdate(
            can_id=can_id,
            pos=count_to_rad(pos_count),
            vel=float(vel_rpm_x100) * 0.01 * RPM_TO_RAD_S,
            temp=float(data[1]),
            q_current=float(q_current_ma) * 0.001,
        )

    if opcode in (int(Opcode.READ_VELOCITY), int(Opcode.VELOCITY_CONTROL)) and len(data) >= 5:
        vel_rpm_x100 = struct.unpack("<i", data[1:5])[0]
        return VelocityUpdate(
            can_id=can_id,
            vel=float(vel_rpm_x100) * 0.01 * RPM_TO_RAD_S,
            opcode=opcode,
        )

    if opcode in (
        int(Opcode.READ_POSITION),
        int(Opcode.ABSOLUTE_POSITION_CONTROL),
        int(Opcode.RELATIVE_POSITION_CONTROL),
        int(Opcode.RETURN_TO_ZERO),
        int(Opcode.TRAPEZOID_POSITION_CONTROL),
        int(Opcode.POSITION_FILTER_CONTROL),
    ) and len(data) >= 7:
        pos_count = struct.unpack("<i", data[3:7])[0]
        return PositionUpdate(can_id=can_id, pos=count_to_rad(pos_count), opcode=opcode)

    if opcode == int(Opcode.READ_MOTOR_PARAM) and len(data) >= 6:
        return FieldsUpdate(
            can_id=can_id,
            opcode=opcode,
            name=name,
            fields={"torque_constant_nm_per_a": struct.unpack("<f", data[2:6])[0]},
        )

    if opcode in [int(item) for item in ControlParam] and len(data) >= 5:
        return FieldsUpdate(
            can_id=can_id,
            opcode=opcode,
            name=name,
            fields=_decode_control_param(opcode, data),
        )

    if opcode == int(Opcode.DEVICE_ADDRESS) and len(data) >= 2:
        return FieldsUpdate(
            can_id=can_id,
            opcode=opcode,
            name=name,
            fields={"configured_device_address": int(data[1])},
        )

    if opcode in [int(item) for item in AdvancedParam]:
        fields = _decode_advanced_param(opcode, data)
        if fields:
            return FieldsUpdate(can_id=can_id, opcode=opcode, name=name, fields=fields)

    if opcode == int(Opcode.CAN_TIMEOUT) and len(data) >= 5:
        return FieldsUpdate(
            can_id=can_id,
            opcode=opcode,
            name=name,
            fields={
                "can_timeout_enabled": bool(data[1]),
                "can_timeout_ms": struct.unpack("<H", data[2:4])[0],
                "can_timeout_action_flags": int(data[4]),
            },
        )

    if opcode == int(Opcode.MIT_LIMITS) and len(data) >= 7:
        return FieldsUpdate(
            can_id=can_id,
            opcode=opcode,
            name=name,
            fields={
                "mit_position_max_rad": struct.unpack("<H", data[1:3])[0] * 0.1,
                "mit_velocity_max_rad_s": struct.unpack("<H", data[3:5])[0] * 0.01,
                "mit_torque_max_nm": struct.unpack("<H", data[5:7])[0] * 0.01,
            },
        )

    if opcode == int(Opcode.MIT_STATE) and len(data) >= 7:
        p_uint = (data[1] << 8) | data[2]
        v_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        return MitStateUpdate(
            can_id=can_id,
            pos=uint_to_float(p_uint, -limits.position_max_rad, limits.position_max_rad, 16),
            vel=uint_to_float(v_uint, -limits.velocity_max_rad_s, limits.velocity_max_rad_s, 12),
            torq=uint_to_float(tau_uint, -limits.torque_max_nm, limits.torque_max_nm, 12),
            status_code=1 if (data[6] & 0x02) else 0,
            in_mit_mode=bool(data[6] & 0x01),
        )

    return None


def _opcode_name(opcode: int) -> str:
    try:
        return Opcode(opcode).name.lower()
    except ValueError:
        return f"0x{opcode:02x}"


def _decode_control_param(opcode: int, data: bytes) -> dict[str, object]:
    param = ControlParam(opcode)
    if param == ControlParam.POSITION_MAX_SPEED:
        return {"position_max_speed_rad_s": struct.unpack("<I", data[1:5])[0] * 0.01 * RPM_TO_RAD_S}
    if param == ControlParam.MAX_Q_CURRENT:
        return {"max_q_current_a": struct.unpack("<I", data[1:5])[0] * 0.001}
    if param == ControlParam.CURRENT_SLOPE:
        return {"current_slope_a_s": struct.unpack("<I", data[1:5])[0] * 0.001}
    if param == ControlParam.VELOCITY_ACCELERATION:
        return {
            "velocity_acceleration_rad_s2": struct.unpack("<I", data[1:5])[0] * 0.01 * RPM_TO_RAD_S
        }
    if param == ControlParam.POSITION_KP:
        return {"position_kp": struct.unpack("<f", data[1:5])[0]}
    if param == ControlParam.POSITION_KI:
        return {"position_ki": struct.unpack("<f", data[1:5])[0]}
    if param == ControlParam.VELOCITY_KP:
        return {"velocity_kp": struct.unpack("<f", data[1:5])[0]}
    return {"velocity_ki": struct.unpack("<f", data[1:5])[0]}


def _decode_advanced_param(opcode: int, data: bytes) -> dict[str, object]:
    param = AdvancedParam(opcode)
    if param == AdvancedParam.TRAPEZOID_ACCELERATION and len(data) >= 5:
        return {
            "trapezoid_acceleration_rad_s2": struct.unpack("<I", data[1:5])[0] * 0.01 * RPM_TO_RAD_S
        }
    if param == AdvancedParam.TRAPEZOID_DECELERATION and len(data) >= 5:
        return {
            "trapezoid_deceleration_rad_s2": struct.unpack("<I", data[1:5])[0] * 0.01 * RPM_TO_RAD_S
        }
    if param == AdvancedParam.POSITION_FILTER_BANDWIDTH and len(data) >= 3:
        return {"position_filter_bandwidth_hz": struct.unpack("<H", data[1:3])[0]}
    if param == AdvancedParam.POSITION_FILTER_INERTIA and len(data) >= 5:
        return {"position_filter_inertia_nm_per_turn_s2": struct.unpack("<f", data[1:5])[0]}
    if param == AdvancedParam.POSITION_FILTER_FEEDFORWARD_CURRENT and len(data) >= 5:
        return {"position_filter_feedforward_current_a": struct.unpack("<I", data[1:5])[0] * 0.001}
    return {}
