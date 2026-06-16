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
    READ_VELOCITY = 0xA2
    READ_POSITION = 0xA3
    READ_FAST_STATE = 0xA4
    READ_STATUS = 0xAE
    CLEAR_ERROR = 0xAF
    SET_ZERO = 0xB1
    POSITION_MAX_SPEED = 0xB2
    VELOCITY_CONTROL = 0xC1
    ABSOLUTE_POSITION_CONTROL = 0xC2
    RELATIVE_POSITION_CONTROL = 0xC3
    FREE_MOTOR = 0xCF
    MIT_STATE = 0xF1


@dataclass(frozen=True)
class OutboundFrame:
    arbitration_id: int
    data: bytes


@dataclass(frozen=True)
class StatusUpdate:
    can_id: int
    status_code: int
    temp: float
    opcode: int


@dataclass(frozen=True)
class FastStateUpdate:
    can_id: int
    pos: float
    vel: float
    temp: float
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
class MitStateUpdate:
    can_id: int
    pos: float
    vel: float
    torq: float
    status_code: int
    opcode: int = int(Opcode.MIT_STATE)


FeedbackUpdate = Union[
    StatusUpdate,
    FastStateUpdate,
    PositionUpdate,
    VelocityUpdate,
    MitStateUpdate,
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
    return int(float(rad) * CPR / TWO_PI)


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
    raw = int(float(rad_s) * RAD_S_TO_RPM * 100.0)
    return OutboundFrame(
        host_command_id(motor_id),
        bytes([int(Opcode.VELOCITY_CONTROL)]) + struct.pack("<i", raw),
    )


def encode_position_max_speed(motor_id: int, rad_s: float) -> OutboundFrame:
    raw = int(abs(float(rad_s)) * RAD_S_TO_RPM * 100.0)
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


def encode_mit_control(
    motor_id: int,
    pos: float,
    vel: float,
    kp: float,
    kd: float,
    tau: float,
) -> OutboundFrame:
    p_uint = float_to_uint(pos, MIT_POSITION_MIN, MIT_POSITION_MAX, 16)
    v_uint = float_to_uint(vel, MIT_VELOCITY_MIN, MIT_VELOCITY_MAX, 12)
    kp_uint = float_to_uint(kp, MIT_KP_MIN, MIT_KP_MAX, 12)
    kd_uint = float_to_uint(kd, MIT_KD_MIN, MIT_KD_MAX, 12)
    tau_uint = float_to_uint(tau, MIT_TAU_MIN, MIT_TAU_MAX, 12)
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


def decode_reply(data: bytes, can_id: int = 0) -> Optional[FeedbackUpdate]:
    if not data:
        return None

    opcode = data[0]
    if opcode in (int(Opcode.READ_STATUS), int(Opcode.FREE_MOTOR)) and len(data) >= 8:
        return StatusUpdate(
            can_id=can_id,
            status_code=int(data[7]),
            temp=float(data[5]),
            opcode=opcode,
        )

    if opcode == int(Opcode.READ_FAST_STATE) and len(data) >= 8:
        pos_count = struct.unpack("<h", data[6:8])[0]
        vel_rpm_x100 = struct.unpack("<h", data[4:6])[0]
        return FastStateUpdate(
            can_id=can_id,
            pos=count_to_rad(pos_count),
            vel=float(vel_rpm_x100) * 0.01 * RPM_TO_RAD_S,
            temp=float(data[1]),
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
    ) and len(data) >= 7:
        pos_count = struct.unpack("<i", data[3:7])[0]
        return PositionUpdate(can_id=can_id, pos=count_to_rad(pos_count), opcode=opcode)

    if opcode == int(Opcode.MIT_STATE) and len(data) >= 7:
        p_uint = (data[1] << 8) | data[2]
        v_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        return MitStateUpdate(
            can_id=can_id,
            pos=uint_to_float(p_uint, MIT_POSITION_MIN, MIT_POSITION_MAX, 16),
            vel=uint_to_float(v_uint, MIT_VELOCITY_MIN, MIT_VELOCITY_MAX, 12),
            torq=uint_to_float(tau_uint, MIT_TAU_MIN, MIT_TAU_MAX, 12),
            status_code=1 if (data[6] & 0x02) else 0,
        )

    return None
