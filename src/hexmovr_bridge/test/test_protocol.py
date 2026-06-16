import math
import struct

from hexmovr_bridge import protocol as p


def test_command_ids():
    assert p.host_command_id(1) == 0x101
    assert p.mit_command_id(1) == 0x501


def test_velocity_control_encoding():
    frame = p.encode_velocity_control(3, math.pi)
    assert frame.arbitration_id == 0x103
    assert frame.data[0] == p.Opcode.VELOCITY_CONTROL
    assert struct.unpack("<i", frame.data[1:5])[0] == int(round(math.pi * p.RAD_S_TO_RPM * 100))


def test_position_encoding():
    frame = p.encode_absolute_position(2, math.pi)
    assert frame.arbitration_id == 0x102
    assert frame.data[0] == p.Opcode.ABSOLUTE_POSITION_CONTROL
    assert struct.unpack("<i", frame.data[1:5])[0] == 8192


def test_current_control_encoding():
    frame = p.encode_current_control(4, 1.25)
    assert frame.arbitration_id == 0x104
    assert frame.data[0] == p.Opcode.CURRENT_CONTROL
    assert struct.unpack("<i", frame.data[1:5])[0] == 1250


def test_trapezoid_position_encoding():
    frame = p.encode_trapezoid_position(2, math.pi, p.PositionType.RELATIVE)
    assert frame.arbitration_id == 0x102
    assert frame.data[0] == p.Opcode.TRAPEZOID_POSITION_CONTROL
    assert frame.data[1] == p.PositionType.RELATIVE
    assert struct.unpack("<i", frame.data[2:6])[0] == 8192


def test_write_params_encoding():
    control = p.encode_write_control_param(1, p.ControlParam.MAX_Q_CURRENT, 2.5)
    advanced = p.encode_write_advanced_param(1, p.AdvancedParam.POSITION_FILTER_BANDWIDTH, 120)
    assert control.data[0] == p.Opcode.MAX_Q_CURRENT
    assert struct.unpack("<I", control.data[1:5])[0] == 2500
    assert advanced.data[0] == p.Opcode.POSITION_FILTER_BANDWIDTH
    assert struct.unpack("<H", advanced.data[1:3])[0] == 120


def test_mit_control_encoding_has_8_byte_payload():
    frame = p.encode_mit_control(1, 0.0, 0.0, 100.0, 1.0, 0.0)
    assert frame.arbitration_id == 0x501
    assert len(frame.data) == 8


def test_fast_state_decode():
    data = bytes([0xA4, 25, 0, 0]) + struct.pack("<h", 100) + struct.pack("<h", 1024)
    update = p.decode_reply(data, can_id=1)
    assert update.can_id == 1
    assert update.temp == 25.0
    assert update.vel == 100 * 0.01 * p.RPM_TO_RAD_S
    assert update.pos == p.count_to_rad(1024)


def test_status_decode_has_bus_fields():
    data = bytes([0xAE]) + struct.pack("<H", 2400) + struct.pack("<H", 125) + bytes([30, 3, 0])
    update = p.decode_reply(data, can_id=1)
    assert update.status_code == 0
    assert update.temp == 30.0
    assert update.bus_voltage == 24.0
    assert update.bus_current == 1.25
    assert update.run_mode == 3


def test_fields_decode_for_mit_limits():
    data = bytes([0xF0]) + struct.pack("<HHH", 955, 4500, 1800)
    update = p.decode_reply(data, can_id=1)
    assert update.name == "mit_limits"
    assert update.fields["mit_position_max_rad"] == 95.5
    assert update.fields["mit_velocity_max_rad_s"] == 45.0
    assert update.fields["mit_torque_max_nm"] == 18.0


def test_position_decode():
    data = bytes([0xC2, 0, 0]) + struct.pack("<i", 2048)
    update = p.decode_reply(data, can_id=7)
    assert update.can_id == 7
    assert update.pos == p.count_to_rad(2048)


def test_mit_state_decode():
    p_uint = 32767
    v_uint = 2047
    tau_uint = 2047
    data = bytes(
        [
            0xF1,
            (p_uint >> 8) & 0xFF,
            p_uint & 0xFF,
            (v_uint >> 4) & 0xFF,
            ((v_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
            tau_uint & 0xFF,
            0x02,
        ]
    )
    update = p.decode_reply(data, can_id=1)
    assert update.status_code == 1
    assert abs(update.pos) < 0.01
    assert abs(update.vel) < 0.02
    assert abs(update.torq) < 0.02
