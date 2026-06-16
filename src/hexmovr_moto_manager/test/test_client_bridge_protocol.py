import struct

from hexmovr_bridge import protocol as bridge_protocol
from hexmovr_moto_manager import hexmovr_client as client
from hexmovr_moto_manager.hexmovr_protocol import MotorSnapshot


def test_client_velocity_encoder_uses_bridge_protocol():
    command = client.encode_velocity_control(3, 1.0)
    bridge_frame = bridge_protocol.encode_velocity_control(3, 1.0)

    assert command.arbitration_id == bridge_frame.arbitration_id
    assert command.payload == bridge_frame.data
    assert command.expected_reply_id == 3
    assert command.expected_command == int(bridge_protocol.Opcode.VELOCITY_CONTROL)


def test_client_decode_bridge_status_to_manager_snapshot_fields():
    data = bytes([0xAE]) + struct.pack("<H", 2400) + struct.pack("<H", 125) + bytes([30, 3, 0])
    reply = client._decode_reply(1, data)
    snapshot = MotorSnapshot(motor_id=1)

    assert reply is not None
    reply.apply(snapshot)
    assert snapshot.bus_voltage_v == 24.0
    assert snapshot.bus_current_a == 1.25
    assert snapshot.temperature_c == 30
    assert snapshot.run_mode == 3
    assert snapshot.fault_code == 0
