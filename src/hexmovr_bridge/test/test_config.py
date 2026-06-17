import pytest

from hexmovr_bridge.config import parse_hexmovr_config


def test_parse_config_with_defaults_and_disabled_motor():
    config = parse_hexmovr_config(
        {
            "channel": "can1",
            "feedback_period_s": 0.2,
            "defaults": {
                "model": "hexmovr",
                "max_velocity_rad_s": 3.0,
                "mit_limits": {
                    "position_max_rad": 80.0,
                    "velocity_max_rad_s": 30.0,
                    "torque_max_nm": 12.0,
                },
            },
            "motors": [
                {"id": 1, "name": "left"},
                {"id": 2, "enabled": False, "max_velocity_rad_s": 1.5},
            ],
        }
    )

    assert config.channel == "can1"
    assert config.feedback_period_s == 0.2
    assert config.motor_ids == [1]
    assert config.motors[0].model == "hexmovr"
    assert config.motors[0].mit_limits.position_max_rad == 80.0
    assert config.motors[1].enabled is False
    assert config.motors[1].max_velocity_rad_s == 1.5


def test_parse_config_rejects_duplicate_motor_ids():
    with pytest.raises(ValueError, match="duplicate motor id"):
        parse_hexmovr_config({"motors": [{"id": 1}, {"id": 1}]})


def test_parse_config_rejects_invalid_fb_id():
    with pytest.raises(ValueError, match="fb_id"):
        parse_hexmovr_config({"motors": [{"id": 1, "fb_id": 2}]})
