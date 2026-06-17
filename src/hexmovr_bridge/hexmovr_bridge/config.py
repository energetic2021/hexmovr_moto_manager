from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import yaml

from .protocol import MITLimits


@dataclass(frozen=True)
class MotorConfig:
    id: int
    fb_id: int = 0
    name: str = ""
    model: str = ""
    enabled: bool = True
    default_mode: str = ""
    max_velocity_rad_s: Optional[float] = None
    max_current_a: Optional[float] = None
    max_torque_nm: Optional[float] = None
    feedback_period_s: Optional[float] = None
    mit_limits: MITLimits = field(default_factory=MITLimits)


@dataclass(frozen=True)
class HexmovrConfig:
    channel: str = "can0"
    motors: tuple[MotorConfig, ...] = ()
    control_period_s: Optional[float] = None
    state_period_s: Optional[float] = None
    feedback_period_s: Optional[float] = None

    @property
    def motor_ids(self) -> list[int]:
        return [motor.id for motor in self.motors if motor.enabled]

    def motor_by_id(self, motor_id: int) -> Optional[MotorConfig]:
        for motor in self.motors:
            if motor.id == int(motor_id):
                return motor
        return None


def load_hexmovr_config(path: str | Path) -> HexmovrConfig:
    config_path = resolve_config_path(path)
    if not config_path.exists():
        raise FileNotFoundError(
            f"Hexmovr config file not found: {config_path}. "
            "Use an absolute path or package://hexmovr_bridge/config/hexmovr_motors.example.yaml"
        )
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    if not isinstance(raw, dict):
        raise ValueError("hexmovr config YAML must contain a mapping at the top level")
    return parse_hexmovr_config(raw)


def resolve_config_path(path: str | Path) -> Path:
    raw_path = str(path).strip()
    if raw_path.startswith("package://"):
        package_and_path = raw_path[len("package://") :]
        package_name, sep, relative_path = package_and_path.partition("/")
        if not sep or not package_name or not relative_path:
            raise ValueError(
                "package config path must look like package://package_name/path/to/file.yaml"
            )
        try:
            from ament_index_python.packages import get_package_share_directory
        except ImportError as exc:
            raise RuntimeError("package:// config paths require ament_index_python") from exc
        return Path(get_package_share_directory(package_name)) / relative_path
    return Path(raw_path).expanduser()


def parse_hexmovr_config(raw: dict[str, Any]) -> HexmovrConfig:
    channel = str(raw.get("channel", raw.get("can_interface", "can0")))
    defaults = _optional_mapping(raw.get("defaults", {}), "defaults")
    motor_items = raw.get("motors", [])
    if not isinstance(motor_items, list):
        raise ValueError("motors must be a list")

    motors: list[MotorConfig] = []
    seen_ids: set[int] = set()
    for item in motor_items:
        merged = dict(defaults)
        merged.update(_optional_mapping(item, "motor item"))
        motor = _parse_motor_config(merged)
        if motor.id in seen_ids:
            raise ValueError(f"duplicate motor id: {motor.id}")
        seen_ids.add(motor.id)
        motors.append(motor)

    return HexmovrConfig(
        channel=channel,
        motors=tuple(motors),
        control_period_s=_optional_float(raw.get("control_period_s"), "control_period_s"),
        state_period_s=_optional_float(raw.get("state_period_s"), "state_period_s"),
        feedback_period_s=_optional_float(raw.get("feedback_period_s"), "feedback_period_s"),
    )


def _parse_motor_config(raw: dict[str, Any]) -> MotorConfig:
    if "id" not in raw:
        raise ValueError("each motor item must contain id")
    motor_id = int(raw["id"])
    if motor_id < 1 or motor_id > 254:
        raise ValueError(f"motor id must be in [1, 254], got {motor_id}")

    fb_id = int(raw.get("fb_id", 0))
    if fb_id not in (0, motor_id):
        raise ValueError(f"motor {motor_id}: fb_id must be 0 or equal to id")

    return MotorConfig(
        id=motor_id,
        fb_id=fb_id,
        name=str(raw.get("name", "")),
        model=str(raw.get("model", "")),
        enabled=bool(raw.get("enabled", True)),
        default_mode=str(raw.get("default_mode", "")),
        max_velocity_rad_s=_optional_float(raw.get("max_velocity_rad_s"), "max_velocity_rad_s"),
        max_current_a=_optional_float(raw.get("max_current_a"), "max_current_a"),
        max_torque_nm=_optional_float(raw.get("max_torque_nm"), "max_torque_nm"),
        feedback_period_s=_optional_float(raw.get("feedback_period_s"), "feedback_period_s"),
        mit_limits=_parse_mit_limits(raw.get("mit_limits", {})),
    )


def _parse_mit_limits(raw: Any) -> MITLimits:
    values = _optional_mapping(raw, "mit_limits")
    return MITLimits(
        position_max_rad=float(values.get("position_max_rad", 95.5)),
        velocity_max_rad_s=float(values.get("velocity_max_rad_s", 45.0)),
        torque_max_nm=float(values.get("torque_max_nm", 18.0)),
    )


def _optional_mapping(raw: Any, name: str) -> dict[str, Any]:
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(f"{name} must be a mapping")
    return dict(raw)


def _optional_float(raw: Any, name: str) -> Optional[float]:
    if raw is None:
        return None
    value = float(raw)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return value
