from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .controller import Controller
from .protocol import (
    ADVANCED_PARAM_NAMES,
    CONTROL_PARAM_NAMES,
    MITLimits,
    AdvancedParam,
    ControlParam,
    PositionType,
)


@dataclass
class ContinuousCommand:
    op: str
    payload: dict[str, Any]


class HexmovrBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("hexmovr_bridge")
        self.declare_parameter("channel", "can0")
        self.declare_parameter("motor_ids", [1])
        self.declare_parameter("model", "")
        self.declare_parameter("control_period_s", 0.02)
        self.declare_parameter("state_period_s", 0.05)
        self.declare_parameter("feedback_period_s", 0.1)

        self._channel = str(self.get_parameter("channel").value)
        self._model = str(self.get_parameter("model").value)
        self._control_period_s = max(float(self.get_parameter("control_period_s").value), 0.001)
        self._state_period_s = max(float(self.get_parameter("state_period_s").value), 0.001)
        self._feedback_period_s = max(float(self.get_parameter("feedback_period_s").value), 0.001)

        self._controller: Optional[Controller] = None
        self._continuous: dict[int, ContinuousCommand] = {}
        self._last_state_publish_s = 0.0
        self._last_feedback_request_s = 0.0

        self._cmd_sub = self.create_subscription(String, "/hexmovr/cmd", self._on_cmd, 10)
        self._state_pub = self.create_publisher(String, "/hexmovr/state", 10)
        self._event_pub = self.create_publisher(String, "/hexmovr/event", 10)
        self._timer = self.create_timer(self._control_period_s, self._tick)

        try:
            self._controller = Controller(self._channel)
            for motor_id in self._configured_motor_ids():
                self._controller.add_motor(motor_id, model=self._model)
            self._emit_event("bridge_started", {"channel": self._channel})
        except Exception as exc:
            self._emit_event("error", {"where": "start", "message": str(exc)})
            self.get_logger().error(f"Failed to start Hexmovr bridge: {exc}")

    def destroy_node(self) -> bool:
        self._shutdown_controller()
        return super().destroy_node()

    def __del__(self) -> None:
        self._shutdown_controller()

    def _configured_motor_ids(self) -> list[int]:
        value = self.get_parameter("motor_ids").value
        if isinstance(value, (list, tuple)):
            return [int(item) for item in value]
        if isinstance(value, str):
            return [int(item.strip()) for item in value.split(",") if item.strip()]
        return [int(value)]

    def _shutdown_controller(self) -> None:
        controller = self._controller
        self._controller = None
        if controller is not None:
            controller.shutdown()

    def _on_cmd(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("command JSON must be an object")
            self._handle_command(payload)
        except Exception as exc:
            self._emit_event("error", {"where": "cmd", "message": str(exc), "raw": msg.data})

    def _handle_command(self, payload: dict[str, Any]) -> None:
        controller = self._require_controller()
        op = str(payload.get("op", "")).strip().lower()
        motor_id = int(payload.get("id", payload.get("motor_id", payload.get("can_id", 1))))
        motor = controller.get_motor(motor_id)
        if motor is None:
            motor = controller.add_motor(motor_id, model=str(payload.get("model", self._model)))

        if op == "mit":
            motor.send_mit(
                float(payload.get("pos", 0.0)),
                float(payload.get("vel", 0.0)),
                float(payload.get("kp", 0.0)),
                float(payload.get("kd", 0.0)),
                float(payload.get("tau", payload.get("torq", 0.0))),
            )
            self._continuous[motor_id] = ContinuousCommand(op, dict(payload))
        elif op == "pos_vel":
            motor.send_pos_vel(float(payload.get("pos", 0.0)), float(payload.get("vel", 0.0)))
            self._continuous[motor_id] = ContinuousCommand(op, dict(payload))
        elif op == "vel":
            motor.send_vel(float(payload.get("vel", 0.0)))
            self._continuous[motor_id] = ContinuousCommand(op, dict(payload))
        elif op in ("current", "current_control"):
            motor.send_current(float(payload.get("current", payload.get("current_a", 0.0))))
            self._continuous[motor_id] = ContinuousCommand(op, dict(payload))
        elif op in ("relative_pos", "rel_pos"):
            motor.send_relative_pos(float(payload.get("pos", payload.get("position", 0.0))))
        elif op in ("return_zero", "return_to_zero"):
            motor.return_to_zero()
        elif op in ("trapezoid_pos", "trapezoid_position"):
            motor.send_trapezoid_pos(
                float(payload.get("pos", payload.get("position", 0.0))),
                self._position_type(payload),
            )
            self._continuous[motor_id] = ContinuousCommand(op, dict(payload))
        elif op in ("position_filter", "filter_pos"):
            motor.send_position_filter(
                float(payload.get("pos", payload.get("position", 0.0))),
                self._position_type(payload),
            )
            self._continuous[motor_id] = ContinuousCommand(op, dict(payload))
        elif op in ("set_param", "write_param"):
            param = self._control_param(payload)
            motor.set_control_param(param, float(payload.get("value", 0.0)))
        elif op in ("set_advanced_param", "write_advanced_param"):
            param = self._advanced_param(payload)
            motor.set_advanced_param(param, float(payload.get("value", 0.0)))
        elif op == "set_can_timeout":
            motor.set_can_timeout(
                bool(payload.get("enabled", True)),
                int(payload.get("timeout_ms", 0)),
                int(payload.get("action_flags", 0)),
            )
        elif op == "set_device_address":
            motor.set_device_address(int(payload.get("device_address", payload.get("address", 0))))
        elif op == "set_mit_limits":
            motor.set_mit_limits(
                MITLimits(
                    position_max_rad=float(payload.get("position_max_rad", 95.5)),
                    velocity_max_rad_s=float(payload.get("velocity_max_rad_s", 45.0)),
                    torque_max_nm=float(payload.get("torque_max_nm", 18.0)),
                )
            )
        elif op == "enable":
            motor.enable()
        elif op in ("disable", "stop"):
            self._continuous.pop(motor_id, None)
            motor.disable()
        elif op == "set_zero":
            motor.set_zero()
        elif op == "clear_error":
            motor.clear_error()
        elif op in ("feedback", "request_feedback"):
            motor.request_feedback()
        else:
            raise ValueError(f"unsupported op: {op!r}")

        self._emit_event("control_ok", {"op": op, "motor_id": motor_id})

    def _tick(self) -> None:
        controller = self._controller
        if controller is None:
            self._publish_no_value()
            return

        for motor_id, command in list(self._continuous.items()):
            motor = controller.get_motor(motor_id)
            if motor is None:
                self._continuous.pop(motor_id, None)
                continue
            try:
                self._repeat_command(motor, command)
            except Exception as exc:
                self._continuous.pop(motor_id, None)
                self._emit_event(
                    "error",
                    {"where": "tick", "motor_id": motor_id, "message": str(exc)},
                )

        now = time.time()
        if now - self._last_feedback_request_s >= self._feedback_period_s:
            for motor in controller.motors():
                try:
                    motor.request_feedback()
                except Exception as exc:
                    self._emit_event(
                        "error",
                        {"where": "feedback", "motor_id": motor.id, "message": str(exc)},
                    )
            self._last_feedback_request_s = now

        if now - self._last_state_publish_s >= self._state_period_s:
            self._publish_states(controller)
            self._last_state_publish_s = now

    def _repeat_command(self, motor, command: ContinuousCommand) -> None:
        payload = command.payload
        if command.op == "mit":
            motor.send_mit(
                float(payload.get("pos", 0.0)),
                float(payload.get("vel", 0.0)),
                float(payload.get("kp", 0.0)),
                float(payload.get("kd", 0.0)),
                float(payload.get("tau", payload.get("torq", 0.0))),
            )
        elif command.op == "pos_vel":
            motor.send_pos_vel(float(payload.get("pos", 0.0)), float(payload.get("vel", 0.0)))
        elif command.op == "vel":
            motor.send_vel(float(payload.get("vel", 0.0)))
        elif command.op in ("current", "current_control"):
            motor.send_current(float(payload.get("current", payload.get("current_a", 0.0))))
        elif command.op in ("trapezoid_pos", "trapezoid_position"):
            motor.send_trapezoid_pos(
                float(payload.get("pos", payload.get("position", 0.0))),
                self._position_type(payload),
            )
        elif command.op in ("position_filter", "filter_pos"):
            motor.send_position_filter(
                float(payload.get("pos", payload.get("position", 0.0))),
                self._position_type(payload),
            )

    def _position_type(self, payload: dict[str, Any]) -> PositionType:
        value = str(payload.get("position_type", payload.get("type", "absolute"))).strip().lower()
        if value in ("relative", "rel", "1"):
            return PositionType.RELATIVE
        return PositionType.ABSOLUTE

    def _control_param(self, payload: dict[str, Any]) -> ControlParam:
        value = payload.get("param", payload.get("name", ""))
        if isinstance(value, str):
            key = value.strip().lower()
            if key in CONTROL_PARAM_NAMES:
                return CONTROL_PARAM_NAMES[key]
        return ControlParam(int(value))

    def _advanced_param(self, payload: dict[str, Any]) -> AdvancedParam:
        value = payload.get("param", payload.get("name", ""))
        if isinstance(value, str):
            key = value.strip().lower()
            if key in ADVANCED_PARAM_NAMES:
                return ADVANCED_PARAM_NAMES[key]
        return AdvancedParam(int(value))

    def _publish_states(self, controller: Controller) -> None:
        motors = controller.motors()
        if not motors:
            self._publish_no_value()
            return
        for motor in motors:
            state = motor.latest_state()
            payload: dict[str, Any]
            if state is None:
                payload = {
                    "has_value": False,
                    "can_id": motor.id,
                    "model": motor.model,
                }
            else:
                payload = state.as_dict()
            payload["stamp"] = time.time()
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=True)
            self._state_pub.publish(msg)

    def _publish_no_value(self) -> None:
        msg = String()
        msg.data = json.dumps({"has_value": False, "stamp": time.time()}, ensure_ascii=True)
        self._state_pub.publish(msg)

    def _emit_event(self, event: str, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps({"event": event, "payload": payload}, ensure_ascii=True)
        self._event_pub.publish(msg)

    def _require_controller(self) -> Controller:
        if self._controller is None:
            self._controller = Controller(self._channel)
        return self._controller


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HexmovrBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
