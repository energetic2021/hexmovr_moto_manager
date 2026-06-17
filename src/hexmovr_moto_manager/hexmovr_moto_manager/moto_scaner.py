import json
import math
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, TransformStamped
from hexmovr_bridge.config import HexmovrConfig, load_hexmovr_config
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
    MarkerArray,
)

from .can_transport import SocketCanError
from .hexmovr_client import HexmovrClient
from .hexmovr_protocol import MITLimits, MotorSnapshot, PositionType

MOTOR_MESH_RESOURCE = "package://hexmovr_moto_panel/meshes/moto.STL"


@dataclass
class ManagedMotorView:
    motor_id: int
    snapshot: MotorSnapshot
    last_seen: float = 0.0
    layout_index: int = 0
    last_error: str = ""
    mit_limits: MITLimits = field(default_factory=MITLimits)

    def is_stale(self, now_s: float, stale_timeout_s: float) -> bool:
        return self.last_seen <= 0.0 or (now_s - self.last_seen) > stale_timeout_s


class HexmovrMotoManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("hexmovr_moto_manager")
        self.declare_parameter("motor_config_file", "")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("scan_start_id", 1)
        self.declare_parameter("scan_end_id", 12)
        self.declare_parameter("poll_period_s", 0.2)
        self.declare_parameter("scan_timeout_s", 0.02)
        self.declare_parameter("request_timeout_s", 0.04)
        self.declare_parameter("stale_timeout_s", 1.0)
        self.declare_parameter("auto_scan_on_start", True)
        self.declare_parameter("deep_refresh_on_scan", True)
        self.declare_parameter("motor_spacing_m", 0.22)
        self.declare_parameter("use_interactive_markers", True)
        self.declare_parameter("show_labels", False)
        self.declare_parameter("jog_step_rad", 0.25)
        self.declare_parameter("velocity_step_rad_s", 1.0)
        self.declare_parameter("retry_connect_period_s", 2.0)

        self._motor_config = self._load_motor_config()
        interface = (
            self._motor_config.channel
            if self._motor_config is not None
            else str(self.get_parameter("can_interface").value)
        )
        timeout_s = float(self.get_parameter("request_timeout_s").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._poll_period_s = float(self.get_parameter("poll_period_s").value)
        self._scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self._stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self._deep_refresh_on_scan = bool(self.get_parameter("deep_refresh_on_scan").value)
        self._motor_spacing_m = float(self.get_parameter("motor_spacing_m").value)
        self._use_interactive_markers = bool(
            self.get_parameter("use_interactive_markers").value
        )
        self._show_labels = bool(self.get_parameter("show_labels").value)
        self._jog_step_rad = float(self.get_parameter("jog_step_rad").value)
        self._velocity_step_rad_s = float(self.get_parameter("velocity_step_rad_s").value)
        self._retry_connect_period_s = float(
            self.get_parameter("retry_connect_period_s").value
        )
        self._can_interface = interface
        self._request_timeout_s = timeout_s
        self._transport_error = ""
        self._workspace_frame = f"{self._frame_id}_workspace"

        self._client: Optional[HexmovrClient] = None
        self._motors: dict[int, ManagedMotorView] = {}
        self._menu_handlers: dict[str, MenuHandler] = {}
        self._menu_actions: dict[str, dict[int, str]] = {}
        self._fault_history: list[dict[str, Any]] = []
        self._fault_history_limit = 500
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._seed_configured_motors()

        self._command_sub = self.create_subscription(
            String,
            "~/command",
            self._on_command_message,
            10,
        )
        self._state_pub = self.create_publisher(String, "~/state", 10)
        self._event_pub = self.create_publisher(String, "~/event", 10)
        self._history_pub = self.create_publisher(String, "~/history", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "~/markers", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._scan_srv = self.create_service(Trigger, "~/scan", self._handle_scan)
        self._refresh_srv = self.create_service(Trigger, "~/refresh", self._handle_refresh)

        self._marker_server: Optional[InteractiveMarkerServer]
        if self._use_interactive_markers:
            self._marker_server = InteractiveMarkerServer(self, "hexmovr_manager")
        else:
            self._marker_server = None

        self._poll_timer = self.create_timer(self._poll_period_s, self._poll_once)
        self._scene_timer = self.create_timer(0.5, self._publish_scene)
        self._reconnect_timer = self.create_timer(
            self._retry_connect_period_s,
            self._retry_client_connection,
        )
        self._startup_scan_timer = None

        self._publish_static_workspace_tf()
        self._connect_client(initial=True)
        self._publish_history()
        if bool(self.get_parameter("auto_scan_on_start").value):
            self._startup_scan_timer = self.create_timer(0.5, self._scan_once_on_start)

        self._emit_event(
            "manager_started",
            {
                "can_interface": interface,
                "frame_id": self._frame_id,
                "poll_period_s": self._poll_period_s,
            },
        )

    def _load_motor_config(self) -> Optional[HexmovrConfig]:
        config_file = str(self.get_parameter("motor_config_file").value).strip()
        if not config_file:
            return None
        config = load_hexmovr_config(config_file)
        self.get_logger().info(
            f"Loaded Hexmovr motor config from {config_file}: "
            f"channel={config.channel}, motors={config.motor_ids}"
        )
        return config

    def _seed_configured_motors(self) -> None:
        if self._motor_config is None:
            return
        for layout_index, motor_config in enumerate(
            motor for motor in self._motor_config.motors if motor.enabled
        ):
            self._motors[motor_config.id] = ManagedMotorView(
                motor_id=motor_config.id,
                snapshot=MotorSnapshot(motor_id=motor_config.id),
                layout_index=layout_index,
                mit_limits=MITLimits(
                    position_max_rad=motor_config.mit_limits.position_max_rad,
                    velocity_max_rad_s=motor_config.mit_limits.velocity_max_rad_s,
                    torque_max_nm=motor_config.mit_limits.torque_max_nm,
                ),
            )

    def destroy_node(self) -> bool:
        if self._marker_server is not None:
            try:
                self._marker_server.clear()
                if rclpy.ok():
                    self._marker_server.applyChanges()
            except Exception as exc:
                self.get_logger().debug(
                    f"Skipped interactive marker cleanup during shutdown: {exc}"
                )
        if self._client is not None:
            self._client.close()
        return super().destroy_node()

    def _connect_client(self, initial: bool = False) -> bool:
        if self._client is not None:
            return True
        try:
            self._client = HexmovrClient(self._can_interface, timeout_s=self._request_timeout_s)
            previous_error = self._transport_error
            self._transport_error = ""
            if previous_error or initial:
                self.get_logger().info(
                    f"Connected to CAN interface '{self._can_interface}'."
                )
                self._emit_event(
                    "can_connected",
                    {"can_interface": self._can_interface},
                )
            return True
        except SocketCanError as exc:
            self._transport_error = str(exc)
            if initial:
                self.get_logger().warning(self._transport_error)
            return False

    def _publish_static_workspace_tf(self) -> None:
        stamp = self.get_clock().now().to_msg()

        map_to_base = TransformStamped()
        map_to_base.header.stamp = stamp
        map_to_base.header.frame_id = self._frame_id
        map_to_base.child_frame_id = "base_link"
        map_to_base.transform.translation.x = 0.0
        map_to_base.transform.translation.y = 0.0
        map_to_base.transform.translation.z = 0.0
        map_to_base.transform.rotation.x = 0.0
        map_to_base.transform.rotation.y = 0.0
        map_to_base.transform.rotation.z = 0.0
        map_to_base.transform.rotation.w = 1.0

        base_to_workspace = TransformStamped()
        base_to_workspace.header.stamp = stamp
        base_to_workspace.header.frame_id = "base_link"
        base_to_workspace.child_frame_id = self._workspace_frame
        base_to_workspace.transform.translation.x = 0.0
        base_to_workspace.transform.translation.y = 0.0
        base_to_workspace.transform.translation.z = 0.0
        base_to_workspace.transform.rotation.x = 0.0
        base_to_workspace.transform.rotation.y = 0.0
        base_to_workspace.transform.rotation.z = 0.0
        base_to_workspace.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform([map_to_base, base_to_workspace])

    def _retry_client_connection(self) -> None:
        if self._client is None:
            self._connect_client(initial=False)
            self._publish_scene()
            self._publish_state()

    def _require_client(self) -> HexmovrClient:
        if self._client is None and not self._connect_client(initial=False):
            raise RuntimeError(self._transport_error or "CAN interface is unavailable")
        assert self._client is not None
        return self._client

    def _scan_once_on_start(self) -> None:
        if self._startup_scan_timer is not None:
            self._startup_scan_timer.cancel()
            self._startup_scan_timer = None
        self._scan_bus()

    def _handle_scan(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        count = self._scan_bus()
        response.success = True
        response.message = f"scan complete, {count} motor(s) found"
        return response

    def _handle_refresh(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        refreshed = 0
        for motor_id in sorted(self._motors):
            self._refresh_motor(motor_id, deep=True)
            refreshed += 1
        response.success = True
        response.message = f"refreshed {refreshed} motor(s)"
        return response

    def _emit_event(self, event: str, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps({"event": event, "payload": payload}, ensure_ascii=True)
        self._event_pub.publish(msg)

    def _publish_history(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "fault_history": self._fault_history,
                "count": len(self._fault_history),
            },
            ensure_ascii=True,
        )
        self._history_pub.publish(msg)

    def _append_fault_history(
        self,
        kind: str,
        motor_id: Optional[int],
        message: str,
        extra: Optional[dict[str, Any]] = None,
    ) -> None:
        entry: dict[str, Any] = {
            "timestamp": time.time(),
            "kind": kind,
            "motor_id": motor_id,
            "message": message,
        }
        if extra:
            entry.update(extra)
        self._fault_history.append(entry)
        if len(self._fault_history) > self._fault_history_limit:
            self._fault_history = self._fault_history[-self._fault_history_limit :]
        self._publish_history()

    def _publish_state(self) -> None:
        payload = {
            "can_interface": self._can_interface,
            "connected": self._client is not None,
            "transport_error": self._transport_error,
            "motor_count": len(self._motors),
            "history_size": len(self._fault_history),
            "motors": [
                {
                    "motor_id": motor_id,
                    "last_seen": view.last_seen,
                    "last_error": view.last_error,
                    "snapshot": view.snapshot.as_dict(),
                }
                for motor_id, view in sorted(self._motors.items())
            ]
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self._state_pub.publish(msg)

    def _scan_bus(self) -> int:
        if self._client is None and not self._connect_client(initial=False):
            self._emit_event(
                "scan_skipped",
                {
                    "can_interface": self._can_interface,
                    "reason": self._transport_error,
                },
            )
            return 0
        client = self._require_client()
        start_id = int(self.get_parameter("scan_start_id").value)
        end_id = int(self.get_parameter("scan_end_id").value)
        found = client.scan(start_id, end_id, timeout_s=self._scan_timeout_s)
        now = time.time()
        for layout_index, motor_id in enumerate(sorted(found)):
            snapshot = found[motor_id]
            existing = self._motors.get(motor_id)
            if existing is None:
                existing = ManagedMotorView(
                    motor_id=motor_id,
                    snapshot=snapshot,
                    last_seen=now,
                    layout_index=layout_index,
                )
                self._motors[motor_id] = existing
            else:
                existing.snapshot = snapshot
                existing.last_seen = now
                existing.layout_index = layout_index
                existing.last_error = ""
            if self._deep_refresh_on_scan:
                self._refresh_motor(motor_id, deep=True)
        self._reindex_motors()
        self._rebuild_interactive_markers()
        self._publish_scene()
        self._publish_state()
        self._emit_event(
            "scan_complete",
            {
                "count": len(found),
                "motor_ids": sorted(found),
                "start_id": start_id,
                "end_id": end_id,
            },
        )
        return len(found)

    def _reindex_motors(self) -> None:
        for layout_index, motor_id in enumerate(sorted(self._motors)):
            self._motors[motor_id].layout_index = layout_index

    def _poll_once(self) -> None:
        if self._client is None:
            self._publish_scene()
            self._publish_state()
            return
        for motor_id in sorted(self._motors):
            self._refresh_motor(motor_id, deep=False)
        self._publish_scene()
        self._publish_state()

    def _refresh_motor(self, motor_id: int, deep: bool) -> None:
        view = self._motors.get(motor_id)
        if view is None:
            return
        previous_fault = view.snapshot.fault_code
        previous_error = view.last_error
        try:
            snapshot = self._require_client().refresh_motor(
                motor_id,
                timeout_s=self._scan_timeout_s if not deep else self._scan_timeout_s * 2.0,
                deep=deep,
            )
            view.snapshot = snapshot
            view.mit_limits = MITLimits(
                position_max_rad=snapshot.mit_position_max_rad,
                velocity_max_rad_s=snapshot.mit_velocity_max_rad_s,
                torque_max_nm=snapshot.mit_torque_max_nm,
            )
            view.last_seen = time.time()
            view.last_error = ""
            if previous_error:
                self._append_fault_history(
                    "communication_restored",
                    motor_id,
                    "Motor communication restored",
                )
            if snapshot.fault_code != previous_fault:
                if snapshot.fault_code:
                    self._append_fault_history(
                        "fault_active",
                        motor_id,
                        f"Fault became active: 0x{snapshot.fault_code:02X}",
                        {"fault_code": snapshot.fault_code},
                    )
                elif previous_fault:
                    self._append_fault_history(
                        "fault_cleared",
                        motor_id,
                        f"Fault cleared: 0x{previous_fault:02X}",
                        {"fault_code": previous_fault},
                    )
        except Exception as exc:
            view.last_error = str(exc)
            if view.last_error != previous_error:
                self._append_fault_history(
                    "communication_error",
                    motor_id,
                    view.last_error,
                )

    def _publish_scene(self) -> None:
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        current_time = time.time()
        if not self._motors:
            hint = Marker()
            hint.header.frame_id = self._workspace_frame
            hint.header.stamp = now
            hint.ns = "hexmovr_hint"
            hint.id = 1
            hint.type = Marker.TEXT_VIEW_FACING
            hint.action = Marker.ADD
            hint.pose.position.x = 0.0
            hint.pose.position.y = 0.0
            hint.pose.position.z = 0.25
            hint.scale.z = 0.08
            hint.color.r = 0.92
            hint.color.g = 0.92
            hint.color.b = 0.92
            hint.color.a = 1.0
            hint.text = "No motors discovered yet\nCall /hexmovr_moto_manager/scan or use Hexmovr Bus"
            marker_array.markers.append(hint)

        for motor_id, view in sorted(self._motors.items()):
            x = view.layout_index * self._motor_spacing_m

            body = Marker()
            body.header.frame_id = self._workspace_frame
            body.header.stamp = now
            body.ns = "hexmovr_body"
            body.id = motor_id
            body.type = Marker.MESH_RESOURCE
            body.action = Marker.ADD
            body.mesh_resource = MOTOR_MESH_RESOURCE
            body.mesh_use_embedded_materials = True
            body.pose.position.x = x
            body.pose.position.y = 0.0
            body.pose.position.z = 0.0
            body.scale.x = 1.0
            body.scale.y = 1.0
            body.scale.z = 1.0
            self._apply_motor_mesh_style(body, view, current_time)
            marker_array.markers.append(body)

            arrow = Marker()
            arrow.header.frame_id = self._workspace_frame
            arrow.header.stamp = now
            arrow.ns = "hexmovr_axis"
            arrow.id = 1000 + motor_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.003
            arrow.scale.y = 0.010
            arrow.scale.z = 0.018
            start = Point(x=x, y=0.0, z=0.070)
            direction = self._arrow_tip(x, view.snapshot.position_rad)
            arrow.points = [start, direction]
            arrow.color.r = 0.90
            arrow.color.g = 0.05
            arrow.color.b = 0.04
            arrow.color.a = 0.95
            marker_array.markers.append(arrow)

            if self._show_labels:
                text = Marker()
                text.header.frame_id = self._workspace_frame
                text.header.stamp = now
                text.ns = "hexmovr_label"
                text.id = 2000 + motor_id
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x
                text.pose.position.y = 0.0
                text.pose.position.z = 0.16
                text.scale.z = 0.035
                text.color.r = 0.1
                text.color.g = 0.1
                text.color.b = 0.1
                text.color.a = 1.0
                text.text = self._label_text(view, current_time)
                marker_array.markers.append(text)

        self._marker_pub.publish(marker_array)
        self._publish_diagnostics(current_time)

    def _publish_diagnostics(self, current_time: float) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        statuses: list[DiagnosticStatus] = []

        bus_status = DiagnosticStatus()
        bus_status.name = f"hexmovr/bus/{self._can_interface}"
        bus_status.hardware_id = self._can_interface
        if self._client is None:
            bus_status.level = DiagnosticStatus.WARN
            bus_status.message = self._transport_error or "CAN interface unavailable"
        else:
            bus_status.level = DiagnosticStatus.OK
            bus_status.message = "connected"
        bus_status.values = [
            KeyValue(key="can_interface", value=self._can_interface),
            KeyValue(key="connected", value=str(self._client is not None).lower()),
            KeyValue(key="transport_error", value=self._transport_error),
        ]
        statuses.append(bus_status)

        for motor_id, view in sorted(self._motors.items()):
            status = DiagnosticStatus()
            status.name = f"hexmovr/motor_{motor_id}"
            stale = view.is_stale(current_time, self._stale_timeout_s)
            if view.last_error:
                status.level = DiagnosticStatus.ERROR
                status.message = view.last_error
            elif view.snapshot.fault_code:
                status.level = DiagnosticStatus.ERROR
                status.message = f"fault=0x{view.snapshot.fault_code:02X}"
            elif stale:
                status.level = DiagnosticStatus.WARN
                status.message = "stale telemetry"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "healthy"
            status.hardware_id = f"hexmovr:{motor_id}"
            status.values = [
                KeyValue(key="position_rad", value=f"{view.snapshot.position_rad:.4f}"),
                KeyValue(key="velocity_rad_s", value=f"{view.snapshot.velocity_rad_s:.4f}"),
                KeyValue(key="q_current_a", value=f"{view.snapshot.q_current_a:.4f}"),
                KeyValue(key="temperature_c", value=str(view.snapshot.temperature_c)),
                KeyValue(key="run_mode", value=str(view.snapshot.run_mode)),
                KeyValue(key="last_seen_age_s", value=f"{max(current_time - view.last_seen, 0.0):.3f}"),
            ]
            statuses.append(status)
        array.status = statuses
        self._diag_pub.publish(array)

    def _rebuild_interactive_markers(self) -> None:
        if self._marker_server is None:
            return
        self._marker_server.clear()
        self._menu_handlers.clear()
        self._menu_actions.clear()

        self._insert_console_marker()
        for motor_id, view in sorted(self._motors.items()):
            self._insert_motor_marker(view)
        self._marker_server.applyChanges()

    def _insert_console_marker(self) -> None:
        if self._marker_server is None:
            return
        marker = InteractiveMarker()
        marker.header.frame_id = self._workspace_frame
        marker.name = "console"
        marker.description = "Hexmovr Bus"
        marker.scale = 0.20
        marker.pose.position.x = -0.25
        marker.pose.position.z = 0.10

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = 0.12
        cube.scale.y = 0.12
        cube.scale.z = 0.12
        cube.color.r = 0.20
        cube.color.g = 0.45
        cube.color.b = 0.85
        cube.color.a = 0.90
        control.markers.append(cube)
        marker.controls.append(control)

        handler = MenuHandler()
        action_map: dict[int, str] = {}
        action_map[handler.insert("Scan Bus")] = "scan_bus"
        action_map[handler.insert("Refresh All")] = "refresh_all"
        self._menu_handlers["console"] = handler
        self._menu_actions["console"] = action_map
        self._marker_server.insert(marker, feedback_callback=self._on_interactive_feedback)
        handler.apply(self._marker_server, marker.name)

    def _insert_motor_marker(self, view: ManagedMotorView) -> None:
        if self._marker_server is None:
            return
        marker = InteractiveMarker()
        marker.header.frame_id = self._workspace_frame
        marker.name = f"motor_{view.motor_id}"
        marker.description = f"Motor {view.motor_id}"
        marker.scale = 0.18
        marker.pose.position.x = view.layout_index * self._motor_spacing_m
        marker.pose.position.z = 0.0

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        body = Marker()
        body.type = Marker.MESH_RESOURCE
        body.mesh_resource = MOTOR_MESH_RESOURCE
        body.mesh_use_embedded_materials = True
        body.scale.x = 1.0
        body.scale.y = 1.0
        body.scale.z = 1.0
        self._apply_motor_mesh_style(body, view, time.time())
        control.markers.append(body)
        marker.controls.append(control)

        self._marker_server.insert(marker, feedback_callback=self._on_interactive_feedback)
        handler = MenuHandler()
        action_map: dict[int, str] = {}
        action_map[handler.insert("Refresh Snapshot")] = "refresh_snapshot"
        action_map[handler.insert("Deep Refresh")] = "deep_refresh"
        action_map[handler.insert("Clear Error")] = "clear_error"
        action_map[handler.insert("Set Zero")] = "set_zero"
        action_map[handler.insert("Return To Zero")] = "return_to_zero"
        action_map[handler.insert("Free Motor")] = "free_motor"
        action_map[handler.insert(f"Jog +{self._jog_step_rad:.2f} rad")] = "jog_positive"
        action_map[handler.insert(f"Jog -{self._jog_step_rad:.2f} rad")] = "jog_negative"
        action_map[handler.insert(f"Spin +{self._velocity_step_rad_s:.2f} rad/s")] = "vel_positive"
        action_map[handler.insert(f"Spin -{self._velocity_step_rad_s:.2f} rad/s")] = "vel_negative"
        action_map[handler.insert("Stop Velocity")] = "vel_zero"
        self._menu_handlers[marker.name] = handler
        self._menu_actions[marker.name] = action_map
        handler.apply(self._marker_server, marker.name)

    def _on_interactive_feedback(self, feedback: Any) -> None:
        marker_name = feedback.marker_name
        if feedback.event_type != feedback.MENU_SELECT:
            return
        action = self._menu_actions.get(marker_name, {}).get(feedback.menu_entry_id)
        if action is None:
            return
        if marker_name == "console":
            if action == "scan_bus":
                count = self._scan_bus()
                self._emit_event("console_action", {"action": action, "count": count})
            elif action == "refresh_all":
                for motor_id in sorted(self._motors):
                    self._refresh_motor(motor_id, deep=True)
                self._publish_scene()
                self._publish_state()
                self._emit_event("console_action", {"action": action})
            return

        motor_id = int(marker_name.split("_", 1)[1])
        self._handle_motor_action(motor_id, action)

    def _handle_motor_action(self, motor_id: int, action: str) -> None:
        view = self._motors.get(motor_id)
        if view is None:
            return
        try:
            if action == "refresh_snapshot":
                self._refresh_motor(motor_id, deep=False)
            elif action == "deep_refresh":
                self._refresh_motor(motor_id, deep=True)
            elif action == "clear_error":
                self._require_client().clear_error(motor_id)
                self._refresh_motor(motor_id, deep=False)
            elif action == "set_zero":
                self._require_client().set_zero(motor_id)
                self._refresh_motor(motor_id, deep=False)
            elif action == "return_to_zero":
                self._require_client().return_to_zero(motor_id)
                self._refresh_motor(motor_id, deep=False)
            elif action == "free_motor":
                self._require_client().free_motor(motor_id)
                self._refresh_motor(motor_id, deep=False)
            elif action == "jog_positive":
                self._require_client().set_relative_position(motor_id, self._jog_step_rad)
                self._refresh_motor(motor_id, deep=False)
            elif action == "jog_negative":
                self._require_client().set_relative_position(motor_id, -self._jog_step_rad)
                self._refresh_motor(motor_id, deep=False)
            elif action == "vel_positive":
                self._require_client().set_velocity(motor_id, self._velocity_step_rad_s)
                self._refresh_motor(motor_id, deep=False)
            elif action == "vel_negative":
                self._require_client().set_velocity(motor_id, -self._velocity_step_rad_s)
                self._refresh_motor(motor_id, deep=False)
            elif action == "vel_zero":
                self._require_client().set_velocity(motor_id, 0.0)
                self._refresh_motor(motor_id, deep=False)
            self._publish_scene()
            self._publish_state()
            self._emit_event("motor_action", {"motor_id": motor_id, "action": action})
        except Exception as exc:
            view.last_error = str(exc)
            self._emit_event(
                "motor_action_error",
                {"motor_id": motor_id, "action": action, "error": str(exc)},
            )

    def _on_command_message(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("command payload must be a JSON object")
            result = self._execute_json_command(payload)
            self._publish_scene()
            self._publish_state()
            self._emit_event("command_result", result)
        except Exception as exc:
            self._emit_event("command_error", {"error": str(exc), "raw": msg.data})

    def _execute_json_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        op = str(payload.get("op", "")).lower()
        if op == "scan":
            start_id = int(payload.get("start_id", self.get_parameter("scan_start_id").value))
            end_id = int(payload.get("end_id", self.get_parameter("scan_end_id").value))
            self.set_parameters(
                [
                    Parameter("scan_start_id", value=start_id),
                    Parameter("scan_end_id", value=end_id),
                ]
            )
            count = self._scan_bus()
            return {"op": op, "count": count}
        if op == "refresh_all":
            refreshed = 0
            for motor_id in sorted(self._motors):
                self._refresh_motor(motor_id, deep=bool(payload.get("deep", True)))
                refreshed += 1
            return {"op": op, "refreshed": refreshed}
        if op == "clear_history":
            self._fault_history = []
            self._publish_history()
            return {"op": op, "cleared": True}
        if op == "batch":
            return self._execute_batch_command(payload)

        motor_id = int(payload["motor_id"])
        if motor_id not in self._motors:
            self._motors[motor_id] = ManagedMotorView(
                motor_id=motor_id,
                snapshot=MotorSnapshot(motor_id=motor_id),
                layout_index=len(self._motors),
            )
            self._reindex_motors()

        if op == "refresh":
            deep = bool(payload.get("deep", True))
            self._refresh_motor(motor_id, deep=deep)
            return {"op": op, "motor_id": motor_id, "deep": deep}
        if op == "clear_error":
            self._require_client().clear_error(motor_id)
            self._refresh_motor(motor_id, deep=False)
            return {"op": op, "motor_id": motor_id}
        if op == "set_zero":
            self._require_client().set_zero(motor_id)
            self._refresh_motor(motor_id, deep=False)
            return {"op": op, "motor_id": motor_id}
        if op == "free_motor":
            self._require_client().free_motor(motor_id)
            self._refresh_motor(motor_id, deep=False)
            return {"op": op, "motor_id": motor_id}
        if op == "return_to_zero":
            self._require_client().return_to_zero(motor_id)
            self._refresh_motor(motor_id, deep=False)
            return {"op": op, "motor_id": motor_id}
        if op == "brake":
            raise ValueError("brake control is disabled because the brake hardware is not available")
        if op == "control":
            mode = str(payload.get("mode", "absolute_position")).lower()
            return self._execute_control_command(motor_id, mode, payload)
        if op == "set_param":
            group = str(payload.get("group", "control")).lower()
            name = str(payload["name"])
            value = float(payload["value"])
            if group == "control":
                self._require_client().write_control_param(motor_id, name, value)
            else:
                self._require_client().write_advanced_param(motor_id, name, value)
            self._refresh_motor(motor_id, deep=True)
            return {"op": op, "motor_id": motor_id, "group": group, "name": name, "value": value}
        if op == "set_device_address":
            device_address = int(payload["device_address"])
            self._require_client().write_device_address(motor_id, device_address)
            self._refresh_motor(motor_id, deep=True)
            return {"op": op, "motor_id": motor_id, "device_address": device_address}
        if op == "set_can_timeout":
            enabled = bool(payload.get("enabled", True))
            timeout_ms = int(payload.get("timeout_ms", 100))
            action_flags = int(payload.get("action_flags", 0))
            self._require_client().write_can_timeout(
                motor_id, enabled, timeout_ms, action_flags
            )
            self._refresh_motor(motor_id, deep=True)
            return {
                "op": op,
                "motor_id": motor_id,
                "enabled": enabled,
                "timeout_ms": timeout_ms,
                "action_flags": action_flags,
            }
        if op == "set_mit_limits":
            limits = MITLimits(
                position_max_rad=float(payload.get("position_max_rad", 95.5)),
                velocity_max_rad_s=float(payload.get("velocity_max_rad_s", 45.0)),
                torque_max_nm=float(payload.get("torque_max_nm", 18.0)),
            )
            self._require_client().write_mit_limits(motor_id, limits)
            self._motors[motor_id].mit_limits = limits
            self._refresh_motor(motor_id, deep=True)
            return {"op": op, "motor_id": motor_id, "limits": limits.as_dict()}
        raise ValueError(f"unsupported command op: {op}")

    def _resolve_batch_target_ids(self, payload: dict[str, Any]) -> list[int]:
        if bool(payload.get("all", False)):
            return sorted(self._motors)
        raw_ids = payload.get("motor_ids", [])
        if not isinstance(raw_ids, list):
            raise ValueError("batch motor_ids must be a JSON array")
        return sorted({int(motor_id) for motor_id in raw_ids})

    def _execute_batch_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        target_ids = self._resolve_batch_target_ids(payload)
        command = payload.get("command")
        if not isinstance(command, dict):
            raise ValueError("batch command must contain a JSON object field named command")
        command_op = str(command.get("op", "")).lower()
        if command_op in ("", "scan", "batch", "refresh_all", "clear_history"):
            raise ValueError(f"unsupported nested batch op: {command_op or '<empty>'}")

        results: list[dict[str, Any]] = []
        for motor_id in target_ids:
            nested = dict(command)
            nested["motor_id"] = motor_id
            results.append(self._execute_json_command(nested))
        return {
            "op": "batch",
            "count": len(results),
            "motor_ids": target_ids,
            "results": results,
        }

    def _execute_control_command(
        self, motor_id: int, mode: str, payload: dict[str, Any]
    ) -> dict[str, Any]:
        client = self._require_client()
        if mode == "current":
            current_a = float(payload.get("current_a", 0.0))
            client.set_current(motor_id, current_a)
            self._refresh_motor(motor_id, deep=False)
            return {"op": "control", "motor_id": motor_id, "mode": mode, "current_a": current_a}
        if mode == "velocity":
            velocity_rad_s = float(payload.get("velocity_rad_s", 0.0))
            client.set_velocity(motor_id, velocity_rad_s)
            self._refresh_motor(motor_id, deep=False)
            return {
                "op": "control",
                "motor_id": motor_id,
                "mode": mode,
                "velocity_rad_s": velocity_rad_s,
            }
        if mode == "absolute_position":
            position_rad = float(payload.get("position_rad", 0.0))
            client.set_absolute_position(motor_id, position_rad)
            self._refresh_motor(motor_id, deep=False)
            return {
                "op": "control",
                "motor_id": motor_id,
                "mode": mode,
                "position_rad": position_rad,
            }
        if mode == "relative_position":
            position_rad = float(payload.get("position_rad", 0.0))
            client.set_relative_position(motor_id, position_rad)
            self._refresh_motor(motor_id, deep=False)
            return {
                "op": "control",
                "motor_id": motor_id,
                "mode": mode,
                "position_rad": position_rad,
            }
        if mode == "trapezoid_position":
            position_rad = float(payload.get("position_rad", 0.0))
            pos_type = PositionType.RELATIVE if bool(payload.get("relative", False)) else PositionType.ABSOLUTE
            client.set_trapezoid_position(motor_id, position_rad, pos_type)
            self._refresh_motor(motor_id, deep=False)
            return {
                "op": "control",
                "motor_id": motor_id,
                "mode": mode,
                "position_rad": position_rad,
                "relative": bool(payload.get("relative", False)),
            }
        if mode == "position_filter":
            position_rad = float(payload.get("position_rad", 0.0))
            pos_type = PositionType.RELATIVE if bool(payload.get("relative", False)) else PositionType.ABSOLUTE
            client.set_position_filter(motor_id, position_rad, pos_type)
            self._refresh_motor(motor_id, deep=False)
            return {
                "op": "control",
                "motor_id": motor_id,
                "mode": mode,
                "position_rad": position_rad,
                "relative": bool(payload.get("relative", False)),
            }
        if mode == "mit":
            limits = self._motors[motor_id].mit_limits
            position_rad = float(payload.get("position_rad", 0.0))
            velocity_rad_s = float(payload.get("velocity_rad_s", 0.0))
            stiffness = float(payload.get("stiffness", 30.0))
            damping = float(payload.get("damping", 1.0))
            torque_nm = float(payload.get("torque_nm", 0.0))
            client.set_mit_control(
                motor_id,
                position_rad,
                velocity_rad_s,
                stiffness,
                damping,
                torque_nm,
                limits=limits,
            )
            self._refresh_motor(motor_id, deep=False)
            return {
                "op": "control",
                "motor_id": motor_id,
                "mode": mode,
                "position_rad": position_rad,
                "velocity_rad_s": velocity_rad_s,
                "stiffness": stiffness,
                "damping": damping,
                "torque_nm": torque_nm,
            }
        raise ValueError(f"unsupported control mode: {mode}")

    def _state_color(self, view: ManagedMotorView, current_time: float) -> tuple[float, float, float]:
        if view.last_error or view.snapshot.fault_code:
            return (0.85, 0.22, 0.22)
        if view.is_stale(current_time, self._stale_timeout_s):
            return (0.90, 0.66, 0.18)
        return (0.18, 0.68, 0.33)

    def _apply_motor_mesh_style(
        self,
        marker: Marker,
        view: ManagedMotorView,
        current_time: float,
    ) -> None:
        if view.last_error or view.snapshot.fault_code:
            marker.mesh_use_embedded_materials = False
            marker.color.r = 0.85
            marker.color.g = 0.05
            marker.color.b = 0.04
            marker.color.a = 1.0
            return
        if view.is_stale(current_time, self._stale_timeout_s):
            marker.mesh_use_embedded_materials = False
            marker.color.r = 1.0
            marker.color.g = 0.72
            marker.color.b = 0.08
            marker.color.a = 1.0
            return

        marker.mesh_use_embedded_materials = True
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

    def _label_text(self, view: ManagedMotorView, current_time: float) -> str:
        age = max(current_time - view.last_seen, 0.0)
        return (
            f"ID {view.motor_id} | mode {view.snapshot.run_mode} | fault 0x{view.snapshot.fault_code:02X}\n"
            f"pos {view.snapshot.position_rad:+.3f} rad | vel {view.snapshot.velocity_rad_s:+.3f} rad/s\n"
            f"Iq {view.snapshot.q_current_a:+.3f} A | temp {view.snapshot.temperature_c} C | age {age:.2f}s"
        )

    def _arrow_tip(self, x: float, position_rad: float) -> Point:
        length = 0.055
        clamped = max(min(position_rad, math.pi), -math.pi)
        return Point(
            x=x + math.cos(clamped) * length,
            y=math.sin(clamped) * length,
            z=0.070,
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HexmovrMotoManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，准备退出。")
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
