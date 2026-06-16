import json
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotorDemoNode(Node):
    """通过 hexmovr_moto_manager 控制 Hexmovr 电机的示例节点。

    这个节点故意不直接打开 SocketCAN，而是向 /hexmovr_moto_manager/command
    发布 JSON 命令，并订阅 /hexmovr_moto_manager/state 读取状态。

    为什么这里通过 manager topic 调用，而不是自己打开 can0？
    - 同一条 CAN 总线建议只由一个进程直接控制。
    - hexmovr_moto_manager 已经负责占用总线、扫描电机、解析反馈，并发布
      RViz 和其他工具都能使用的稳定状态 JSON。
    - 其他功能包只需要“发命令、读状态”，代码会更简单，也更不容易抢帧。
    """

    def __init__(self) -> None:
        super().__init__("motor_demo_node")

        # 参数让这个例程不用改源码也能复用。
        # 默认控制 ID 为 1 的电机。
        self.declare_parameter("motor_id", 1)
        self.declare_parameter("command_topic", "/hexmovr_moto_manager/command")
        self.declare_parameter("state_topic", "/hexmovr_moto_manager/state")

        # 安全默认值：除非用户显式设置 demo_enabled=true，否则节点不会主动让电机运动。
        self.declare_parameter("demo_enabled", False)
        self.declare_parameter("demo_mode", "velocity")
        self.declare_parameter("velocity_rad_s", 0.5)
        self.declare_parameter("position_rad", 0.5)
        self.declare_parameter("current_a", 0.2)
        self.declare_parameter("kp", 20.0)
        self.declare_parameter("kd", 0.5)
        self.declare_parameter("torque_nm", 0.0)
        self.declare_parameter("run_seconds", 2.0)

        self.motor_id = int(self.get_parameter("motor_id").value)
        command_topic = str(self.get_parameter("command_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self.demo_enabled = bool(self.get_parameter("demo_enabled").value)
        self.demo_mode = str(self.get_parameter("demo_mode").value).strip().lower()
        self.velocity_rad_s = float(self.get_parameter("velocity_rad_s").value)
        self.position_rad = float(self.get_parameter("position_rad").value)
        self.current_a = float(self.get_parameter("current_a").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.torque_nm = float(self.get_parameter("torque_nm").value)
        self.run_seconds = max(float(self.get_parameter("run_seconds").value), 0.0)

        self.command_pub = self.create_publisher(String, command_topic, 10)
        self.state_sub = self.create_subscription(String, state_topic, self.on_state, 10)

        self.latest_snapshot: Optional[dict[str, Any]] = None
        self._demo_started_at: Optional[float] = None
        self._demo_stop_sent = False

        # 定时器是 ROS 中实现周期行为的常见方式。
        # 这里仅用于可选的演示运动和状态打印。
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f"manager topic 调用例程已就绪。motor_id={self.motor_id}, "
            f"command_topic={command_topic}, state_topic={state_topic}, "
            f"demo_enabled={self.demo_enabled}, demo_mode={self.demo_mode}"
        )

        if self.demo_enabled:
            self.get_logger().warn(
                "演示运动已启用。节点会通过 manager topic 发送控制命令，并在设定时间后释放电机。"
            )
        else:
            self.get_logger().info(
                "演示运动未启用。设置 demo_enabled:=true 后才会发送短时间运动命令。"
            )

    # -------------------------------------------------------------------------
    # 命令发送辅助函数
    # -------------------------------------------------------------------------
    # 下面这些方法就是其他功能包最应该复制的模式：
    # 1. 构造一个 JSON 字典；
    # 2. 序列化成 std_msgs/String；
    # 3. 发布到 /hexmovr_moto_manager/command。

    def publish_command(self, payload: dict[str, Any]) -> None:
        """把一条 manager 命令发布为 JSON 字符串。"""
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.command_pub.publish(msg)

    def send_velocity(self, velocity_rad_s: float) -> None:
        """速度模式控制，单位是 rad/s。"""
        self.publish_command(
            {
                "op": "control",
                "motor_id": self.motor_id,
                "mode": "velocity",
                "velocity_rad_s": float(velocity_rad_s),
            }
        )

    def send_current(self, current_a: float) -> None:
        """电流模式控制，单位是 A。"""
        self.publish_command(
            {
                "op": "control",
                "motor_id": self.motor_id,
                "mode": "current",
                "current_a": float(current_a),
            }
        )

    def send_absolute_position(self, position_rad: float) -> None:
        """绝对位置控制，目标位置单位是 rad。"""
        self.publish_command(
            {
                "op": "control",
                "motor_id": self.motor_id,
                "mode": "absolute_position",
                "position_rad": float(position_rad),
            }
        )

    def send_relative_position(self, delta_rad: float) -> None:
        """相对位置控制，目标位移单位是 rad。"""
        self.publish_command(
            {
                "op": "control",
                "motor_id": self.motor_id,
                "mode": "relative_position",
                "position_rad": float(delta_rad),
            }
        )

    def send_trapezoid_position(self, position_rad: float, relative: bool = False) -> None:
        """梯形位置控制，可选相对位置。"""
        self.publish_command(
            {
                "op": "control",
                "motor_id": self.motor_id,
                "mode": "trapezoid_position",
                "position_rad": float(position_rad),
                "relative": bool(relative),
            }
        )

    def send_position_filter(self, position_rad: float, relative: bool = False) -> None:
        """位置滤波控制，可选相对位置。"""
        self.publish_command(
            {
                "op": "control",
                "motor_id": self.motor_id,
                "mode": "position_filter",
                "position_rad": float(position_rad),
                "relative": bool(relative),
            }
        )

    def send_mit(
        self,
        position_rad: float,
        velocity_rad_s: float,
        stiffness: float,
        damping: float,
        torque_nm: float,
    ) -> None:
        """MIT 模式控制命令。"""
        self.publish_command(
            {
                "op": "control",
                "motor_id": self.motor_id,
                "mode": "mit",
                "position_rad": float(position_rad),
                "velocity_rad_s": float(velocity_rad_s),
                "stiffness": float(stiffness),
                "damping": float(damping),
                "torque_nm": float(torque_nm),
            }
        )

    def clear_error(self) -> None:
        """清除电机故障状态。"""
        self.publish_command({"op": "clear_error", "motor_id": self.motor_id})

    def set_zero(self) -> None:
        """把当前电机位置设置为零点。真机上使用时要确认机械姿态。"""
        self.publish_command({"op": "set_zero", "motor_id": self.motor_id})

    def free_motor(self) -> None:
        """释放电机输出。这个 manager 中通常用它作为安全停止命令。"""
        self.publish_command({"op": "free_motor", "motor_id": self.motor_id})

    # -------------------------------------------------------------------------
    # 状态读取
    # -------------------------------------------------------------------------

    def on_state(self, msg: String) -> None:
        """读取 manager 发布的聚合状态，并保存当前 motor_id 对应的 snapshot。"""
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"状态 JSON 无法解析: {exc}")
            return

        for motor in state.get("motors", []):
            if int(motor.get("motor_id", -1)) != self.motor_id:
                continue
            snapshot = motor.get("snapshot", {})
            if isinstance(snapshot, dict):
                self.latest_snapshot = snapshot
            return

    def log_latest_state(self) -> None:
        """从最新状态中打印一行简洁的电机信息。"""
        if not self.latest_snapshot:
            self.get_logger().info(f"还没有收到 {self.motor_id} 号电机的状态。")
            return

        pos = self.latest_snapshot.get("position_rad")
        vel = self.latest_snapshot.get("velocity_rad_s")
        temp = self.latest_snapshot.get("temperature_c")
        fault = self.latest_snapshot.get("fault_code")
        self.get_logger().info(
            f"motor={self.motor_id} pos={pos} rad vel={vel} rad/s "
            f"temp={temp} C fault={fault}"
        )

    # -------------------------------------------------------------------------
    # 可选演示逻辑
    # -------------------------------------------------------------------------

    def on_timer(self) -> None:
        """可选演示：清错，短时间运行，然后释放电机。"""
        if not self.demo_enabled:
            return

        now = time.monotonic()
        if self._demo_started_at is None:
            self._demo_started_at = now
            self.clear_error()
            self._send_selected_demo_command()
            self.get_logger().info(
                f"已向 {self.motor_id} 号电机发送演示命令："
                f"demo_mode={self.demo_mode}，持续 {self.run_seconds} s"
            )
            return

        elapsed = now - self._demo_started_at
        if elapsed >= self.run_seconds and not self._demo_stop_sent:
            self.free_motor()
            self._demo_stop_sent = True
            self.log_latest_state()
            self.get_logger().info("演示结束，已发送 free_motor 命令。")

    def _send_selected_demo_command(self) -> None:
        """根据 demo_mode 选择 manager 支持的一种控制命令。"""
        if self.demo_mode == "velocity":
            self.send_velocity(self.velocity_rad_s)
        elif self.demo_mode == "current":
            self.send_current(self.current_a)
        elif self.demo_mode in ("position", "absolute_position"):
            self.send_absolute_position(self.position_rad)
        elif self.demo_mode in ("relative_position", "relative"):
            self.send_relative_position(self.position_rad)
        elif self.demo_mode in ("trapezoid", "trapezoid_position"):
            self.send_trapezoid_position(self.position_rad)
        elif self.demo_mode in ("position_filter", "filter"):
            self.send_position_filter(self.position_rad)
        elif self.demo_mode == "mit":
            self.send_mit(
                self.position_rad,
                self.velocity_rad_s,
                self.kp,
                self.kd,
                self.torque_nm,
            )
        else:
            self.get_logger().warning(
                f"未知 demo_mode={self.demo_mode!r}，回退为 velocity 模式。"
            )
            self.send_velocity(self.velocity_rad_s)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MotorDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，准备退出。")
    finally:
        # 如果演示模式正在控制电机，退出时尽量释放电机输出。
        if node.demo_enabled and not node._demo_stop_sent:
            try:
                node.free_motor()
            except Exception as exc:
                node.get_logger().warning(f"退出时发送 free_motor 失败: {exc}")
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
