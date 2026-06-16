import time
from typing import Optional

import rclpy
from rclpy.node import Node

from hexmovr_bridge.controller import Controller


class Motor6DirectLibraryNode(Node):
    """直接实例化 hexmovr_bridge.Controller 控制 6 号电机的示例节点。

    这个例程不通过 /hexmovr_moto_manager/command，也不依赖 manager 节点。
    它会在当前节点内部直接打开 SocketCAN 接口，例如 can0。

    使用这个模式时要特别注意：
    - 当前节点会成为 CAN 总线拥有者。
    - 同一条 CAN 总线上不要同时运行 hexmovr_moto_manager 或 hexmovr_bridge 节点。
    - 如果多个业务节点都要共享电机，优先使用 headless manager 模式。
    """

    def __init__(self) -> None:
        super().__init__("motor6_direct_library_node")

        # 默认控制 6 号电机，默认使用 can0。
        self.declare_parameter("channel", "can0")
        self.declare_parameter("motor_id", 6)

        # 安全默认值：启动节点后不主动运动。
        # 只有 demo_enabled=true 时，才发送短时间速度命令。
        self.declare_parameter("demo_enabled", False)
        self.declare_parameter("velocity_rad_s", 0.5)
        self.declare_parameter("run_seconds", 2.0)

        self.channel = str(self.get_parameter("channel").value)
        self.motor_id = int(self.get_parameter("motor_id").value)
        self.demo_enabled = bool(self.get_parameter("demo_enabled").value)
        self.velocity_rad_s = float(self.get_parameter("velocity_rad_s").value)
        self.run_seconds = max(float(self.get_parameter("run_seconds").value), 0.0)

        # 这里就是“直接引入库并实例化”的关键：
        # Controller 会打开 SocketCAN，并启动后台 RX 线程接收电机反馈。
        self.controller = Controller(self.channel)
        self.motor = self.controller.add_motor(self.motor_id)

        self._demo_started_at: Optional[float] = None
        self._demo_stop_sent = False
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f"直接库调用例程已启动：channel={self.channel}, "
            f"motor_id={self.motor_id}, demo_enabled={self.demo_enabled}"
        )
        self.get_logger().warn(
            "当前节点会直接占用 CAN 总线；不要同时运行 manager/bridge 控制同一个 can 接口。"
        )
        if self.demo_enabled:
            self.get_logger().warn("演示运动已启用，节点会发送速度命令。")
        else:
            self.get_logger().info("演示运动未启用；如需运动，请显式设置 demo_enabled:=true。")

    def send_velocity(self, velocity_rad_s: float) -> None:
        """直接通过 HexmovrMotor API 发送速度控制。"""
        self.motor.send_vel(float(velocity_rad_s))

    def send_absolute_position_with_speed(self, position_rad: float, max_speed_rad_s: float) -> None:
        """直接发送“最大速度 + 绝对位置”控制。"""
        self.motor.send_pos_vel(float(position_rad), float(max_speed_rad_s))

    def send_mit(
        self,
        position_rad: float,
        velocity_rad_s: float,
        stiffness: float,
        damping: float,
        torque_nm: float,
    ) -> None:
        """直接发送 MIT 控制。"""
        self.motor.send_mit(
            float(position_rad),
            float(velocity_rad_s),
            float(stiffness),
            float(damping),
            float(torque_nm),
        )

    def request_and_log_state(self) -> None:
        """请求一次反馈，并打印当前缓存状态。"""
        self.motor.request_feedback()
        state = self.motor.latest_state()
        if state is None:
            self.get_logger().info(f"还没有收到 {self.motor_id} 号电机状态。")
            return
        self.get_logger().info(
            f"motor={state.can_id} pos={state.pos} rad vel={state.vel} rad/s "
            f"temp={state.t_mos} C fault={state.status_code}"
        )

    def stop_motor(self) -> None:
        """释放电机输出。"""
        self.motor.disable()

    def safe_stop_motor(self) -> None:
        """尽量释放电机，但不要在 Ctrl+C 退出时继续抛出长 traceback。"""
        try:
            self.stop_motor()
        except KeyboardInterrupt:
            self.get_logger().warning("停止电机时再次收到 Ctrl+C，跳过剩余停止流程。")
        except Exception as exc:
            self.get_logger().error(f"停止电机失败: {exc}")

    def on_timer(self) -> None:
        """可选演示：短时间速度控制，然后释放电机。"""
        if not self.demo_enabled:
            return

        now = time.monotonic()
        if self._demo_started_at is None:
            self._demo_started_at = now
            self.motor.clear_error()
            self.send_velocity(self.velocity_rad_s)
            self.get_logger().info(
                f"已直接向 {self.motor_id} 号电机发送速度："
                f"{self.velocity_rad_s} rad/s，持续 {self.run_seconds} s"
            )
            return

        elapsed = now - self._demo_started_at
        if elapsed >= self.run_seconds and not self._demo_stop_sent:
            self.safe_stop_motor()
            self.request_and_log_state()
            self._demo_stop_sent = True
            self.get_logger().info("直接库调用演示结束，已释放电机。")

    def destroy_node(self) -> bool:
        # 节点退出时关闭后台 RX 线程，并关闭 SocketCAN。
        try:
            self.controller.shutdown()
        except Exception as exc:
            self.get_logger().error(f"关闭 CAN 控制器失败: {exc}")
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Motor6DirectLibraryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，准备退出。")
    finally:
        if node.demo_enabled and not node._demo_stop_sent:
            node.safe_stop_motor()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
