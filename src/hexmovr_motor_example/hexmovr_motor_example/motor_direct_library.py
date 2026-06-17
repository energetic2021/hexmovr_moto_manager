import time
from typing import Optional

import rclpy
from rclpy.node import Node

from hexmovr_bridge.config import HexmovrConfig, load_hexmovr_config
from hexmovr_bridge.controller import Controller
from hexmovr_bridge.protocol import AdvancedParam, ControlParam, MITLimits, Opcode, PositionType


class MotorDirectLibraryNode(Node):
    """直接实例化 hexmovr_bridge.Controller 控制 Hexmovr 电机的示例节点。

    这个例程直接调用 hexmovr_bridge，不通过 /hexmovr_moto_manager/command，
    也不依赖 manager 节点。
    它会在当前节点内部直接打开 SocketCAN 接口，例如 can0。

    使用这个模式时要特别注意：
    - 当前节点会成为 CAN 总线拥有者。
    - 同一条 CAN 总线上不要同时运行 hexmovr_moto_manager 或 hexmovr_bridge 节点。
    - 如果多个业务节点都要共享电机，优先使用 headless manager 模式。
    - 如果要学习 manager/topic 调用方式，请看 motor_demo.py。
    - 参数写入类命令默认关闭，真机使用前要确认电机型号和机械安全边界。
    """

    def __init__(self) -> None:
        super().__init__("motor_direct_library_node")

        # 默认控制 1 号电机，默认使用 can0。
        self.declare_parameter("config_file", "")
        self.declare_parameter("channel", "can0")
        self.declare_parameter("motor_id", 1)

        # 安全默认值：启动节点后不主动运动。
        # 只有 demo_enabled=true 时，才会根据 demo_mode 发送短时间运动命令。
        self.declare_parameter("demo_enabled", False)
        self.declare_parameter("demo_mode", "velocity")
        self.declare_parameter("velocity_rad_s", 0.5)
        self.declare_parameter("position_rad", 0.5)
        self.declare_parameter("max_speed_rad_s", 0.5)
        self.declare_parameter("current_a", 0.2)
        self.declare_parameter("kp", 20.0)
        self.declare_parameter("kd", 0.5)
        self.declare_parameter("torque_nm", 0.0)
        self.declare_parameter("run_seconds", 2.0)
        self.declare_parameter("demo_repeat_period_s", 0.1)

        # 进阶参数写入示例默认关闭。
        # 打开后会写入一组温和示例参数，用于展示 API 形式。
        self.declare_parameter("configure_params", False)
        self.declare_parameter("mit_position_max_rad", 95.5)
        self.declare_parameter("mit_velocity_max_rad_s", 45.0)
        self.declare_parameter("mit_torque_max_nm", 18.0)

        self.config = self._load_config()
        self.channel = self.config.channel if self.config else str(self.get_parameter("channel").value)
        self.motor_id = int(self.get_parameter("motor_id").value)
        self.demo_enabled = bool(self.get_parameter("demo_enabled").value)
        self.demo_mode = str(self.get_parameter("demo_mode").value).strip().lower()
        self.velocity_rad_s = float(self.get_parameter("velocity_rad_s").value)
        self.position_rad = float(self.get_parameter("position_rad").value)
        self.max_speed_rad_s = float(self.get_parameter("max_speed_rad_s").value)
        self.current_a = float(self.get_parameter("current_a").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.torque_nm = float(self.get_parameter("torque_nm").value)
        self.run_seconds = max(float(self.get_parameter("run_seconds").value), 0.0)
        self.demo_repeat_period_s = max(float(self.get_parameter("demo_repeat_period_s").value), 0.02)
        self.configure_params = bool(self.get_parameter("configure_params").value)

        # 这里就是“直接引入库并实例化”的关键：
        # Controller 会打开 SocketCAN，并启动后台 RX 线程接收电机反馈。
        self.controller = Controller(self.channel)
        self.motor = self.controller.add_motor(self.motor_id)
        motor_config = self.config.motor_by_id(self.motor_id) if self.config else None
        if motor_config is not None:
            self.motor.configure_mit_limits(motor_config.mit_limits)

        self._demo_started_at: Optional[float] = None
        self._last_demo_send_at = 0.0
        self._demo_stop_sent = False
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f"直接库调用例程已启动：channel={self.channel}, "
            f"motor_id={self.motor_id}, demo_enabled={self.demo_enabled}, "
            f"demo_mode={self.demo_mode}"
        )
        self.get_logger().warn(
            "当前节点会直接占用 CAN 总线；不要同时运行 manager/bridge 控制同一个 can 接口。"
        )
        if self.configure_params:
            self.get_logger().warn("参数写入示例已启用，启动后会向电机写入示例配置。")
        if self.demo_enabled:
            self.get_logger().warn("演示运动已启用，节点会根据 demo_mode 发送运动命令。")
        else:
            self.get_logger().info("演示运动未启用；如需运动，请显式设置 demo_enabled:=true。")

    def _load_config(self) -> Optional[HexmovrConfig]:
        """读取可选 YAML 配置；没有配置文件时返回 None。"""
        config_file = str(self.get_parameter("config_file").value).strip()
        if not config_file:
            return None
        config = load_hexmovr_config(config_file)
        self.get_logger().info(
            f"已读取 Hexmovr 配置：file={config_file}, "
            f"channel={config.channel}, motors={config.motor_ids}"
        )
        return config

    # -------------------------------------------------------------------------
    # 基础控制 API 示例
    # -------------------------------------------------------------------------

    def send_velocity(self, velocity_rad_s: float) -> None:
        """直接通过 HexmovrMotor API 发送速度控制。"""
        self.motor.send_vel(float(velocity_rad_s))

    def send_current(self, current_a: float) -> None:
        """直接发送 q 轴电流控制，单位 A。"""
        self.motor.send_current(float(current_a))

    def send_absolute_position_with_speed(self, position_rad: float, max_speed_rad_s: float) -> None:
        """直接发送“最大速度 + 绝对位置”控制。"""
        self.motor.send_pos_vel(float(position_rad), float(max_speed_rad_s))

    def send_relative_position(self, delta_rad: float) -> None:
        """直接发送相对位置控制，单位 rad。"""
        self.motor.send_relative_pos(float(delta_rad))

    def send_trapezoid_position(self, position_rad: float, relative: bool = False) -> None:
        """直接发送梯形位置控制，可选择绝对/相对位置。"""
        position_type = PositionType.RELATIVE if relative else PositionType.ABSOLUTE
        self.motor.send_trapezoid_pos(float(position_rad), position_type)

    def send_position_filter(self, position_rad: float, relative: bool = False) -> None:
        """直接发送位置滤波控制，可选择绝对/相对位置。"""
        position_type = PositionType.RELATIVE if relative else PositionType.ABSOLUTE
        self.motor.send_position_filter(float(position_rad), position_type)

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

    # -------------------------------------------------------------------------
    # 参数配置 API 示例
    # -------------------------------------------------------------------------

    def configure_example_params(self) -> None:
        """写入一组示例参数。

        这个函数只在 configure_params=true 时调用，用来演示参数写入 API。
        实际项目中应根据电机型号、负载、减速比和机械限位重新设置这些值。
        """
        limits = MITLimits(
            position_max_rad=float(self.get_parameter("mit_position_max_rad").value),
            velocity_max_rad_s=float(self.get_parameter("mit_velocity_max_rad_s").value),
            torque_max_nm=float(self.get_parameter("mit_torque_max_nm").value),
        )
        self.motor.set_mit_limits(limits)

        # 普通控制参数：最大位置速度、速度加速度、位置/速度 PI。
        self.motor.set_control_param(ControlParam.POSITION_MAX_SPEED, self.max_speed_rad_s)
        self.motor.set_control_param(ControlParam.VELOCITY_ACCELERATION, self.max_speed_rad_s)
        self.motor.set_control_param(ControlParam.POSITION_KP, self.kp)
        self.motor.set_control_param(ControlParam.VELOCITY_KP, self.kp)

        # 进阶参数：梯形加减速、位置滤波带宽。
        self.motor.set_advanced_param(AdvancedParam.TRAPEZOID_ACCELERATION, self.max_speed_rad_s)
        self.motor.set_advanced_param(AdvancedParam.TRAPEZOID_DECELERATION, self.max_speed_rad_s)
        self.motor.set_advanced_param(AdvancedParam.POSITION_FILTER_BANDWIDTH, 100.0)

        self.get_logger().info("已写入示例参数。")

    def request_and_log_state(self) -> None:
        """请求一次反馈，并打印当前缓存状态。

        latest_state() 读取的是后台 RX 线程解析后的缓存值。
        如果刚刚 request_feedback()，可能要等下一次定时器才能看到新反馈。
        """
        self.motor.request_feedback(int(Opcode.READ_FAST_STATE))
        state = self.motor.latest_state()
        if state is None:
            self.get_logger().info(f"还没有收到 {self.motor_id} 号电机状态。")
            return
        extra = state.extra if state.extra else {}
        self.get_logger().info(
            f"motor={state.can_id} pos={state.pos} rad vel={state.vel} rad/s "
            f"iq={state.q_current} A temp={state.t_mos} C fault={state.status_code} "
            f"extra={extra}"
        )

    def stop_motor(self) -> None:
        """先发送零速度，再释放电机输出。"""
        for _ in range(3):
            self.motor.send_vel(0.0)
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
        """可选演示：启动时发一次命令，到时间后释放电机。"""
        if not self.demo_enabled and not self.configure_params:
            return

        if self._demo_started_at is None:
            try:
                self.motor.clear_error()
                if self.configure_params:
                    self.configure_example_params()
                if self.demo_enabled:
                    self._send_selected_demo_command()
            except Exception as exc:
                self._demo_stop_sent = True
                self.get_logger().error(f"发送演示命令失败，演示已中止: {exc}")
                return
            self._demo_started_at = time.monotonic()
            self._last_demo_send_at = self._demo_started_at
            self.get_logger().info(
                f"已直接向 {self.motor_id} 号电机发送演示命令："
                f"demo_mode={self.demo_mode}，持续 {self.run_seconds} s"
            )
            return

        now = time.monotonic()
        elapsed = now - self._demo_started_at
        if elapsed >= self.run_seconds and not self._demo_stop_sent:
            self.safe_stop_motor()
            self.request_and_log_state()
            self._demo_stop_sent = True
            self.get_logger().info("直接库调用演示结束，已释放电机。")
            return

        if (
            self.demo_enabled
            and not self._demo_stop_sent
            and (now - self._last_demo_send_at) >= self.demo_repeat_period_s
        ):
            try:
                self._send_selected_demo_command()
                self._last_demo_send_at = now
            except Exception as exc:
                self._demo_stop_sent = True
                self.get_logger().error(f"重发演示命令失败，准备停止: {exc}")
                self.safe_stop_motor()

    def _send_selected_demo_command(self) -> None:
        """根据 demo_mode 选择一种协议调用方式。"""
        if self.demo_mode == "velocity":
            self.send_velocity(self.velocity_rad_s)
        elif self.demo_mode == "current":
            self.send_current(self.current_a)
        elif self.demo_mode in ("position", "absolute_position"):
            self.send_absolute_position_with_speed(self.position_rad, self.max_speed_rad_s)
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

    def destroy_node(self) -> bool:
        # 节点退出时关闭后台 RX 线程，并关闭 SocketCAN。
        try:
            self.controller.shutdown()
        except Exception as exc:
            self.get_logger().error(f"关闭 CAN 控制器失败: {exc}")
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MotorDirectLibraryNode()
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
