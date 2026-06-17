# Hexmovr Python SDK

`hexmovr_bridge` 可以作为普通 Python 库使用。它适合单个节点独占 CAN 总线、脚本调试、测试电机协议，或者作为更上层机器人控制包的底层依赖。

如果多个 ROS 节点需要共享同一条 CAN 总线，优先使用 `hexmovr_moto_manager` 的 topic 接口，而不是让多个节点同时 import `hexmovr_bridge.Controller`。

## 核心对象

| 对象 | 作用 |
| --- | --- |
| `CanBus` | SocketCAN 封装，负责收发 CAN 帧。 |
| `Controller` | 管理一条 CAN 总线和多个 `HexmovrMotor`，启动后台 RX 线程。 |
| `HexmovrMotor` | 单个电机对象，提供控制 API 和状态缓存。 |
| `MotorState` | `HexmovrMotor.latest_state()` 返回的状态快照。 |
| `protocol.py` | Hexmovr CAN 帧纯函数编解码，适合单元测试。 |
| `config.py` | YAML 电机配置加载和校验。 |

## 最小示例

```python
from hexmovr_bridge.controller import Controller


controller = Controller("can0")
try:
    motor = controller.add_motor(1)
    motor.clear_error()
    motor.send_vel(0.5)
finally:
    controller.shutdown()
```

`Controller("can0")` 会直接打开 SocketCAN 接口，并启动后台 RX 线程。确保同一时间没有 `hexmovr_moto_manager`、独立 `hexmovr_bridge` 节点或其他 direct library 节点占用同一个 CAN 接口。

## 使用 YAML 配置

```python
from hexmovr_bridge.config import load_hexmovr_config
from hexmovr_bridge.controller import Controller


config = load_hexmovr_config(
    "package://hexmovr_bridge/config/hexmovr_motors.example.yaml"
)

controller = Controller(config.channel)
try:
    for motor_config in config.motors:
        if not motor_config.enabled:
            continue
        motor = controller.add_motor(
            motor_config.id,
            fb_id=motor_config.fb_id,
            model=motor_config.model,
        )
        motor.configure_mit_limits(motor_config.mit_limits)
finally:
    controller.shutdown()
```

YAML 中的 `mit_limits` 只用于本地 MIT 编解码限幅。调用 `configure_mit_limits()` 不会写电机参数；只有 `set_mit_limits()` 会发送 CAN 写入命令。

## 常用控制 API

| 方法 | 单位 | 说明 |
| --- | --- | --- |
| `motor.clear_error()` | 无 | 清除故障。 |
| `motor.set_zero()` | 无 | 将当前位置设为电机零点。 |
| `motor.disable()` | 无 | 释放电机输出。 |
| `motor.send_vel(vel)` | rad/s | 速度控制。 |
| `motor.send_current(current)` | A | q 轴电流控制。 |
| `motor.send_pos_vel(pos, vel)` | rad, rad/s | 先设置最大位置速度，再发送绝对位置。 |
| `motor.send_relative_pos(delta)` | rad | 相对位置控制。 |
| `motor.send_trapezoid_pos(pos, position_type)` | rad | 梯形位置控制。 |
| `motor.send_position_filter(pos, position_type)` | rad | 位置滤波控制。 |
| `motor.send_mit(pos, vel, kp, kd, tau)` | rad, rad/s, Nm | MIT 控制。 |
| `motor.return_to_zero()` | 无 | 回零命令。 |
| `motor.request_feedback(opcode)` | 无 | 请求反馈，默认 fast state。 |

位置类型：

```python
from hexmovr_bridge.protocol import PositionType

motor.send_trapezoid_pos(1.0, PositionType.ABSOLUTE)
motor.send_trapezoid_pos(0.1, PositionType.RELATIVE)
```

## 参数写入 API

```python
from hexmovr_bridge.protocol import AdvancedParam, ControlParam, MITLimits

motor.set_control_param(ControlParam.POSITION_MAX_SPEED, 1.0)
motor.set_control_param(ControlParam.POSITION_KP, 20.0)

motor.set_advanced_param(AdvancedParam.TRAPEZOID_ACCELERATION, 1.0)
motor.set_advanced_param(AdvancedParam.POSITION_FILTER_BANDWIDTH, 100.0)

motor.set_can_timeout(enabled=True, timeout_ms=100, action_flags=0)
motor.set_device_address(1)
motor.set_mit_limits(MITLimits(position_max_rad=95.5, velocity_max_rad_s=45.0, torque_max_nm=18.0))
```

参数写入会改变电机配置，真机使用前要确认电机型号、负载、限位和安全范围。

抱闸相关接口当前不提供，因为硬件暂不可用。

## 读取状态

后台 RX 线程会把电机反馈解析并缓存到 `MotorState`。

```python
motor.request_feedback()
state = motor.latest_state()
if state is not None:
    print(state.pos, state.vel, state.q_current, state.t_mos, state.status_code)
```

常用字段：

| 字段 | 说明 |
| --- | --- |
| `has_value` | 是否已有有效反馈。 |
| `can_id` | 反馈 CAN ID。 |
| `pos` | 位置，rad。 |
| `vel` | 速度，rad/s。 |
| `torq` | MIT 力矩，Nm。 |
| `q_current` | q 轴电流，A。 |
| `t_mos` | 温度，C。 |
| `bus_voltage` | 母线电压，V。 |
| `bus_current` | 母线电流，A。 |
| `run_mode` | 运行模式编码。 |
| `status_code` | 故障码或状态码。 |
| `last_feedback` | 最近解析到的反馈类型。 |
| `extra` | 参数反馈等扩展字段。 |

## 连续控制建议

速度、电流、MIT 这类控制最好周期发送，而不是只发一次。一个简单写法：

```python
import time

deadline = time.monotonic() + 2.0
try:
    while time.monotonic() < deadline:
        motor.send_vel(0.5)
        time.sleep(0.1)
finally:
    for _ in range(3):
        motor.send_vel(0.0)
    motor.disable()
```

如果是在 ROS 节点里，用 timer 周期发送更合适。

## 错误处理和关闭

始终在退出时关闭 controller：

```python
controller = Controller("can0")
try:
    ...
finally:
    controller.shutdown()
```

如果发送命令偶发超时，优先检查：

```bash
ip -details -statistics link show can0
```

以及确认没有多个进程同时占用同一个 CAN 接口。

## 何时不要直接使用 SDK

这些场景更推荐通过 `hexmovr_moto_manager`：

- 多个 ROS 节点都要控制/读取电机。
- 需要 RViz panel 调试。
- 需要扫描、故障历史、diagnostics。
- 希望所有业务节点不直接碰 CAN。

这些场景适合直接使用 SDK：

- 单节点独占 CAN。
- 写底层测试脚本。
- 做 ros2_control hardware plugin 或其他机器人控制适配层的底层依赖。
