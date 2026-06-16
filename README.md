# hexmovr_manager

`hexmovr_manager` 是一个面向 Hexmovr 协议电机的 ROS 2 工作空间，用于通过 SocketCAN 控制电机，并提供 RViz 上位机调试、manager 共享控制和直接库调用示例。

当前工作空间包含：

- `hexmovr_bridge`：底层 Hexmovr 协议库和轻量 ROS2 bridge 节点。
- `hexmovr_moto_manager`：上层 ROS2 manager，负责扫描、刷新、聚合状态、诊断、RViz marker 和 panel 接口。
- `hexmovr_moto_panel`：RViz Panel 插件。
- `hexmovr_motor_example`：示例功能包，演示两种使用方式。

## 架构关系

当前推荐架构是：

```text
hexmovr_bridge
  ├── protocol.py    # 唯一底层协议源，负责 CAN 帧编解码
  ├── bus.py         # python-can / SocketCAN 封装
  ├── motor.py       # HexmovrMotor 控制 API 和状态缓存
  ├── controller.py  # 多电机注册、总线管理、后台 RX 线程
  └── node.py        # 轻量 ROS2 bridge 节点

hexmovr_moto_manager
  ├── 复用 hexmovr_bridge.protocol 和 hexmovr_bridge.bus
  ├── 提供 /hexmovr_moto_manager/* ROS 接口
  ├── 负责多节点共享 CAN、扫描、刷新、状态聚合
  └── 负责 RViz marker / panel / diagnostics

hexmovr_moto_panel
  └── 只连接 manager，不直接访问 CAN
```

也就是说，`hexmovr_bridge` 是底层库；`hexmovr_moto_manager` 是上位机和共享服务层。

## 重要原则

同一条 CAN 总线只建议由一个进程直接占用。

不要同时让这些程序控制同一个 `can0`：

- `hexmovr_moto_manager`
- `hexmovr_bridge` 独立节点
- 直接实例化 `hexmovr_bridge.controller.Controller` 的业务节点

推荐选择：

- 需要 RViz / panel / 多节点共享：运行 `hexmovr_moto_manager`。
- 不需要 RViz，但需要共享 CAN：运行 `hexmovr_manager_headless.launch.py`。
- 单个业务节点独占 CAN：直接 import `hexmovr_bridge`。
- 简单 JSON bridge：单独运行 `hexmovr_bridge` 节点。

抱闸功能当前禁用，因为硬件暂不可用。

## 依赖

- Ubuntu Linux
- ROS 2 Humble
- SocketCAN
- Python 3
- `python-can`
- RViz 2

CAN 接口示例：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 构建

```bash
cd /home/hexmovr02/hexmovr_manager
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只构建核心包：

```bash
colcon build --packages-select hexmovr_bridge hexmovr_moto_manager hexmovr_moto_panel --symlink-install
source install/setup.bash
```

只构建示例包：

```bash
colcon build --packages-select hexmovr_motor_example --symlink-install
source install/setup.bash
```

## 启动 Manager

启动完整 RViz 上位机：

```bash
ros2 launch hexmovr_moto_manager hexmovr_manager.launch.py \
  can_interface:=can0 scan_end_id:=32
```

只启动 manager 后端，不启动 RViz：

```bash
ros2 launch hexmovr_moto_manager hexmovr_manager_headless.launch.py \
  can_interface:=can0 scan_end_id:=32
```

也可以使用完整 launch 但关闭 RViz：

```bash
ros2 launch hexmovr_moto_manager hexmovr_manager.launch.py \
  can_interface:=can0 scan_end_id:=32 open_rviz:=false
```

RViz 在 Ctrl+C 退出时偶尔会打印 rclcpp context 相关错误；只要 `moto_scaner` 显示 cleanly finished，manager 本身就是正常退出。

## Manager ROS 接口

Topics:

- `/hexmovr_moto_manager/command`：输入 JSON 命令，类型 `std_msgs/msg/String`
- `/hexmovr_moto_manager/state`：聚合状态 JSON
- `/hexmovr_moto_manager/event`：事件 JSON
- `/hexmovr_moto_manager/history`：故障历史 JSON
- `/hexmovr_moto_manager/markers`：RViz MarkerArray
- `/diagnostics`：ROS diagnostics

Services:

- `/hexmovr_moto_manager/scan`
- `/hexmovr_moto_manager/refresh`

扫描和刷新：

```bash
ros2 service call /hexmovr_moto_manager/scan std_srvs/srv/Trigger "{}"
ros2 service call /hexmovr_moto_manager/refresh std_srvs/srv/Trigger "{}"
```

查看状态：

```bash
ros2 topic echo /hexmovr_moto_manager/state
ros2 topic echo /hexmovr_moto_manager/event
ros2 topic echo /diagnostics
```

## Manager 命令示例

注意 `ros2 topic pub` 发送 JSON 时，需要把 JSON 放进 `std_msgs/msg/String.data` 字段。

速度控制：

```bash
ros2 topic pub --once /hexmovr_moto_manager/command std_msgs/msg/String \
'{data: "{\"op\":\"control\",\"motor_id\":1,\"mode\":\"velocity\",\"velocity_rad_s\":0.5}"}'
```

绝对位置控制：

```bash
ros2 topic pub --once /hexmovr_moto_manager/command std_msgs/msg/String \
'{data: "{\"op\":\"control\",\"motor_id\":1,\"mode\":\"absolute_position\",\"position_rad\":1.0}"}'
```

相对位置控制：

```bash
ros2 topic pub --once /hexmovr_moto_manager/command std_msgs/msg/String \
'{data: "{\"op\":\"control\",\"motor_id\":1,\"mode\":\"relative_position\",\"position_rad\":0.1}"}'
```

MIT 控制：

```bash
ros2 topic pub --once /hexmovr_moto_manager/command std_msgs/msg/String \
'{data: "{\"op\":\"control\",\"motor_id\":1,\"mode\":\"mit\",\"position_rad\":0.0,\"velocity_rad_s\":0.0,\"stiffness\":30.0,\"damping\":1.0,\"torque_nm\":0.0}"}'
```

清错、设零、释放：

```bash
ros2 topic pub --once /hexmovr_moto_manager/command std_msgs/msg/String \
'{data: "{\"op\":\"clear_error\",\"motor_id\":1}"}'

ros2 topic pub --once /hexmovr_moto_manager/command std_msgs/msg/String \
'{data: "{\"op\":\"set_zero\",\"motor_id\":1}"}'

ros2 topic pub --once /hexmovr_moto_manager/command std_msgs/msg/String \
'{data: "{\"op\":\"free_motor\",\"motor_id\":1}"}'
```

常用 JSON：

```json
{"op":"scan"}
{"op":"refresh_all","deep":true}
{"op":"refresh","motor_id":1,"deep":true}
{"op":"clear_error","motor_id":1}
{"op":"set_zero","motor_id":1}
{"op":"free_motor","motor_id":1}
{"op":"return_to_zero","motor_id":1}
{"op":"control","motor_id":1,"mode":"current","current_a":0.2}
{"op":"control","motor_id":1,"mode":"velocity","velocity_rad_s":0.5}
{"op":"control","motor_id":1,"mode":"absolute_position","position_rad":1.0}
{"op":"control","motor_id":1,"mode":"relative_position","position_rad":0.1}
{"op":"control","motor_id":1,"mode":"trapezoid_position","position_rad":1.0,"relative":false}
{"op":"control","motor_id":1,"mode":"position_filter","position_rad":1.0,"relative":false}
{"op":"control","motor_id":1,"mode":"mit","position_rad":0.0,"velocity_rad_s":0.0,"stiffness":30.0,"damping":1.0,"torque_nm":0.0}
{"op":"set_param","motor_id":1,"group":"control","name":"position_kp","value":1.0}
{"op":"batch","all":true,"command":{"op":"clear_error"}}
```

## Manager JSON Command 表

所有命令都发布到 `/hexmovr_moto_manager/command`，消息类型是 `std_msgs/msg/String`，JSON 字符串放在 `data` 字段内。

通用字段：

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `op` | string | 是 | 命令类型。 |
| `motor_id` | int | 部分命令需要 | 目标电机 ID，范围通常为 `1..254`。 |

顶层 `op`：

| `op` | 必填字段 | 可选字段 | 作用 |
| --- | --- | --- | --- |
| `scan` | 无 | `start_id`, `end_id` | 扫描指定 ID 范围内的电机，并更新 manager 电机列表。 |
| `refresh_all` | 无 | `deep` | 刷新已知电机状态；`deep=true` 时读取更多参数。 |
| `refresh` | `motor_id` | `deep` | 刷新单个电机状态。 |
| `clear_history` | 无 | 无 | 清空 manager 内部故障历史。 |
| `clear_error` | `motor_id` | 无 | 发送清错命令。 |
| `set_zero` | `motor_id` | 无 | 将当前电机位置设置为零点。 |
| `free_motor` | `motor_id` | 无 | 释放电机输出，常用于停止。 |
| `return_to_zero` | `motor_id` | 无 | 发送回零命令。 |
| `control` | `motor_id`, `mode` | 取决于 `mode` | 发送电机控制命令。 |
| `set_param` | `motor_id`, `group`, `name`, `value` | 无 | 写入控制参数或高级参数。 |
| `set_device_address` | `motor_id`, `device_address` | 无 | 写入设备地址。 |
| `set_can_timeout` | `motor_id` | `enabled`, `timeout_ms`, `action_flags` | 写入 CAN timeout 配置。 |
| `set_mit_limits` | `motor_id` | `position_max_rad`, `velocity_max_rad_s`, `torque_max_nm` | 写入 MIT 模式限幅。 |
| `batch` | `command` | `all`, `motor_ids` | 对多个电机执行同一条子命令。 |

`control.mode`：

| `mode` | 必填字段 | 可选字段 | 作用 |
| --- | --- | --- | --- |
| `current` | `current_a` | 无 | q 轴电流控制，单位 A。 |
| `velocity` | `velocity_rad_s` | 无 | 速度控制，单位 rad/s。 |
| `absolute_position` | `position_rad` | 无 | 绝对位置控制，单位 rad。 |
| `relative_position` | `position_rad` | 无 | 相对位置控制，单位 rad。 |
| `trapezoid_position` | `position_rad` | `relative` | 梯形位置控制；`relative=true` 时为相对位置。 |
| `position_filter` | `position_rad` | `relative` | 位置滤波控制；`relative=true` 时为相对位置。 |
| `mit` | 无 | `position_rad`, `velocity_rad_s`, `stiffness`, `damping`, `torque_nm` | MIT 控制，未提供字段时使用安全默认值。 |

`set_param.group`：

| `group` | `name` 可选值 | `value` 单位/含义 |
| --- | --- | --- |
| `control` | `position_max_speed` | rad/s |
| `control` | `max_q_current` | A |
| `control` | `current_slope` | A/s |
| `control` | `velocity_acceleration` | rad/s^2 |
| `control` | `position_kp`, `position_ki`, `velocity_kp`, `velocity_ki` | 控制器参数 |
| `advanced` | `trapezoid_acceleration`, `trapezoid_deceleration` | rad/s^2 |
| `advanced` | `position_filter_bandwidth` | Hz |
| `advanced` | `position_filter_inertia` | 滤波惯量参数 |
| `advanced` | `position_filter_feedforward_current` | A |

`batch` 示例：

```json
{"op":"batch","all":true,"command":{"op":"free_motor"}}
{"op":"batch","motor_ids":[1,2,3],"command":{"op":"clear_error"}}
{"op":"batch","motor_ids":[1,2],"command":{"op":"control","mode":"velocity","velocity_rad_s":0.5}}
```

抱闸命令当前禁用。即使发送 `{"op":"brake"}`，manager 也会返回错误事件，不会向 CAN 总线发送抱闸帧。

## Manager JSON State 表

`/hexmovr_moto_manager/state` 是聚合状态 JSON，供业务节点、RViz panel 和调试工具读取。

顶层字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `connected` | bool | manager 是否已连接 CAN 接口。 |
| `can_interface` | string | 当前 CAN 接口，例如 `can0`。 |
| `transport_error` | string | CAN 连接或收发错误；正常时为空字符串。 |
| `motor_count` | int | 当前 manager 记录的电机数量。 |
| `history_size` | int | manager 内部故障历史条数。 |
| `motors` | array | 电机状态列表。 |

`motors[]` 字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `motor_id` | int | 电机 ID。 |
| `last_seen` | float | 最近一次收到有效反馈的时间戳。 |
| `last_error` | string | manager 记录的该电机最近错误。 |
| `snapshot` | object | 电机详细状态。 |

常用 `snapshot` 字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `motor_id` | int | 电机 ID。 |
| `position_rad` | float | 位置，单位 rad。 |
| `velocity_rad_s` | float | 速度，单位 rad/s。 |
| `torque_nm` | float | MIT 状态解析出的力矩，单位 Nm。 |
| `q_current_a` | float | q 轴电流，单位 A。 |
| `bus_voltage_v` | float | 母线电压，单位 V。 |
| `bus_current_a` | float | 母线电流，单位 A。 |
| `temperature_c` | int | 温度，单位 C。 |
| `run_mode` | int | 电机运行模式编码。 |
| `fault_code` | int | 故障码，`0` 通常表示无故障。 |
| `can_timeout_enabled` | bool | CAN timeout 是否启用。 |
| `can_timeout_ms` | int | CAN timeout 时间，单位 ms。 |
| `can_timeout_action_flags` | int | CAN timeout 动作标志。 |
| `boot_version`, `software_version`, `hardware_version`, `can_protocol_version` | int | 版本信息。 |
| `torque_constant_nm_per_a` | float | 力矩常数。 |
| `position_max_speed_rad_s` | float | 位置最大速度。 |
| `max_q_current_a` | float | 最大 q 轴电流。 |
| `current_slope_a_s` | float | 电流斜率。 |
| `velocity_acceleration_rad_s2` | float | 速度加速度。 |
| `position_kp`, `position_ki`, `velocity_kp`, `velocity_ki` | float | 控制器参数。 |
| `trapezoid_acceleration_rad_s2`, `trapezoid_deceleration_rad_s2` | float | 梯形加减速。 |
| `position_filter_bandwidth_hz` | int | 位置滤波带宽。 |
| `position_filter_inertia_nm_per_turn_s2` | float | 位置滤波惯量参数。 |
| `position_filter_feedforward_current_a` | float | 位置滤波前馈电流。 |
| `configured_device_address` | int | 已配置设备地址。 |
| `mit_position_max_rad`, `mit_velocity_max_rad_s`, `mit_torque_max_nm` | float | MIT 限幅。 |
| `in_mit_mode` | bool | MIT 状态位。 |
| `valid` | bool | snapshot 是否已由反馈更新过。 |
| `last_command` | string | 最近一次成功解析的反馈命令名。 |

示例：

```json
{
  "connected": true,
  "can_interface": "can0",
  "transport_error": "",
  "motor_count": 1,
  "history_size": 0,
  "motors": [
    {
      "motor_id": 1,
      "last_seen": 1781600000.0,
      "last_error": "",
      "snapshot": {
        "motor_id": 1,
        "position_rad": 0.5,
        "velocity_rad_s": 0.1,
        "q_current_a": 0.05,
        "temperature_c": 25,
        "fault_code": 0,
        "valid": true
      }
    }
  ]
}
```

## Manager JSON Event / History 表

`/hexmovr_moto_manager/event`：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `event` | string | 事件名，例如 `manager_started`, `can_connected`, `command_result`, `command_error`, `motor_action_error`。 |
| `payload` | object | 事件附带数据。 |

常见事件：

| `event` | 说明 |
| --- | --- |
| `manager_started` | manager 节点启动。 |
| `can_connected` | CAN 接口连接成功。 |
| `command_result` | JSON 命令执行成功。 |
| `command_error` | JSON 命令解析或执行失败。 |
| `scan_complete` | 扫描完成。 |
| `motor_action` | RViz 交互菜单动作成功。 |
| `motor_action_error` | RViz 交互菜单动作失败。 |

`/hexmovr_moto_manager/history`：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `fault_history` | array | 故障/错误历史列表。 |
| `count` | int | 当前历史条数。 |

## 在其他节点中使用

### 方式一：通过 Manager topic 调用

这是正式机器人项目里更推荐的方式。manager 是唯一直接占用 `can0` 的节点，其他业务节点只发布命令、订阅状态。

示例：

```python
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyHexmovrNode(Node):
    def __init__(self):
        super().__init__("my_hexmovr_node")
        self.command_pub = self.create_publisher(
            String,
            "/hexmovr_moto_manager/command",
            10,
        )
        self.state_sub = self.create_subscription(
            String,
            "/hexmovr_moto_manager/state",
            self.on_state,
            10,
        )

    def send_velocity(self, motor_id: int, velocity_rad_s: float) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "op": "control",
                "motor_id": motor_id,
                "mode": "velocity",
                "velocity_rad_s": velocity_rad_s,
            },
            ensure_ascii=True,
        )
        self.command_pub.publish(msg)

    def on_state(self, msg: String) -> None:
        state = json.loads(msg.data)
        for motor in state.get("motors", []):
            snapshot = motor.get("snapshot", {})
            self.get_logger().info(
                f"motor={motor.get('motor_id')} "
                f"pos={snapshot.get('position_rad')} "
                f"vel={snapshot.get('velocity_rad_s')}"
            )
```

对应完整例程见：

- `src/hexmovr_motor_example/hexmovr_motor_example/motor_demo.py`
- `src/hexmovr_motor_example/launch/motor_demo.launch.py`

运行：

```bash
ros2 launch hexmovr_moto_manager hexmovr_manager_headless.launch.py can_interface:=can0

ros2 launch hexmovr_motor_example motor_demo.launch.py \
  motor_id:=1 demo_enabled:=true demo_mode:=velocity velocity_rad_s:=0.5
```

### 方式二：直接调用 bridge 库

这种方式适合单节点独占 CAN 的场景。不要同时启动 manager。

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

对应完整例程见：

- `src/hexmovr_motor_example/hexmovr_motor_example/motor_direct_library.py`
- `src/hexmovr_motor_example/launch/motor_direct_library.launch.py`

运行：

```bash
ros2 launch hexmovr_motor_example motor_direct_library.launch.py \
  channel:=can0 motor_id:=1 demo_enabled:=true demo_mode:=velocity velocity_rad_s:=0.5
```

## 示例功能包

`hexmovr_motor_example` 当前包含两个节点：

- `motor_demo`：通过 `hexmovr_moto_manager` topic 调用，适合共享 CAN 和 RViz 调试。
- `motor_direct_library`：直接 import `hexmovr_bridge`，适合单节点独占 CAN。

两个示例默认都不会让电机运动，需要显式设置 `demo_enabled:=true`。

支持的 `demo_mode`：

- `velocity`
- `current`
- `position`
- `relative_position`
- `trapezoid`
- `position_filter`
- `mit`

## 轻量 Bridge 节点

`hexmovr_bridge` 也可以作为单独 ROS2 节点运行。它不提供 RViz/panel 聚合能力，只提供轻量 JSON topic。

启动：

```bash
ros2 run hexmovr_bridge hexmovr_bridge --ros-args \
  -p channel:=can0 -p motor_ids:="[1]"
```

发送命令：

```bash
ros2 topic pub --once /hexmovr/cmd std_msgs/msg/String \
'{data: "{\"op\":\"vel\",\"id\":1,\"vel\":0.5}"}'

ros2 topic pub --once /hexmovr/cmd std_msgs/msg/String \
'{data: "{\"op\":\"mit\",\"id\":1,\"pos\":0.0,\"vel\":0.0,\"kp\":30.0,\"kd\":1.0,\"tau\":0.0}"}'

ros2 topic echo /hexmovr/state
ros2 topic echo /hexmovr/event
```

注意：`/hexmovr/state` 的 JSON 结构和 `/hexmovr_moto_manager/state` 不同，不能直接替代 panel 所需状态。

## 测试

bridge 协议测试：

```bash
cd /home/hexmovr02/hexmovr_manager
PYTHONPATH=src/hexmovr_bridge PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q -p no:cacheprovider src/hexmovr_bridge/test
```

manager/bridge 兼容测试：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/hexmovr_bridge:src/hexmovr_moto_manager:$PYTHONPATH \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q -p no:cacheprovider \
  src/hexmovr_moto_manager/test/test_client_bridge_protocol.py
```

构建验证：

```bash
colcon build --packages-select \
  hexmovr_bridge hexmovr_moto_manager hexmovr_moto_panel hexmovr_motor_example \
  --symlink-install
```
