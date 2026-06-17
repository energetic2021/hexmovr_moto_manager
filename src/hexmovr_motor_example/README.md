# hexmovr_motor_example

这个包演示如何在自己的 ROS2 功能包中调用 Hexmovr 电机。默认控制电机 ID `1`，启动后默认不会主动运动。

包含两个示例节点：

- `motor_demo`：通过 `hexmovr_moto_manager` 的 topic 调用，适合多节点共享 CAN 和 RViz 调试。
- `motor_direct_library`：直接实例化 `hexmovr_bridge.controller.Controller`，适合单节点独占 CAN。

## 构建

```bash
cd /home/hexmovr02/hexmovr_manager
source /opt/ros/humble/setup.bash
colcon build --packages-select hexmovr_bridge hexmovr_moto_manager hexmovr_motor_example --symlink-install
source install/setup.bash
```

## 方式一：通过 manager topic 调用

先启动 manager：

```bash
ros2 launch hexmovr_moto_manager hexmovr_manager_headless.launch.py can_interface:=can0
```

启动示例节点。默认不会让电机运动：

```bash
ros2 run hexmovr_motor_example motor_demo
```

启动短速度演示：

```bash
ros2 launch hexmovr_motor_example motor_demo.launch.py \
  motor_id:=1 demo_enabled:=true demo_mode:=velocity velocity_rad_s:=0.5 run_seconds:=2.0
```

切换演示模式：

```bash
ros2 launch hexmovr_motor_example motor_demo.launch.py \
  motor_id:=1 demo_enabled:=true demo_mode:=mit \
  position_rad:=0.0 velocity_rad_s:=0.0 kp:=20.0 kd:=0.5 torque_nm:=0.0
```

## 方式二：直接调用 bridge 库

这种方式不需要启动 manager，当前节点会直接打开 `can0` 并控制电机。

注意：直接库调用模式下，不要同时运行 `hexmovr_moto_manager` 或独立 `hexmovr_bridge` 节点控制同一个 CAN 接口。

默认启动不会让电机运动：

```bash
ros2 run hexmovr_motor_example motor_direct_library --ros-args \
  -p channel:=can0 -p motor_id:=1
```

启动短速度演示：

```bash
ros2 launch hexmovr_motor_example motor_direct_library.launch.py \
  channel:=can0 motor_id:=1 demo_enabled:=true demo_mode:=velocity velocity_rad_s:=0.5
```

也可以使用 Hexmovr 电机 YAML 配置：

```bash
ros2 launch hexmovr_motor_example motor_direct_library.launch.py \
  config_file:=package://hexmovr_bridge/config/hexmovr_motors.example.yaml \
  motor_id:=1 demo_enabled:=true demo_mode:=velocity velocity_rad_s:=0.5
```

直接库调用核心代码：

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

## demo_mode

两个示例都支持这些 `demo_mode`：

- `velocity`
- `current`
- `position`
- `relative_position`
- `trapezoid`
- `position_filter`
- `mit`

## manager topic 接口

命令 topic：

```text
/hexmovr_moto_manager/command
std_msgs/msg/String
```

状态 topic：

```text
/hexmovr_moto_manager/state
std_msgs/msg/String
```

常用 JSON：

```json
{"op":"control","motor_id":1,"mode":"velocity","velocity_rad_s":0.5}
{"op":"control","motor_id":1,"mode":"absolute_position","position_rad":1.0}
{"op":"control","motor_id":1,"mode":"relative_position","position_rad":0.1}
{"op":"control","motor_id":1,"mode":"current","current_a":0.2}
{"op":"control","motor_id":1,"mode":"mit","position_rad":0.0,"velocity_rad_s":0.0,"stiffness":30.0,"damping":1.0,"torque_nm":0.0}
{"op":"clear_error","motor_id":1}
{"op":"set_zero","motor_id":1}
{"op":"free_motor","motor_id":1}
```
