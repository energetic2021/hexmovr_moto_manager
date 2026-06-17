# hexmovr_ros2_control

这是 Hexmovr 电机接入 `ros2_control` 的设计骨架包。

当前状态：

- 只提供设计文档和示例配置。
- 不编译、不注册真实 `hardware_interface::SystemInterface` 插件。
- 不会被 `controller_manager` 加载。

这样做的目的，是先把未来接入 `ros2_control` 时需要的配置边界、接口和安全策略整理出来，同时不影响当前 `hexmovr_bridge`、`hexmovr_moto_manager`、`hexmovr_moto_panel` 的稳定使用。

## 依赖关系

未来正式实现时建议关系如下：

```text
MoveIt / joint_trajectory_controller
        ↓
controller_manager
        ↓
hexmovr_ros2_control hardware plugin
        ↓
hexmovr_bridge
        ↓
SocketCAN
        ↓
Hexmovr motors
```

`hexmovr_bridge` 保持电机级通用驱动；本包负责把机器人 joint command/state 转换成 Hexmovr motor command/state。

## 当前文件

- `doc/design.md`：设计说明和实现路线。
- `config/example.ros2_control.yaml`：示例 ros2_control 配置。
- `include/hexmovr_ros2_control/hexmovr_system.hpp.todo`：未来 C++ hardware plugin 头文件草案。
- `src/hexmovr_system.cpp.todo`：未来 C++ hardware plugin 源文件草案。

## 后续实现顺序

1. 先在具体机器人项目中确定 joint 名称、motor ID、方向、减速比、零点偏移和限位。
2. 用 `hexmovr_bridge` 验证每个电机的控制模式和反馈稳定性。
3. 实现 `hardware_interface::SystemInterface`。
4. 对接 `joint_state_broadcaster` 和基础 forward controller。
5. 再对接 `joint_trajectory_controller` / MoveIt。

当前通用电机适配阶段，不建议把 joint mapping 写进 `hexmovr_bridge` 或 `hexmovr_moto_manager`。
