# hexmovr_manager

`hexmovr_manager` 是一个面向 Hexmovr 电机的 ROS 2 调试上位机工作空间，目标是在 ROS/RViz 环境里完成电机扫描、状态检查、控制、参数编辑和后续扩展功能。

当前实现以 `hexmovr_moto_manager` 包为核心，采用 Python + SocketCAN + ROS 2 + RViz 的方式搭建。

## 项目目标

- 扫描识别 Hexmovr 电机
- 检查电机状态、故障和基础参数
- 控制电机位置、速度、电流和 MIT 模式
- 编辑电机控制参数和部分高级参数
- 在 RViz 中可视化查看和执行常用操作

## 当前状态

当前已经完成的第一阶段能力：

- Hexmovr 自定义 CAN 协议的 Python 编解码
- SocketCAN 传输层
- ROS 2 管理节点
- 电机扫描、刷新、基础状态读取
- 位置/速度/电流/MIT 控制接口
- 参数读写入口
- 故障历史记录与历史 topic
- 批量命令入口
- RViz `MarkerArray` 可视化
- RViz `InteractiveMarker` 菜单交互
- 专门的 RViz Panel
- RViz Panel 内的参数表单、批量电机页、故障历史页和基础曲线图
- RViz Panel 中英文显示切换
- CAN 接口缺失时的友好报错与自动重连
- RViz 启动环境净化，规避部分 Snap 环境库冲突

当前仍在持续完善：

- 真机联调覆盖更多协议命令
- 更完整的参数编辑体验
- 更适合上位机操作的 RViz Panel / GUI
- 批量电机管理和更强的诊断展示

## 工作空间结构

- `src/hexmovr_moto_manager`
  ROS 2 Python 包，当前主要开发内容
- `src/hexmovr_moto_panel`
  RViz Panel 插件包
- `build/`, `install/`, `log/`
  colcon 构建产物

## 依赖环境

- Ubuntu + ROS 2 Humble
- SocketCAN
- RViz 2
- 已正确配置的 CAN 接口，例如 `can0`

## 构建

```bash
cd /home/hexmovr02/hexmovr_manager
source /opt/ros/humble/setup.bash
colcon build --packages-select hexmovr_moto_manager hexmovr_moto_panel
source install/setup.bash
```

## 启动

建议先设置 ROS 日志目录，避免默认写入路径带来权限问题：

```bash
export ROS_LOG_DIR=/tmp/ros_logs_hexmovr
ros2 launch hexmovr_moto_manager hexmovr_manager.launch.py can_interface:=can0 scan_end_id:=32
```

如果 `can0` 不存在，节点现在不会直接崩溃，而是进入等待重连状态，并在诊断里报告总线不可用。

## 常用调试命令

扫描总线：

```bash
ros2 service call /hexmovr_moto_manager/scan std_srvs/srv/Trigger "{}"
```

刷新全部电机：

```bash
ros2 service call /hexmovr_moto_manager/refresh std_srvs/srv/Trigger "{}"
```

查看状态：

```bash
ros2 topic echo /hexmovr_moto_manager/state
ros2 topic echo /diagnostics
```

发送一个位置控制命令：

```bash
ros2 topic pub --once /hexmovr_moto_manager/command std_msgs/msg/String \
'{data: "{\"op\":\"control\",\"motor_id\":1,\"mode\":\"absolute_position\",\"position_rad\":1.0}"}'
```

## 当前 ROS 接口

- Topic
  - `/hexmovr_moto_manager/command`
  - `/hexmovr_moto_manager/state`
  - `/hexmovr_moto_manager/event`
  - `/hexmovr_moto_manager/history`
  - `/hexmovr_moto_manager/markers`
  - `/diagnostics`
- Service
  - `/hexmovr_moto_manager/scan`
  - `/hexmovr_moto_manager/refresh`

## RViz 交互

启动 launch 后会同时启动 RViz。

当前支持：

- 查看电机状态颜色
- 查看位置、速度、电流、温度等文本信息
- 在交互菜单中执行扫描、刷新、清错、回零、抱闸、点动、速度控制等常用操作
- 自动加载 `Hexmovr Panel`
- 在 Panel 中查看电机列表和结构化状态
- 在 Panel 中执行单电机控制和参数写入
- 在 Panel 中执行批量动作、批量控制和批量参数下发
- 在 Panel 中查看故障历史
- 在 Panel 中查看基础时间序列曲线
- 在 Panel 右上角切换中文 / English 界面显示
- 电机朝向指针已调整为更贴近本体比例的顶部短箭头

说明：

- 中英文切换目前覆盖 RViz Panel 的主要界面元素，包括页签、按钮、表单标签、表头、批量控制选项、参数分组、曲线指标和状态提示
- 电机实时数据字段名、后端事件名和故障消息内容目前仍以协议/后端原始字段为主，后续可以继续补成完整双语

## 参考实现来源

本工作空间实现过程中参考了以下本地项目和资料：

- `/home/hexmovr02/openarm_can`
- `/home/hexmovr02/Library_design/motorbridge`
- `/home/hexmovr02/openarm_ros2`
- `/mnt/share/电机文档/自定义CAN通信协议_3.09b0.pdf`

## 开发记录

### 2026-06-10

已完成：

- 初版 Hexmovr ROS 2 管理节点
- 协议编解码与 SocketCAN 接入
- RViz 可视化与交互菜单
- CAN 接口不存在时的容错与重连
- RViz 启动环境兼容处理
- RViz marker topic 和静态 TF 兼容性修正
- 新增专门的 RViz Panel、批量命令、故障历史和基础曲线图
- 新增 RViz Panel 中英文显示切换
- 优化 RViz 电机朝向指针比例，减少悬空和过大视觉效果

## README 维护约定

后续每次功能成功实现后，继续同步更新以下内容：

- `当前状态`
- `常用调试命令`
- `当前 ROS 接口`
- `开发记录`

这样 README 会始终反映当前真实可用能力，而不是停留在规划阶段。
