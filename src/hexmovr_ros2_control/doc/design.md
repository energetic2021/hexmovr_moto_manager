# Hexmovr ros2_control Design

## Scope

This package is intended to become a `ros2_control` hardware adapter for robots that use Hexmovr motors.

It should not replace:

- `hexmovr_bridge`: motor-level protocol, CAN, and direct API.
- `hexmovr_moto_manager`: motor-level shared service and RViz/debug backend.
- `hexmovr_moto_panel`: RViz UI.

The hardware plugin should depend on `hexmovr_bridge` for motor-level communication.

## Responsibility Boundary

`hexmovr_bridge` knows:

- CAN channel.
- motor ID.
- Hexmovr protocol.
- motor position/velocity/current/torque feedback.

`hexmovr_ros2_control` should know:

- joint name.
- motor ID.
- joint-to-motor direction.
- reduction ratio.
- software offset.
- command interface type.
- joint limits and safety policy.

Concrete robot packages should know:

- URDF/Xacro.
- actual joint list.
- MoveIt groups.
- robot-specific launch files.
- final controller selection.

## Proposed Joint Mapping

Example:

```yaml
hexmovr_ros2_control:
  ros__parameters:
    channel: can0

    joints:
      - name: joint_1
        motor_id: 1
        command_interface: position
        hexmovr_mode: trapezoid_position
        direction: 1.0
        reduction: 1.0
        offset_rad: 0.0
        max_position_rad: 3.14
        min_position_rad: -3.14
        max_velocity_rad_s: 1.0
        max_current_a: 2.0
```

Conversion:

```text
joint_position = motor_position * direction / reduction + offset_rad
motor_position = (joint_position - offset_rad) * reduction * direction
joint_velocity = motor_velocity * direction / reduction
motor_velocity = joint_velocity * reduction * direction
```

This mapping belongs in a robot-specific config file or this future hardware plugin config, not in `hexmovr_bridge`.

## Proposed Interfaces

State interfaces:

- `position`
- `velocity`
- `effort`

Command interfaces:

- `position`
- `velocity`
- `effort`

Possible Hexmovr command mapping:

| ros2_control command interface | Hexmovr command |
| --- | --- |
| `position` | `send_pos_vel`, `send_trapezoid_pos`, or `send_position_filter` |
| `velocity` | `send_vel` |
| `effort` | `send_current` or MIT torque field |

The first implementation should probably support one mode per joint from config. Avoid automatic mode switching until the basic path is stable.

## Lifecycle Behavior

Recommended behavior:

| Lifecycle hook | Behavior |
| --- | --- |
| `on_init` | Read parameters, validate joint mapping, allocate state/command arrays. |
| `on_configure` | Create `hexmovr_bridge.Controller`, add motors, configure local MIT limits. |
| `on_activate` | Clear stale commands, optionally clear motor errors if explicitly configured. |
| `read` | Copy `HexmovrMotor.latest_state()` into joint state arrays. |
| `write` | Clamp commands, convert joint command to motor command, send to motor. |
| `on_deactivate` | Send zero velocity where appropriate, then `disable()` motors if configured. |
| `on_cleanup` | Shutdown controller and CAN bus. |

## Safety Policy

The plugin should reject or clamp:

- NaN / inf command values.
- position outside soft limits.
- velocity outside configured limit.
- current / effort outside configured limit.
- command updates when feedback is stale.

Recommended parameters:

```yaml
command_timeout_s: 0.2
feedback_timeout_s: 0.5
stop_on_deactivate: true
free_on_deactivate: true
clear_error_on_activate: false
```

## Implementation Notes

Standard `ros2_control` hardware plugins are C++ `hardware_interface::SystemInterface` plugins.

Because `hexmovr_bridge` is currently Python, there are two possible future paths:

1. Implement the hardware plugin in C++ and port the necessary CAN/protocol layer to C++.
2. Keep `hexmovr_bridge` as a Python service and write a C++ plugin that talks to it through ROS topics/services.

Path 1 is cleaner for real-time-ish control and controller_manager integration. Path 2 is faster for prototyping but adds ROS IPC latency and another process.

For now, do not implement a half-plugin in this package. Keep this skeleton buildable and explicit.
