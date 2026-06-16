from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "channel",
                default_value="can0",
                description="直接库调用模式使用的 SocketCAN 接口。",
            ),
            DeclareLaunchArgument(
                "motor_id",
                default_value="1",
                description="要直接控制的 Hexmovr 电机 ID。",
            ),
            DeclareLaunchArgument(
                "demo_enabled",
                default_value="false",
                description="设置为 true 时，才会直接向电机发送短时间运动命令。",
            ),
            DeclareLaunchArgument(
                "demo_mode",
                default_value="velocity",
                description=(
                    "演示模式：velocity/current/position/relative_position/"
                    "trapezoid/position_filter/mit。"
                ),
            ),
            DeclareLaunchArgument(
                "velocity_rad_s",
                default_value="0.5",
                description="速度或 MIT 演示使用的速度，单位 rad/s。",
            ),
            DeclareLaunchArgument(
                "position_rad",
                default_value="0.5",
                description="位置、相对位置、梯形、滤波或 MIT 演示使用的位置，单位 rad。",
            ),
            DeclareLaunchArgument(
                "max_speed_rad_s",
                default_value="0.5",
                description="位置控制最大速度，也用于参数写入示例，单位 rad/s。",
            ),
            DeclareLaunchArgument(
                "current_a",
                default_value="0.2",
                description="current 演示使用的 q 轴电流，单位 A。",
            ),
            DeclareLaunchArgument(
                "kp",
                default_value="20.0",
                description="MIT 刚度参数，也用于参数写入示例。",
            ),
            DeclareLaunchArgument(
                "kd",
                default_value="0.5",
                description="MIT 阻尼参数。",
            ),
            DeclareLaunchArgument(
                "torque_nm",
                default_value="0.0",
                description="MIT 前馈力矩，单位 Nm。",
            ),
            DeclareLaunchArgument(
                "run_seconds",
                default_value="2.0",
                description="演示运动持续时间，单位秒。",
            ),
            DeclareLaunchArgument(
                "configure_params",
                default_value="false",
                description="设置为 true 时写入一组示例控制参数；默认关闭。",
            ),
            Node(
                package="hexmovr_motor_example",
                executable="motor_direct_library",
                name="hexmovr_motor_direct_library",
                output="screen",
                parameters=[
                    {
                        "channel": LaunchConfiguration("channel"),
                        "motor_id": LaunchConfiguration("motor_id"),
                        "demo_enabled": LaunchConfiguration("demo_enabled"),
                        "demo_mode": LaunchConfiguration("demo_mode"),
                        "velocity_rad_s": LaunchConfiguration("velocity_rad_s"),
                        "position_rad": LaunchConfiguration("position_rad"),
                        "max_speed_rad_s": LaunchConfiguration("max_speed_rad_s"),
                        "current_a": LaunchConfiguration("current_a"),
                        "kp": LaunchConfiguration("kp"),
                        "kd": LaunchConfiguration("kd"),
                        "torque_nm": LaunchConfiguration("torque_nm"),
                        "run_seconds": LaunchConfiguration("run_seconds"),
                        "configure_params": LaunchConfiguration("configure_params"),
                    }
                ],
            ),
        ]
    )
