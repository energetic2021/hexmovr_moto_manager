from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motor_id",
                default_value="1",
                description="要通过 manager 控制的 Hexmovr 电机 ID。",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="/hexmovr_moto_manager/command",
                description="manager 命令 topic。",
            ),
            DeclareLaunchArgument(
                "state_topic",
                default_value="/hexmovr_moto_manager/state",
                description="manager 状态 topic。",
            ),
            DeclareLaunchArgument(
                "demo_enabled",
                default_value="false",
                description="设置为 true 时，才会通过 manager 向电机发送短时间运动命令。",
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
                "current_a",
                default_value="0.2",
                description="current 演示使用的 q 轴电流，单位 A。",
            ),
            DeclareLaunchArgument(
                "kp",
                default_value="20.0",
                description="MIT 刚度参数。",
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
            Node(
                package="hexmovr_motor_example",
                executable="motor_demo",
                name="hexmovr_motor_demo",
                output="screen",
                parameters=[
                    {
                        "motor_id": LaunchConfiguration("motor_id"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "state_topic": LaunchConfiguration("state_topic"),
                        "demo_enabled": LaunchConfiguration("demo_enabled"),
                        "demo_mode": LaunchConfiguration("demo_mode"),
                        "velocity_rad_s": LaunchConfiguration("velocity_rad_s"),
                        "position_rad": LaunchConfiguration("position_rad"),
                        "current_a": LaunchConfiguration("current_a"),
                        "kp": LaunchConfiguration("kp"),
                        "kd": LaunchConfiguration("kd"),
                        "torque_nm": LaunchConfiguration("torque_nm"),
                        "run_seconds": LaunchConfiguration("run_seconds"),
                    }
                ],
            ),
        ]
    )
