from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("scan_start_id", default_value="1"),
            DeclareLaunchArgument("scan_end_id", default_value="12"),
            DeclareLaunchArgument("show_labels", default_value="false"),
            Node(
                package="hexmovr_moto_manager",
                executable="moto_scaner",
                name="hexmovr_moto_manager",
                output="screen",
                parameters=[
                    {
                        "can_interface": LaunchConfiguration("can_interface"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "scan_start_id": LaunchConfiguration("scan_start_id"),
                        "scan_end_id": LaunchConfiguration("scan_end_id"),
                        "show_labels": LaunchConfiguration("show_labels"),
                    }
                ],
            ),
        ]
    )
