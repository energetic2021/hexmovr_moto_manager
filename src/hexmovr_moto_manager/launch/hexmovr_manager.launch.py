from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _filtered_rviz_env() -> dict[str, str]:
    env = {}
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if ld_library_path:
        env["LD_LIBRARY_PATH"] = ":".join(
            path for path in ld_library_path.split(":") if path and not path.startswith("/snap/")
        )
    env["GTK_PATH"] = ""
    env["QT_QPA_PLATFORM"] = os.environ.get("QT_QPA_PLATFORM", "xcb")
    for key in (
        "SNAP",
        "SNAP_NAME",
        "SNAP_REVISION",
        "SNAP_ARCH",
        "SNAP_LIBRARY_PATH",
        "SNAP_INSTANCE_NAME",
        "SNAP_USER_COMMON",
        "SNAP_USER_DATA",
        "SNAP_COMMON",
        "SNAP_DATA",
        "SNAP_CONTEXT",
    ):
        env[key] = ""
    return env


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("hexmovr_moto_manager")
    rviz_config = os.path.join(package_share, "rviz", "hexmovr_manager.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("scan_start_id", default_value="1"),
            DeclareLaunchArgument("scan_end_id", default_value="12"),
            DeclareLaunchArgument("show_labels", default_value="false"),
            DeclareLaunchArgument("open_rviz", default_value="true"),
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
            Node(
                condition=IfCondition(LaunchConfiguration("open_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                additional_env=_filtered_rviz_env(),
                output="screen",
            ),
        ]
    )
