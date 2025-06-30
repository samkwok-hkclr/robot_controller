import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = "robot_controller"

    bringup_dir = get_package_share_directory(pkg_name)

    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nachi_mz07L_config.yaml"),
        description="Full path to the ROS2 parameters file",
    )

    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)

    robot_controller_node = Node(
        package=pkg_name,
        executable="robot_controller_node",
        # namespace="nachi_robot_controller", # FIXME
        parameters=[
            params_file,
        ],
        respawn=use_respawn,
        respawn_delay=3.0,
        output="screen",
    )

    ld.add_action(robot_controller_node)

    return ld