import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bringup_dir = get_package_share_directory('livox_undistortion')

    node_start_cmd = Node(
            package='livox_undistortion',
            executable='livox_dedistortion_node',
        #     prefix=['gnome-terminal -- gdb -ex run --args'],
            output='screen'
            )

    ld = LaunchDescription()

    ld.add_action(node_start_cmd)

    return ld