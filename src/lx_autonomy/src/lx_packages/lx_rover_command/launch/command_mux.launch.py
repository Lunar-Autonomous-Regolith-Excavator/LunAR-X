import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    command_mux_node = Node(
        package='lx_rover_command',
        executable='command_mux_node',
        name='command_mux_node',
        emulate_tty=True
    )

    ld.add_action(command_mux_node)

    return ld