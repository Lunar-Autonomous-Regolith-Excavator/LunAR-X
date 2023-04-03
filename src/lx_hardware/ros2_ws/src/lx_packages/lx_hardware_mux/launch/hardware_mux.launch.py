import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    hardware_mux_node = Node(
        package='lx_hardware_mux',
        executable='hardware_mux_node',
        name='hardware_mux_node'
    )

    ld.add_action(hardware_mux_node)

    return ld