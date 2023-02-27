import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    external_interface_node = Node(
        package='lx_external_interface',
        executable='external_interface_node',
        name='external_interface_node'
    )

    ld.add_action(external_interface_node)

    return ld