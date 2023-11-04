import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    status_relay_node = Node(
        package='lx_status_relay',
        executable='status_relay_node',
        name='status_relay_node',
        emulate_tty=True
    )

    ld.add_action(status_relay_node)

    return ld