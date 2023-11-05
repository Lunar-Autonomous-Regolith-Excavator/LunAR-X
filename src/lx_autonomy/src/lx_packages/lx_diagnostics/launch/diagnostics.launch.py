import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    diagnostics_node = Node(
        package='lx_diagnostics',
        executable='diagnostics_node',
        name='diagnostics_node',
        emulate_tty=True
    )

    ld.add_action(diagnostics_node)

    return ld