import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    joint_viz_node = Node(
        package='lx_diagnostics',
        executable='joint_viz_node',
        name='joint_viz_node'
    )

    ld.add_action(joint_viz_node)

    return ld