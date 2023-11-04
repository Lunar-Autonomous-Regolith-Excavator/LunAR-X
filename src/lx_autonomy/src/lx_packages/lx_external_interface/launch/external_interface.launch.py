import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    external_interface_node = Node(
        package='lx_external_interface',
        executable='external_interface_node',
        name='external_interface_node',
        emulate_tty=True
    )

    goal_handler_node = Node(
        package='lx_external_interface',
        executable='goal_handler_node',
        name='goal_handler_node',
        emulate_tty=True
    )

    ld.add_action(external_interface_node)
    ld.add_action(goal_handler_node)

    return ld