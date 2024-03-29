import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    operations_handler_node = Node(
        package='lx_operation',
        executable='operations_handler_node',
        name='operations_handler_node',
        emulate_tty=True
    )

    auto_dig_handler_node = Node(
        package='lx_operation',
        executable='auto_dig_handler_node',
        name='auto_dig_handler_node',
        emulate_tty=True
    )

    auto_dump_handler_node = Node(
        package='lx_operation',
        executable='auto_dump_handler_node',
        name='auto_dump_handler_node',
        emulate_tty=True
    )

    auto_nav_handler_node = Node(
        package='lx_operation',
        executable='auto_nav_handler_node',
        name='auto_nav_handler_node',
        emulate_tty=True
    )

    ld.add_action(operations_handler_node)
    ld.add_action(auto_dig_handler_node)
    ld.add_action(auto_dump_handler_node)
    ld.add_action(auto_nav_handler_node)

    return ld