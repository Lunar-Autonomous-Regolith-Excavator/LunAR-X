import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    task_planner_node = Node(
        package='lx_planning',
        executable='task_planner_node',
        name='task_planner_node'
    )

    ld.add_action(task_planner_node)

    return ld