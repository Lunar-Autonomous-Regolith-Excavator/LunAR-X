from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # create a list of nodes
    nodes = [
        Node(
            package='lx_berm',
            executable='berm_eval',
            name='berm_eval',
        )
    ]

    # create a launch description with the nodes list
    ld = LaunchDescription(nodes)

    return ld
