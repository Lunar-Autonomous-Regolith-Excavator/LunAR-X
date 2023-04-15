from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # create a list of nodes
    nodes = [
        Node(
            package='lx_perception',
            executable='berm_evaluation_node',
            name='berm_evaluation_node',
        )
    ]

    # create a launch description with the nodes list
    ld = LaunchDescription(nodes)

    return ld