import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    rover_parameters = os.path.join(
        get_package_share_directory('lx_bringup_autonomy'),
        'config',
        'params.yaml'
    )

    param_server_node = Node(
        package='lx_bringup_autonomy',
        executable='param_server_node',
        name='param_server_node',
        parameters=[rover_parameters],
        emulate_tty=True
    )

    ld.add_action(param_server_node)

    return ld