import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    bringup_autonomy_dir = get_package_share_directory('lx_bringup_autonomy')
    param_server_launch = launch.actions.IncludeLaunchDescription(
                          launch.launch_description_sources.PythonLaunchDescriptionSource(
                                bringup_autonomy_dir + '/launch/param_server.launch.py'))
    
    rover_command_dir = get_package_share_directory('lx_rover_command')
    command_mux_launch = launch.actions.IncludeLaunchDescription(
                         launch.launch_description_sources.PythonLaunchDescriptionSource(
                                rover_command_dir + '/launch/command_mux.launch.py'))
    
    external_interface_dir = get_package_share_directory('lx_external_interface')
    external_interface_launch = launch.actions.IncludeLaunchDescription(
                                launch.launch_description_sources.PythonLaunchDescriptionSource(
                                    external_interface_dir + '/launch/external_interface.launch.py'))

    ld.add_action(param_server_launch)
    ld.add_action(command_mux_launch)
    ld.add_action(external_interface_launch)

    return ld