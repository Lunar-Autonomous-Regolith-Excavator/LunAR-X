import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # param_server launch
    bringup_autonomy_dir = get_package_share_directory('lx_bringup_autonomy')
    param_server_launch = IncludeLaunchDescription(
                          PythonLaunchDescriptionSource(
                                bringup_autonomy_dir + '/launch/param_server.launch.py'))
    
    # robot description launch
    description_dir = get_package_share_directory('lx_description')
    description_launch = IncludeLaunchDescription(
                          PythonLaunchDescriptionSource(
                                description_dir + '/launch/description.launch.py'))
    
    # diagnostics launch
    diagnostics_dir = get_package_share_directory('lx_diagnostics')
    diagnostics_launch = IncludeLaunchDescription(
                          PythonLaunchDescriptionSource(
                                diagnostics_dir + '/launch/diagnostics.launch.py'))
    
    # command_mux launch
    rover_command_dir = get_package_share_directory('lx_rover_command')
    command_mux_launch = IncludeLaunchDescription(
                         PythonLaunchDescriptionSource(
                                rover_command_dir + '/launch/command_mux.launch.py'))
    
    # external_interface launch
    external_interface_dir = get_package_share_directory('lx_external_interface')
    external_interface_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    external_interface_dir + '/launch/external_interface.launch.py'))
    
    ld.add_action(param_server_launch)
    ld.add_action(description_launch)
    ld.add_action(command_mux_launch)
    ld.add_action(external_interface_launch)
    
    return ld
