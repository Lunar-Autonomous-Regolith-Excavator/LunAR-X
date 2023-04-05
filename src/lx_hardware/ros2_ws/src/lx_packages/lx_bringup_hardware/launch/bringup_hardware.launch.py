import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # hardware_mux launch
    hardware_mux_dir = get_package_share_directory('lx_hardware_mux')
    hardware_mux_launch = IncludeLaunchDescription(
                         PythonLaunchDescriptionSource(
                                hardware_mux_dir + '/launch/hardware_mux.launch.py'))
    
    # status_relay launch
    status_relay_dir = get_package_share_directory('lx_status_relay')
    status_relay_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    status_relay_dir + '/launch/status_relay.launch.py'))

    ld.add_action(hardware_mux_launch)
    ld.add_action(status_relay_launch)

    return ld