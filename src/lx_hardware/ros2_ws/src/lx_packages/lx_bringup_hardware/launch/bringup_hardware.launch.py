import os
import launch
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
    
    # tool height launch
    tool_height_dir = get_package_share_directory('ros2_aruco')
    tool_height_launch = IncludeLaunchDescription(
                          PythonLaunchDescriptionSource(
                          tool_height_dir + '/launch/aruco_recognition.launch.py'))

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('vectornav') + '/launch/vectornav.launch.py'),
    )

    realsense_params_path = os.path.join(get_package_share_directory('lx_bringup_hardware'), 'config/realsense_config.yaml')
    realsense_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={#'config_file': f"'{realsense_params_path}'"
                            'align_depth.enable': 'true', 
                           'depth_module.depth_format': '16UC1'}.items()
    )

    status_relay_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            status_relay_dir + '/launch/status_relay.launch.py'))

    ld.add_action(hardware_mux_launch)
    ld.add_action(status_relay_launch)
    ld.add_action(imu_launch)
    # ld.add_action(realsense_launch)
    ld.add_action(tool_height_launch)

    return ld