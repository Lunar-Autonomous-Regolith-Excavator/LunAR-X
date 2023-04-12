import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # realsense_params = os.path.join(
    #         get_package_share_directory('lx_bringup_hardware'),
    #         'params',
    #         'realsense.yaml'
    #     )
    
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

    # imu_node = Node(package='imu', executable='imu_node', name='imu_node')

    # realsense_node = Node(    
    #   package='realsense2_camera',
    #   executable='rs_multi_camera_launch.py',
    #   name='rs_multi_camera_node',
    #   #load yaml and set use_sim_time params
    #   parameters=[realsense_params],
    # )

    ld.add_action(hardware_mux_launch)
    ld.add_action(status_relay_launch)
    # ld.add_action(imu_node)
    # ld.add_action(realsense_node)

    return ld