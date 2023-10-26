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
    
    # operation_handler launch
    operation_dir = get_package_share_directory('lx_operation')
    operation_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    operation_dir + '/launch/operation.launch.py'))
                                    
    mapping_dir = get_package_share_directory('lx_mapping')
    mapping_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    mapping_dir + '/launch/mapping.launch.py'))
    
    localization_dir = get_package_share_directory('lx_localization')
    localization_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    localization_dir + '/launch/lx_ekf.launch.py'))
    
    foxglove_dir = get_package_share_directory('foxglove_bridge')
    # launch xml launch file foxglove_bridge_launch.xml
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([foxglove_dir, '/foxglove_bridge_launch.xml']),
    )

    tf_node = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='camera_link_to_base_link',
      output='screen',
      arguments=['0.27', '0.21' ,'0.7' ,'0.0' ,'0.65' ,'-0.06', 'base_link', 'camera_link'],
    ) # (x y z yaw pitch roll frame_id child_frame_id period_in_ms)

    tf_camera_link = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='camera_link_to_camera_depth_frame',
      output='screen',
      arguments=['0.0', '0' ,'0' ,'0.0' ,'0' ,'0', '1', 'camera_link', 'camera_depth_frame'],
    ) 

    tf_camera_depth_link = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='camera_link_to_camera_depth_optical_frame',
      output='screen',
      arguments=['0.0', '0' ,'0' ,'-0.5' ,'0.5' ,'-0.5', '0.5', 'camera_depth_frame', 'camera_depth_optical_frame'],
    ) 

    tf_moonyard_link = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='map_to_moonyard',
      output='screen',
      arguments=['9', '5' ,'0' ,'0' ,'0' ,'0', '0', 'map', 'moonyard'],
    )

    pcl_relay = Node(
      package='lx_mapping',
      executable='pcl_relay_node',
      name='pcl_relay_node',
      output='screen',
    )
    
    ld.add_action(param_server_launch)
    ld.add_action(diagnostics_launch)
    ld.add_action(command_mux_launch)
    ld.add_action(external_interface_launch)
    ld.add_action(operation_launch)
    ld.add_action(tf_node)
    ld.add_action(tf_camera_link)
    ld.add_action(tf_camera_depth_link)
    ld.add_ction(tf_moonyard_link)
    ld.add_action(pcl_relay)
    # ld.add_action(foxglove_bridge_launch)
    ld.add_action(mapping_launch)
    ld.add_action(localization_launch)
    
    return ld
