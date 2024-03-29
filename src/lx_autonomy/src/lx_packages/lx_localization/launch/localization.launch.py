import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  lx_localization_params = os.path.join(
    get_package_share_directory('lx_localization'),
    'params',
    'lx_localization_params.yaml'
  )
  lx_ts_localization_params = os.path.join(
    get_package_share_directory('lx_localization'),
    'params',
    'lx_ts_localization_params.yaml'
  )
  # set use_sim_time param
  use_sim_time_param = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')  
  launch_rviz = launch.substitutions.LaunchConfiguration('launch_rviz', default='false')

  node_list = [
    Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_odom_node',
      #load yaml and set use_sim_time params
      parameters=[lx_localization_params, {'use_sim_time': use_sim_time_param}],
      remappings=[
        ('odometry/filtered', 'odometry/ekf_odom_node')
      ]
    ),
    
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_imu_link',
      output='screen',
      arguments=['0.165', '0', '0.405', '-1.5708', '0', '0', 'base_link', 'vectornav'],
      parameters=[{'use_sim_time': use_sim_time_param}],
      condition=launch.conditions.IfCondition(launch_rviz)
    ), # (x y z yaw pitch roll frame_id child_frame_id period_in_ms)

    # static tf transform from base_link to total_station_prism
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_total_station',
      output='screen',
      arguments=['0.27', '0.19', '0.8', '0', '0', '0', 'base_link', 'total_station_prism'],
      parameters=[{'use_sim_time': use_sim_time_param}],
      condition=launch.conditions.IfCondition(launch_rviz)
    ), # (x y z yaw pitch roll frame_id child_frame_id period_in_ms)

    Node(    
      package='robot_localization',
      executable='ekf_node',
      name='ekf_global_node',
      #load yaml and set use_sim_time params
      parameters=[lx_ts_localization_params, {'use_sim_time': use_sim_time_param}],
      remappings=[
        ('odometry/filtered', 'odometry/ekf_global_node')
      ]
    ),

    Node(
      package='lx_localization',
      executable='localization_node',
      name='localization_node',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time_param}],
    ),
    
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time_param}],
      arguments=['-d', '/home/lx_autonomy/lx_autonomy_ws/src/lx_packages/lx_localization/launch/rviz_config.rviz'],
      condition=launch.conditions.IfCondition(launch_rviz)
    )
  ]
  
  return LaunchDescription(node_list)