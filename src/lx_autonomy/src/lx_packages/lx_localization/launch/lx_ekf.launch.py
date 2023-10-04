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
      package='robot_localization',
      executable='ekf_node',
      name='ekf_global_node',
      #load yaml and set use_sim_time params
      parameters=[lx_ts_localization_params, {'use_sim_time': use_sim_time_param}],
      remappings=[
        ('odometry/filtered', 'odometry/ekf_global_node')
      ]
    ),

    # static tf transform from base_link to imu_link
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_imu_link',
      output='screen',
      arguments=['0.17', '0', '0.52', '0', '0', '3.1415', 'base_link', 'vectornav'],
      parameters=[{'use_sim_time': use_sim_time_param}]
    ), # (x y z yaw pitch roll frame_id child_frame_id period_in_ms)

    # static tf transform from base_link to total_station_prism
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_total_station',
      output='screen',
      arguments=['0.27', '0.19', '0.8', '0.785398', '0', '0', 'base_link', 'total_station_prism'],
      # arguments=['0.27', '0.19', '0.8', '0.523599', '0', '0', 'base_link', 'total_station_prism'],
      parameters=[{'use_sim_time': use_sim_time_param}]
    ), # (x y z yaw pitch roll frame_id child_frame_id period_in_ms)
  
    Node(
      package='lx_localization',
      executable='remap_msgs_localization',
      name='remap_msgs_localization',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time_param}],
    ),

    # Node(
    #   package='lx_localization',
    #   executable='custom_localization',
    #   name='custom_localization',
    #   output='screen',
    #   parameters=[{'use_sim_time': use_sim_time_param}],
    # ),
    
  ]
  if(launch_rviz):
    node_list.append(
      Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}],
      )
    )
  
  return LaunchDescription(node_list)