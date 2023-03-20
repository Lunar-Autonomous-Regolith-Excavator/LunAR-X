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
  return LaunchDescription([
    Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_odom_node',
      #load yaml and set use_sim_time params
      parameters=[lx_localization_params, {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time', default='true')}],
    ),
    Node(
      package='lx_localization',
      executable='remap_husky_odom',
      name='remap_husky_odom',
      output='screen'),
    # static tf transfomom from base_link to imu_link
    Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            output='screen',
            arguments=['0.1', '0', '0.25', '1.57', '0', '3.1415', 'base_link', 'vectornav']), # (x y z yaw pitch roll frame_id child_frame_id period_in_ms)
    # rviz2 node with config file and set use_sim_time params
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', "/home/lx_autonomy/lx_autonomy_ws/bags/localization.rviz"],
      parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time', default='true')}]
    )      

  ])