from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # create a list of nodes
    perception_node = Node(
            package='lx_perception',
            executable='berm_evaluation_node',
            name='berm_evaluation_node',
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
      name='camera_link_to_base_link',
      output='screen',
      arguments=['0.0', '0' ,'0' ,'0.0' ,'0' ,'0', '1', 'camera_link', 'camera_depth_frame'],
    ) 

    tf_camera_depth_link = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='camera_link_to_base_link',
      output='screen',
      arguments=['0.0', '0' ,'0' ,'-0.5' ,'0.5' ,'-0.5', '0.5', 'camera_depth_frame', 'camera_depth_optical_frame'],
    ) 
    

    # create a launch description with the nodes list
    ld = LaunchDescription()
    ld.add_action(perception_node)
    ld.add_action(tf_node)
    ld.add_action(tf_camera_link)
    ld.add_action(tf_camera_depth_link)

    return ld