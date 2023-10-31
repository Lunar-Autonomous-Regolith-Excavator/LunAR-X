from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # create a list of nodes
    perception_node = Node(
            package='lx_mapping',
            executable='berm_evaluation_node',
            name='berm_evaluation_node',
        )
    
    pc_handler_node = Node(
            package='lx_mapping',
            executable='pc_handler_node',
            name='pc_handler_node',
        )

    
    mapping_node = Node(
            package='lx_mapping',
            executable='merge_map_node',
            name='merge_map_node',
        )
    
    auto_dump_visual_servoing_node = Node(
            package='lx_mapping',
            executable='visual_servoing_node',
            name='auto_dump_visual_servoing_node',
        )
    
    world_model_node = Node(
            package='lx_mapping',
            executable='world_model_node',
            name='world_model_node',
        )
    
    tf_node = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='camera_link_to_base_link',
      output='screen',
      arguments=['0.27', '0.21' ,'0.7' ,'0.0' ,'0.65' ,'-0.06', 'base_link', 'camera_link'],
    ) # (x y z yaw pitch roll frame_id child_frame_id period_in_ms)

    # tf_camera_link = Node(
    #   package='tf2_ros',
    #   executable='static_transform_publisher',
    #   name='camera_link_to_base_link',
    #   output='screen',
    #   arguments=['0.0', '0' ,'0' ,'0.0' ,'0' ,'0', '1', 'camera_link', 'camera_depth_frame'],
    # ) 

    # tf_camera_depth_link = Node(
    #   package='tf2_ros',
    #   executable='static_transform_publisher',
    #   name='camera_link_to_base_link',
    #   output='screen',
    #   arguments=['0.0', '0' ,'0' ,'-0.5' ,'0.5' ,'-0.5', '0.5', 'camera_depth_frame', 'camera_depth_optical_frame'],
    # ) 
    
    tf_moonyard_link = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='map_to_moonyard',
      output='screen',
      arguments=['-9', '-5' ,'0' ,'0' ,'0' ,'0', 'map', 'moonyard'],
    )

    # create a launch description with the nodes list
    ld = LaunchDescription()
    # ld.add_action(perception_node)
    # ld.add_action(mapping_node)
    ld.add_action(pc_handler_node)
    ld.add_action(world_model_node)
    ld.add_action(tf_moonyard_link)
    ld.add_action(auto_dump_visual_servoing_node)
    # ld.add_action(tf_node)
    # ld.add_action(tf_camera_link)
    # ld.add_action(tf_camera_depth_link)

    return ld