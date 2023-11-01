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

    return ld