from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pc_handler_node = Node(
            package='lx_mapping',
            executable='pc_handler_node',
            name='pc_handler_node',
            emulate_tty=True
        )
    
    auto_dump_visual_servoing_node = Node(
            package='lx_mapping',
            executable='visual_servoing_node',
            name='auto_dump_visual_servoing_node',
            emulate_tty=True
        )
    
    world_model_node = Node(
            package='lx_mapping',
            executable='world_model_node',
            name='world_model_node',
            emulate_tty=True
        )
    
    berm_evaluation_node = Node(
            package='lx_mapping',
            executable='berm_evaluation_node',
            name='berm_evaluation_node',
            emulate_tty=True
        )

    tf_moonyard_link = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='map_to_moonyard',
      output='screen',
      arguments=['4', '4' ,'0' ,'0' ,'0' ,'0', 'map', 'moonyard'],
      emulate_tty=True
    )

    ld = LaunchDescription()
    ld.add_action(pc_handler_node)
    ld.add_action(world_model_node)
    ld.add_action(tf_moonyard_link)
    ld.add_action(auto_dump_visual_servoing_node)
    ld.add_action(berm_evaluation_node)

    return ld