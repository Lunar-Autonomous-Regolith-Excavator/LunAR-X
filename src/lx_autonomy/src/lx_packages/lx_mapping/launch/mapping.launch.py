from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    use_sim_time_param = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')  
    launch_rviz = launch.substitutions.LaunchConfiguration('launch_rviz', default='false')
    
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
    
    tf_moonyard_link = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='map_to_moonyard',
      output='screen',
      arguments=['-9', '-5' ,'0' ,'0' ,'0' ,'0', 'map', 'moonyard'],
      emulate_tty=True
    )
    
    rviz = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time_param}],
      arguments=['-d', '/home/lx_autonomy/lx_autonomy_ws/src/lx_packages/lx_mapping/launch/rviz_config.rviz'],
      condition=launch.conditions.IfCondition(launch_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(pc_handler_node)
    ld.add_action(world_model_node)
    ld.add_action(tf_moonyard_link)
    ld.add_action(auto_dump_visual_servoing_node)
    ld.add_action(rviz)

    return ld