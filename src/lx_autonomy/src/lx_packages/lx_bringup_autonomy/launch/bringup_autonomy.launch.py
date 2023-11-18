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

    # operation_handler launch
    operation_dir = get_package_share_directory('lx_operation')
    operation_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    operation_dir + '/launch/operation.launch.py'))
    
    # task planner launch
    planning_dir = get_package_share_directory('lx_planning')
    planning_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    planning_dir + '/launch/planning.launch.py'))

    # mapping launch                                
    mapping_dir = get_package_share_directory('lx_mapping')
    mapping_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    mapping_dir + '/launch/mapping.launch.py'))
    
    # localization launch
    localization_dir = get_package_share_directory('lx_localization')
    localization_launch = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    localization_dir + '/launch/localization.launch.py'))
    
    goal_handler_node = Node(
        package='lx_external_interface',
        executable='goal_handler_node',
        name='goal_handler_node',
        emulate_tty=True
    )
    
    # foxglove_bridge launch
    foxglove_dir = get_package_share_directory('foxglove_bridge')
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([foxglove_dir, '/foxglove_bridge_launch.xml']),
    )

    pcl_relay = Node(
      package='lx_mapping',
      executable='pcl_relay_node',
      name='pcl_relay_node',
      output='screen',
      emulate_tty=True
    )
    
    ld.add_action(operation_launch)
    ld.add_action(planning_launch)
    # ld.add_action(foxglove_bridge_launch)
    ld.add_action(mapping_launch)
    # ld.add_action(localization_launch)
    ld.add_action(goal_handler_node)
    
    return ld
