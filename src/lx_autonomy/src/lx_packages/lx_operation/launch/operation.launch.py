import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    operations_handler_node = Node(
        package='lx_operation',
        executable='operations_handler_node',
        name='operations_handler_node',
        emulate_tty=True
    )

    auto_dig_handler_node = Node(
        package='lx_operation',
        executable='auto_dig_handler_node',
        name='auto_dig_handler_node',
        emulate_tty=True
    )

    auto_dump_handler_node = Node(
        package='lx_operation',
        executable='auto_dump_handler_node',
        name='auto_dump_handler_node',
        emulate_tty=True
    )

    auto_nav_handler_node = Node(
        package='lx_operation',
        executable='auto_nav_handler_node',
        name='auto_nav_handler_node',
        emulate_tty=True
    )

    ld.add_action(operations_handler_node)
    ld.add_action(auto_dig_handler_node)
    ld.add_action(auto_dump_handler_node)
    ld.add_action(auto_nav_handler_node)

    # Nav2 Planner Server and Controller Server
    configured_params = os.path.join(get_package_share_directory('lx_operation'), 'config', 'params.yaml')

    lifecycle_nodes = [
                       'controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                    #    'velocity_smoother'
                       ]

    nav2_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    nav2_smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params]
    )
    
    nav2_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params]
    )

    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params]
    )

    nav2_bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params]
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': False}, {'autostart': True}, {'node_names': lifecycle_nodes}]
    )

    # RViz
    rviz_config_dir = os.path.join(get_package_share_directory('lx_operation'), 'rviz', 'autonav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir]
    )

    ld.add_action(nav2_controller_server)
    ld.add_action(nav2_smoother)
    ld.add_action(nav2_planner_server)
    ld.add_action(nav2_behaviors)
    ld.add_action(nav2_bt_navigator_node)
    ld.add_action(nav2_lifecycle_manager)
    ld.add_action(rviz_node)

    return ld