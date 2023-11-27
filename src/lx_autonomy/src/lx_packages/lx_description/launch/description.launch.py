import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    # LX Rover description / transforms / joints
    robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("lx_description"), "urdf", "lunarx.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        emulate_tty=True
    )

    # Joint visualization node
    joint_viz_node = Node(
        package='lx_description',
        executable='joint_viz_node',
        name='joint_viz_node',
        emulate_tty=True
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        emulate_tty=True
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_viz_node)
    ld.add_action(joint_state_publisher_gui_node)

    return ld