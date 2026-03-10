import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # --- Micro ROS Agents ---
    node_microros_1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0"],
    )

    node_microros_2 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB1"],
    )

    # --- Robot Core Nodes ---
    movement_node = Node(
        package="robot_core",
        executable="movement_node",
        output="screen",  
    )

    planting_node = Node(
        package="robot_core",
        executable="planting_node",
        output="screen",
    )

    entry_controller = Node(
        package="robot_core",
        executable="entry_controller",
        output="screen",
    )

    mission_node = Node(
        package="robot_core",
        executable="mission_node",
        output="screen",
    )

    # --- Register all nodes ---
    ld.add_action(node_microros_1)
    ld.add_action(node_microros_2)
    ld.add_action(movement_node)
    # ld.add_action(planting_node)
    ld.add_action(entry_controller)
    ld.add_action(mission_node)

    return ld