from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    apriltag_node = Node(
        package="robot_vision",
        executable="apriltag_node",
        name="apriltag_detector",
        output="screen",
        respawn=True
    )

    entry_node = Node(
        package="robot_vision",
        executable="entry_node",
        name="entry_detector",
        output="screen",
        respawn=True
    )

    cabbage_detector_node = Node(
        package="robot_vision",
        executable="cabbage_detector_node",
        name="cabbage_detector",
        output="screen",
        respawn=True
    )

    return LaunchDescription([
        apriltag_node,
        entry_node,
        cabbage_detector_node
    ])