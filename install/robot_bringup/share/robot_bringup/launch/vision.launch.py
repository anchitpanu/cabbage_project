from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    apriltag_node = Node(
        package="robot_vision",
        executable="apriltag_node",
        output="screen",
    )

    entry_node = Node(
        package="robot_vision",
        executable="entry_node",
        output="screen",
    )

    cabbage_detector_node = Node(
        package="robot_vision",
        executable="cabbage_detector_node",
        output="screen",
    )

    ld.add_action(apriltag_node)
    ld.add_action(entry_node)
    ld.add_action(cabbage_detector_node)

    return ld