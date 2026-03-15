# server.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = ''

    # T1: rosbridge websocket server (XML launch file)
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    # T2: web_video_server on port 8081
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8081}],
        output='screen',
    )

    # # T3: front camera (/dev/video0 → /camera1 namespace)
    # front_camera = Node(
    #     package='usb_cam',
    #     executable='usb_cam_node_exe',
    #     name='usb_cam_node',
    #     namespace='camera1',
    #     parameters=[{
    #         'video_device': '/dev/video0',
    #         'image_width': 320,
    #         'image_height': 240,
    #     }],
    #     output='screen',
    # )

    # # T4: top camera (/dev/video2 → /camera2 namespace)
    # top_camera = Node(
    #     package='usb_cam',
    #     executable='usb_cam_node_exe',
    #     name='usb_cam_node',
    #     namespace='camera2',
    #     parameters=[{
    #         'video_device': '/dev/video2',
    #         'image_width': 320,
    #         'image_height': 240,
    #     }],
    #     output='screen',
    # )

    return LaunchDescription([
        rosbridge,
        web_video_server,
        # front_camera,
        # top_camera,
    ])