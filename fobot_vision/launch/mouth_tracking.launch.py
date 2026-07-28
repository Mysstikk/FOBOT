from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
    
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{'pixel_format': 'mjpeg2rgb'}],
        ),
        Node(
            package='fobot_vision',
            executable='mouth_detector',
            name='mouth_detector',
            output='screen',
            parameters=[{'show_debug_window': False}],
        ),
        Node(
            package='fobot_vision',
            executable='mouth_servo',
            name='mouth_servo',
            output='screen',
        ),
    ])
