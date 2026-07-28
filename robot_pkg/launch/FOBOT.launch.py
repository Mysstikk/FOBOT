import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_pkg')
    pkg_moveit_config = get_package_share_directory('fobot_moveit_config')
    pkg_fobot_vision = get_package_share_directory('fobot_vision')
    
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, 'launch', 'bringup.launch.py')
        )
    )
    
    '''move_group_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')
                )
            )
        ]
    )'''
    
    servo_launch = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot_bringup, 'launch', 'servo_launch.py')
                )
            )
        ]
    )
    
    mouth_tracking_launch = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_fobot_vision, 'launch', 'mouth_tracking.launch.py')
                )
            )
        ]
    )
    
    return LaunchDescription([
        bringup_launch,
        # move_group_launch,
        servo_launch,
        mouth_tracking_launch
    ])
