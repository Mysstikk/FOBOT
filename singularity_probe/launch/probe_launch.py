from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("FOBOT", package_name="fobot_moveit_config")
        .to_moveit_configs()
    )

    probe_node = Node(
        package="singularity_probe",
        executable="probe",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription([probe_node])
