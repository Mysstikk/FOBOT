import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("FOBOT", package_name="fobot_moveit_config")
        .to_moveit_configs()
    )

    # Load YOUR servo.yaml (it lives in robot_pkg, not moveit_servo)
    servo_yaml = load_yaml("robot_pkg", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])
