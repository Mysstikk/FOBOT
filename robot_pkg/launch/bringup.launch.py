from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    
    pkg_name = 'robot_pkg'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    
    # 1. Generar la descripción del robot (URDF/XACRO)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"]),
        ]
    )

    # 2. Envolver como ParameterValue (Necesario para evitar errores de YAML en Humble)
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 3. Ruta al archivo de controladores
    robot_controllers = PathJoinSubstitution(
        [pkg_share, "config", "controllers.yaml"]
    )

    # 4. Nodo ROS2 Control (El cerebro)
    # Importante: Le pasamos 'robot_description' explícitamente para que inicie rápido.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers], # Quitamos robot_description de aquí
        remappings=[
            ("~/robot_description", "/robot_description"), # Le decimos que escuche el topic
        ],
        output="both",
    )

    # 5. Robot State Publisher (Publica las TFs)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 6. Spawners (Cargan los controladores)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    
    position_controller_spawner = Node(
    	package="controller_manager",
    	executable="spawner",
    	arguments=["fobot_joint_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    # Retrasar el controlador de movimiento hasta que el broadcaster esté listo
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner, position_controller_spawner],
        )
    )
    
    interface_node = Node(
        package="robot_pkg",
        executable="interface.py",
        output="screen"
    )
    
    movement_node = Node(
        package="robot_pkg",
        executable="movement.py",
        output="screen"
    )
    
    sequencer_node = Node(
        package="robot_pkg",
        executable="sequencer.py",
        output="screen"
    )
    
    collision_node = Node(
        package="robot_pkg",
        executable="collision_detector.py",
        output="screen"
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        interface_node,
        movement_node,
        sequencer_node,
        collision_node,
    ])


