#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

from controller_manager_msgs.srv import SwitchController

class HomeRobot(Node):
    def __init__(self):
        super().__init__('homing_robot_node')
        
        # 1. Crear el Publicador
        # Publicamos en el topico estándar del controlador de trayectorias
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.homing_sub = self.create_subscription(
            Bool,
            '/homing',
            self.homing_cb,
            10)
            
        # self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        
        self.is_moving = False
        # self.switch_timer = None

    def homing_cb(self, msg):
        if msg.data and not self.is_moving:
            self.is_moving = True
            msg = JointTrajectory()
            
            msg.joint_names = ['joint_hombro', 'joint_hombro_codo', 'joint_codo', 'joint_codo_muneca', 'joint_muneca', 'joint_herramienta']
            
            home = JointTrajectoryPoint()
            
            home.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            home.time_from_start = Duration(sec=10)
            
            msg.points = [home]
            
            self.publisher_.publish(msg)
            
            # self.switch_timer = self.create_timer(10.5, self.switch_cb)
            
    '''def switch_cb(self):
        # Primero matamos el temporizador para que suceda solo una vez
        self.switch_timer.cancel()
        
        # Comprobamos si el gestor de controladores de ROS 2 está vivo
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.is_moving = False
            return
            
        # Preparamos la orden: Apagar trayectoria, Encender posición
        req = SwitchController.Request()
        req.activate_controllers = ["fobot_joint_controller"]
        req.deactivate_controllers = ["joint_trajectory_controller"]
        req.strictness = SwitchController.Request.STRICT
        
        # Enviamos la orden de forma asincrona
        future = self.switch_client.call_async(req)
        future.add_done_callback(self.end_cb)
        
    def end_cb(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info('¡Controlador cambiado con éxito! El robot ya es controlado por Visión.')
            else:
                self.get_logger().error('Fallo interno al cambiar de controlador.')
        except Exception as e:
            self.get_logger().error(f'Error al llamar al servicio: {e}')
    
        self.is_moving = False'''
        
        
def main(args=None):
    rclpy.init(args=args)
    node = HomeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
