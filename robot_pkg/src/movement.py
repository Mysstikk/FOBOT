#!/usr/bin/env python3
import os
import csv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from controller_manager_msgs.srv import SwitchController

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.servo_pub = self.create_publisher(
            String, '/FOBOT/servo', 10)
        
        self.movement_state_pub = self.create_publisher(
            String, '/FOBOT/movement_done', 10)
        
        self.create_subscription(
            String, '/FOBOT/state', self.state_cb, 10)
        
        self.create_subscription(
            String, 'FOBOT/movement_done', self.stop_signal_cb, 10)
        
        self.create_subscription(
            Bool, '/FOBOT/repeat_signal', self.repeat_cb, 10)
        
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller")

        self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)
            
        self.fobot_cmd_pub = self.create_publisher(
            Float64MultiArray, '/fobot_joint_controller/commands', 10)
        
        self.joint_names = [
            'joint_hombro', 'joint_hombro_codo', 'joint_codo',
            'joint_codo_muneca', 'joint_muneca', 'joint_herramienta'
        ]
        
        self.is_moving = False
        self.points_list = []
        self.current_mode = ""            
        self.latest_joint_state = None
        
    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def preseed_fobot_controller(self):
        if self.latest_joint_state is None:
            self.get_logger().error('No /joint_states yet — refusing to switch without a safe pre-seed')
            return False
        try:
            positions = [self.latest_joint_state.position[self.latest_joint_state.name.index(j)]
                         for j in self.joint_names]
        except ValueError as e:
            self.get_logger().error(f'Joint missing from /joint_states: {e}')
            return False
        msg = Float64MultiArray(data=positions)
        for _ in range(3):
            self.fobot_cmd_pub.publish(msg)
        return True
        
    def publish_points(self, points_list):
        self.action_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        time_accum = 0.0
        time_interval = 5.0
        
        for row in points_list:
            float_row = [float(x) for x in row]
            time_accum += time_interval
            
            point = JointTrajectoryPoint()
            point.positions = float_row
            
            int_sec = int(time_accum)
            nanoseconds = int((time_accum - int_sec) * 1e9)
            point.time_from_start = Duration(sec=int_sec, nanosec=nanoseconds)
            
            goal_msg.trajectory.points.append(point)
            
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Trayectoria rechazada')
            self.is_moving = False
            return
            
        self.get_logger().warning('Trayectoria aceptada')
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        
        if result.error_code == 0:
            self.get_logger().info(f'Trayectoria {self.current_mode} completada con éxito')
            
            msg = String()
            msg.data = self.current_mode
            
            if self.current_mode == "auto":
                if self.preseed_fobot_controller():
                    self.switch_controllers(
                        activate=["fobot_joint_controller"], 
                        deactivate=["joint_trajectory_controller"], 
                        end_cb=self.end_servo_cb
                    )
                else:
                    self.get_logger().error('Aborting switch to servo — pre-seed failed')
                    self.is_moving = False
            elif self.current_mode == "homing":
                self.movement_state_pub.publish(msg)
                self.is_moving = False
        else:
            self.get_logger().error(f"Error en la trayectoria. {result.error_code}")
            self.is_moving = False
        
    def switch_controllers(self, activate, deactivate, end_cb):
        # Comprobamos si el gestor de controladores de ROS 2 está vivo
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Servicio de cambio de controladores no disponible.')
            self.is_moving = False
            return
            
        # Preparamos la orden: Apagar trayectoria, Encender posición
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        
        # Enviamos la orden de forma asincrona
        future = self.switch_client.call_async(req)
        future.add_done_callback(end_cb)
        
    def end_servo_cb(self, future):
        try:
            response = future.result()
            
            if response.ok:
                self.get_logger().info('El robot ya es controlado por Visión.')
                self.current_mode = "act_servo"
                msg = String()
                msg.data = self.current_mode
                self.movement_state_pub.publish(msg)
            else:
                self.get_logger().error('Error al cambiar al control por Visión.')
        except Exception as e:
            self.get_logger().error(f'Error al llamar al servicio: {e}')
        finally:
            self.is_moving = False
                
    def end_auto_cb(self, future):
        try:
            response = future.result()
            
            if response.ok:
                self.get_logger().info('El robot ya es controlado por Trayectorias.')
                self.current_mode = "deact_servo"
                msg = String()
                msg.data = self.current_mode
                self.movement_state_pub.publish(msg)
                
                if self.points_list:
                    self.is_moving = True
                    self.current_mode = "auto"
                    self.publish_points(self.points_list)
                else:
                    self.get_logger().warning("Señal ignorada")
                    self.is_moving = False
            else:
                self.get_logger().error("Error en el cambio de controlador")
                self.is_moving = False
        except Exception as e:
            self.get_logger().error(f'Error al llamar al servicio: {e}')
            self.is_moving = False
            
    def end_stop_cb(self, future):
        try:
            response = future.result()
            
            if response.ok:
                self.get_logger().info('El robot ya es controlado por Trayectorias.')
                self.current_mode = "stop"
                
                self.points_list = []
            else:
                self.get_logger().error("Error en el cambio de controlador")
        except Exception as e:
            self.get_logger().error(f'Error al llamar al servicio: {e}')
            self.is_moving = False
        
    def state_cb(self, msg):
        if msg.data and not self.is_moving:
        
            if 'HOMING' in msg.data:
                self.is_moving = True
                self.current_mode = "homing"
                self.publish_points([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
                
            elif 'AUTO' in msg.data:
                nombre_archivo = "/home/gromep/robot_ws/src/robot_pkg/src/trayectoria_dynamixel_rad.csv"

                if not os.path.exists(nombre_archivo):
                    self.get_logger().error(f"no se pudo abrir el archivo o la ruta está mal")
                    return
                    
                self.is_moving = True
                self.current_mode = "auto"
                self.points_list = []
                
                with open(nombre_archivo, "r") as f:
                    data = csv.reader(f)
                    for row in data:
                        if not row:
                            continue
                        self.points_list.append(row)
                    
                if self.points_list:
                    self.publish_points(self.points_list)
                else:
                    self.is_moving = False
                    
    def stop_signal_cb(self, msg):
        if 'stop' in msg.data:
            self.get_logger().info("Abortando secuencia y restaurando controladores...")
            self.is_moving = False
            self.switch_controllers(
                activate=["joint_trajectory_controller"], 
                deactivate=["fobot_joint_controller"], 
                end_cb=self.end_stop_cb # O un callback nuevo que simplemente libere el is_moving
            )
				
    def repeat_cb(self, msg):
        if msg.data and not self.is_moving:
            self.is_moving = True
            self.switch_controllers(
                activate=["joint_trajectory_controller"], 
                deactivate=["fobot_joint_controller"], 
                end_cb=self.end_auto_cb
            )
        
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
