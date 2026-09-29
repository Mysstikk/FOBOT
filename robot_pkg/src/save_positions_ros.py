#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray
from time import sleep

class EnsenanzaCinestesicaNode(Node):
    def __init__(self):
        super().__init__('ensenanza_cinestesica_node')
        
        # --- CONFIGURACIÓN ---
        self.PIN_PULSADOR = 17
        self.ARCHIVO_CSV = "/home/gromep/robot_ws/src/robot_pkg/src/trayectoria_dynamixel_rad.csv"
        
        self.joint_names = [
            'joint_hombro', 'joint_hombro_codo', 'joint_codo',
            'joint_codo_muneca', 'joint_muneca', 'joint_herramienta'
        ]
        self.dxl_ids = [1, 2, 3, 4, 5, 6]
        
        self.latest_positions = {}
        self.captura_num = 1
        self.current_state = ""
        
        # --- SUSCRIPCIONES Y SERVICIOS ---
        self.create_subscription(
            JointState, '/joint_states', self.joint_states_cb, 10)
            
        self.create_subscription(
            String, '/FOBOT/state', self.state_cb, 10)
            
        self.create_subscription(
            String, '/FOBOT/interface', self.interface_cb, 10)
            
        self.torque_client = self.create_client(
            SetBool, '/dynamixel_hardware_interface/set_dxl_torque')
            
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
            
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/fobot_joint_controller/commands', 10)
            
    def sincronizar_controladores(self):
        if len(self.latest_positions) < 6:
            return
            
        posiciones_actuales = [self.latest_positions[j] for j in self.joint_names]
            
        # 1. Sincronizamos el controlador de trayectorias (El principal)
        msg_traj = JointTrajectory()
        msg_traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = posiciones_actuales
        # Le damos un tiempo muy pequeño (0.1s) para que asimile la posición al instante
        point.time_from_start = Duration(sec=0, nanosec=100000000) 
        
        msg_traj.points.append(point)
        self.traj_pub.publish(msg_traj)
        
        # 2. Sincronizamos también el controlador de posición directa por seguridad
        msg_cmd = Float64MultiArray()
        msg_cmd.data = posiciones_actuales
        self.cmd_pub.publish(msg_cmd)
            
    def state_cb(self, msg):
        new_state = msg.data
        
        if new_state == 'MODO_MANUAL' and self.current_state != 'MODO_MANUAL':
            self.get_logger().info("APRENDIZAJE EMPEZADO")
            self.captura_num = 1
            if os.path.exists(self.ARCHIVO_CSV):
                os.remove(self.ARCHIVO_CSV)
            self.set_torque_all(False)
            
        elif new_state != 'MODO_MANUAL' and self.current_state == 'MODO_MANUAL':
            self.get_logger().info("APRENDIZAJE FINALIZADO")
            
            self.sincronizar_controladores()
            
            sleep(0.3)
            
            self.set_torque_all(True)
            
        self.current_state = new_state
        
    def interface_cb(self, msg):
        if self.current_state != 'MODO_MANUAL':
            return
            
        if msg.data == 'pulsacion_corta':
            self.guardar_punto()
            
    def joint_states_cb(self, msg):
        for i, name in enumerate(msg.name):
            self.latest_positions[name] = msg.position[i]

    def set_torque_all(self, enable):
        if not self.torque_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Servicio de torque no disponible.")
            return

        req = SetBool.Request()
        req.data = enable
        self.torque_client.call_async(req)
            
    def guardar_punto(self):
        self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        if len(self.latest_positions) < 6:
            self.get_logger().warning("Faltan datos de articulaciones. Ignorando pulsación.")
            return
            
        try:
            # Extraemos y redondeamos las posiciones
            pos_rad = [
                round(self.latest_positions[j], 4) 
                for j in self.joint_names
            ]
            
            linea_csv = ",".join(map(str, pos_rad))
            
            # Abrimos en modo 'append' (añadir) para mayor seguridad ante cortes de luz
            with open(self.ARCHIVO_CSV, "a") as f:
                f.write(linea_csv + "\n")
                
            self.get_logger().info(f"Captura #{self.captura_num} guardada: [{linea_csv}]")
            self.captura_num += 1
            
        except KeyError as e:
            self.get_logger().error(f"Articulación no encontrada: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = EnsenanzaCinestesicaNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
