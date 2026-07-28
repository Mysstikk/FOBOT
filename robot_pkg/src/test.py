import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        
        # 1. Crear el Publicador
        # Publicamos en el topico estándar del controlador de trayectorias
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Esperamos un segundo para asegurar que la conexión se establece
        time.sleep(1.0)
        self.send_trajectory()

    def send_trajectory(self):
        msg = JointTrajectory()
        
        # Deben ser los mismos que los del archivo config/controllers.yaml
        # msg.joint_names = ['joint_hombro', 'joint_hombro_codo', 'joint_codo', 'joint_codo_muneca', 'joint_muneca', 'joint_herramienta']
        msg.joint_names = ['joint_hombro']
        
        # --- Crear el Punto de Destino ---
        point1 = JointTrajectoryPoint()
        
        # Posiciones objetivo en Radianes.
        
        # point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        
        point1.positions = [3.14]
        
        # --- Definir el TIEMPO (Duration) ---
        point1.time_from_start = Duration(sec=5)
        
        '''point2 = JointTrajectoryPoint()
        point2.positions = [0.0, -1.57, 0.0, 0.0]
        point2.time_from_start = Duration(sec=4)
        
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, -1.57, 0.0, 1.57]
        point3.time_from_start = Duration(sec=6)
        
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, 0.0, 0.0, 1.57]
        point4.time_from_start = Duration(sec=8)
        
        point5 = JointTrajectoryPoint()
        point5.positions = [0.0, 0.0, 0.8, 1.57]
        point5.time_from_start = Duration(sec=9)
        
        point6 = JointTrajectoryPoint()
        point6.positions = [0.0, 0.0, -0.8, 1.57]
        point6.time_from_start = Duration(sec=10)
        
        point7 = JointTrajectoryPoint()
        point7.positions = [0.0, -1.57, 0.0, 1.57]
        point7.time_from_start = Duration(sec=12)
        
        point8 = JointTrajectoryPoint()
        point8.positions = [0.0, -1.57, 0.0, 0.0]
        point8.time_from_start = Duration(sec=13)
        
        point9 = JointTrajectoryPoint()
        point9.positions = [0.0, 0.0, 0.0, 0.0]
        point9.time_from_start = Duration(sec=14)'''
        
        # msg.points = [point1, point2, point3, point4, point5, point6, point7, point8, point9]
        
        msg.points = [point1]
        
        self.publisher_.publish(msg) # Se publica la lista con todos los movimientos 

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    
    # Procesamos una vez y cerramos (script de disparo único)
    rclpy.spin_once(node, timeout_sec=1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
