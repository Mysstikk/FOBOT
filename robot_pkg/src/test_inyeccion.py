import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class ServoTestInjector(Node):
    def __init__(self):
        super().__init__('servo_test_injector')
        
        # Publicador directo al tópico de MoveIt Servo
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        # Frecuencia de publicación: 30 Hz (0.033 segundos)
        timer_period = 0.033 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Inyector de prueba iniciado. Enviando 0.01 m/s en el eje Y...")

    def timer_callback(self):
        msg = TwistStamped()
        
        # El timestamp perfecto es CRÍTICO para que Servo no rechace el mensaje
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Referencia espacial
        msg.header.frame_id = 'base_link' 
        
        # Velocidad constante y segura (1 centímetro por segundo lateral)
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.01
        msg.twist.linear.z = 0.0
        
        # Cero rotaciones
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    test_node = ServoTestInjector()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Prueba detenida por el usuario.")
    finally:
        # Parada de emergencia al cerrar
        stop_msg = TwistStamped()
        stop_msg.header.stamp = test_node.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        test_node.publisher_.publish(stop_msg)
        
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
