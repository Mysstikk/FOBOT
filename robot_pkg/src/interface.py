#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Bool, String
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

class RobotInterface(Node):
    def __init__(self):
        super().__init__('robot_interface_node')
        
        self.interface_pub = self.create_publisher(
            String, 
            '/FOBOT/interface', 
            10
        )
        
        GPIO.setmode(GPIO.BCM)
        
        self.INPUT_PIN = 17
        GPIO.setup(self.INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        timer_period = 0.2 
        self.timer = self.create_timer(timer_period, self.check_button)
        self.estado = True
        
    def check_button(self):
        try:
            # Lee el nivel de tensión en el pin
            input_state = GPIO.input(self.INPUT_PIN)
            if input_state == GPIO.LOW:
                self.get_logger().info("Nivel de tensión ALTO detectado")
                msg = String()
                msg.data = 'homing'
                self.interface_pub.publish(msg)
        except KeyboardInterrupt:
            # Limpia los pines GPIO al salir
            GPIO.cleanup()
            
def main(args=None):
    rclpy.init(args=args)
    node = RobotInterface()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
