#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
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
        
        self.press_duration = 0.0
        self.long_press_triggered = False
        
        self.timer = self.create_timer(0.1, self.check_button)
        self.get_logger().info("Interfaz cargada")
        
    def check_button(self):
        is_pressed = (GPIO.input(self.INPUT_PIN) == GPIO.LOW)
        
        if is_pressed:
            self.press_duration += 0.1
            
            if self.press_duration >= 2.0 and not self.long_press_triggered:
                self.get_logger().info("Pulsacion larga")
                msg = String()
                msg.data = 'pulsacion_larga'
                self.interface_pub.publish(msg)
                self.long_press_triggered = True
                
        else:
            if self.press_duration >= 0.1 and not self.long_press_triggered:
                self.get_logger().info("Pulsacion corta")
                msg = String()
                msg.data = 'pulsacion_corta'
                self.interface_pub.publish(msg)
                
            self.press_duration = 0.0
            self.long_press_triggered = False
            
def main(args=None):
    rclpy.init(args=args)
    node = RobotInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
