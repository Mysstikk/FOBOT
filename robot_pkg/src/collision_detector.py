#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector_node')
        
        self.create_subscription(
            JointState, '/joint_states', self.joint_states_cb, 10)
            
        self.interface_pub = self.create_publisher(
            String, '/FOBOT/interface', 10)
            
        self.create_subscription(
            String, '/FOBOT/interface', self.reset_emergency, 10)
            
        self.effort_thresh = 500.0
        
        self.emergency_active = False
        
    def joint_states_cb(self, msg):
        if self.emergency_active:
            return
            
        for i, effort_value in enumerate(msg.effort):
            if abs(effort_value) > self.effort_thresh:
                joint_name = msg.name[i]
                self.get_logger().error(f'Colision detectada en {joint_name}')
                
                msg_out = String()
                msg_out.data = 'emergencia'
                self.interface_pub.publish(msg_out)
                
                self.emergency_active = True
                
                # self.create_timer(5.0, self.reset_emergency)
                break
                
    def reset_emergency(self, msg):
        if msg.data == 'rearme':
            self.emergency_active = False
        
def main():
    rclpy.init()
    node = CollisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
