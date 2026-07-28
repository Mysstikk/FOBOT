#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_ORDER = [
    "joint_hombro", "joint_hombro_codo", "joint_codo",
    "joint_codo_muneca", "joint_muneca", "joint_herramienta",
]

class PreSeed(Node):
    def __init__(self):
        super().__init__('preseed_fobot_joint_controller')
        self.pub = self.create_publisher(Float64MultiArray, '/fobot_joint_controller/commands', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.done = False

    def cb(self, msg: JointState):
        if self.done:
            return
        positions = [msg.position[msg.name.index(j)] for j in JOINT_ORDER]
        out = Float64MultiArray(data=positions)
        for _ in range(5):          # publish a few times so it's definitely received
            self.pub.publish(out)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f'Pre-seeded with: {positions}')
        self.done = True

def main():
    rclpy.init()
    node = PreSeed()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
