import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)
import numpy as np
from scipy.spatial.transform import Rotation as R

class MouthServo(Node):
    def __init__(self):
        super().__init__('mouth_servo')

        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('max_linear_speed', 0.02)   # m/s — keep SLOW near a face
        self.declare_parameter('p_gain', 1.0)
        self.declare_parameter('deadband', 0.8)           # m — stop jittering once this close

        self.planning_frame = self.get_parameter('planning_frame').value
        self.max_speed = self.get_parameter('max_linear_speed').value
        self.p_gain = self.get_parameter('p_gain').value
        self.deadband = self.get_parameter('deadband').value

        # Rotation parameters
        self.declare_parameter('target_quat_xyzw', [0.001, -0.002, -0.707, 0.707])
        self.declare_parameter('p_gain_angular', 1.5)
        self.declare_parameter('max_angular_speed', 0.3)
        self.declare_parameter('min_approach_distance', 0.05)

        self.target_rotation = R.from_quat(self.get_parameter('target_quat_xyzw').value)
        self.p_gain_angular = self.get_parameter('p_gain_angular').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_approach_distance = self.get_parameter('min_approach_distance').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_sub = self.create_subscription(
            PointStamped, '/mouth_position', self.target_cb, 10)

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
            
        self.repeat_pub = self.create_publisher(
            Bool, '/FOBOT/repeat_signal', 10)
            
        self.start_servo_client = self.create_client(
            Trigger, '/servo_node/start_servo')
            
        self.signal_sent = False
        
        self.last_target_time = None
        self.target_timeout = 0.5

        self.last_target = None
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)  # match servo.yaml publish_period
        self.servo_timer = self.create_timer(5.0, self.activate_servo)
        
    def activate_servo(self):
        self.servo_timer.cancel()
        
        self.get_logger().info("Esperando a activar MoveIt Servo")
        if self.start_servo_client.wait_for_service(timeout_sec=5.0):
            req = Trigger.Request()
            self.start_servo_client.call_async(req)
            self.get_logger().info("Servicio activado")
        else:
            self.get_logger().error("El servicio no se ha podido activar")

    def target_cb(self, msg: PointStamped):
        try:
            transformed = self.tf_buffer.transform(
                msg, self.planning_frame, timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return
            
        transformed.point.z += 0.1
            
        alpha = 0.3  # lower = smoother but laggier; tune by feel
        if self.last_target is None:
            self.last_target = transformed
        else:
            self.last_target.point.x = alpha * transformed.point.x + (1 - alpha) * self.last_target.point.x
            self.last_target.point.y = alpha * transformed.point.y + (1 - alpha) * self.last_target.point.y
            self.last_target.point.z = alpha * transformed.point.z + (1 - alpha) * self.last_target.point.z
            
        self.last_target_time = self.get_clock().now()

    def control_loop(self):    
        self.signal_sent = False
        if self.last_target is None or self.last_target_time is None:
            return
            
        age = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if age > self.target_timeout:
            self.get_logger().warn(f'No fresh detection for {age:.2f}s — holding position')
            return

        try:
            current_tf = self.tf_buffer.lookup_transform(
                self.planning_frame, 'link_tcp', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Could not look up current EE pose: {e}')
            return

        current = np.array([
            current_tf.transform.translation.x,
            current_tf.transform.translation.y,
            current_tf.transform.translation.z,
        ])
        target = np.array([
            self.last_target.point.x,
            self.last_target.point.y,
            self.last_target.point.z,
        ])
        
        error = target - current
        dist = np.linalg.norm(error)
        
        # self.get_logger().info(f'dist={dist:.4f}  target_z={target[2]:.4f}  current_z={current[2]:.4f}', throttle_duration_sec=0.5)
        
        if dist < self.min_approach_distance:
            self.signal_sent = False
            direction = - error / dist if dist > 1e-6 else np.zeros(3)
            speed = np.clip(self.p_gain * (0.2 - dist), 0, self.max_speed)
            velocity = speed * direction
        elif dist < self.deadband:
            if not self.signal_sent:
                self.get_logger().info('Posición alcanzada')
                msg = Bool()
                msg.data = True
                self.repeat_pub.publish(msg)
                self.signal_sent = True
            return
        else:
            direction = error / dist
            speed = np.clip(self.p_gain * dist, 0, self.max_speed)
            velocity = direction * speed
            
        current_rot = R.from_quat([current_tf.transform.rotation.x,
                                    current_tf.transform.rotation.y,
                                    current_tf.transform.rotation.z,
                                    current_tf.transform.rotation.w])
                                    
        error_rot = self.target_rotation * current_rot.inv()
        rotvec = error_rot.as_rotvec()
        angular_velocity = np.clip(self.p_gain_angular * rotvec, -self.max_angular_speed, self.max_angular_speed)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.planning_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = velocity.tolist()
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular_velocity.tolist()
        # self.get_logger().info(f"MENSAJE DE VELOCIDADES {msg}")
        self.twist_pub.publish(msg)


def main():
    rclpy.init()
    node = MouthServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
