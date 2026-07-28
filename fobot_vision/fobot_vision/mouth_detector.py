import cv2
import mediapipe as mp 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from collections import deque
import time

#Building The Landmark Detector

def landmark_px(lm, w, h):
    return np.array([lm.x * w, lm.y * h])

def iris_diameter_px(landmarks, w, h, right_idx, top_idx, left_idx, bottom_idx):
    right = landmark_px(landmarks.landmark[right_idx], w, h)
    left = landmark_px(landmarks.landmark[left_idx], w, h)
    top = landmark_px(landmarks.landmark[top_idx], w, h)
    bottom = landmark_px(landmarks.landmark[bottom_idx], w, h)
    horizontal = np.linalg.norm(right - left)
    vertical = np.linalg.norm(top - bottom)
    return (horizontal + vertical) / 2.0
    
def mouth_center(landmarks, w, h, mouth_idx):
    sum_x = 0
    sum_y = 0
    
    for idx in mouth_idx:
        landmark = landmarks.landmark[idx]
        
        sum_x += landmark.x * w
        sum_y += landmark.y * h
        
    center_x = int(sum_x / 4)
    center_y = int(sum_y / 4)
    
    return (center_x, center_y)
    
def lips_distance(landmarks, w, h):
    top_lip = landmark_px(landmarks.landmark[13], w, h)
    bottom_lip = landmark_px(landmarks.landmark[14], w, h)
    
    return np.sqrt((top_lip[0] - bottom_lip[0])**2 + (top_lip[1] - bottom_lip[1])**2)

class MouthDetector(Node):
    def __init__(self):
        super().__init__('mouth_detector_node')
        
        self.declare_parameter('show_debug_window', False)
        self.show_debug_window = self.get_parameter('show_debug_window').value
        
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_cb, 10)
        
        self.create_subscription(String, '/FOBOT/state', self.state_cb, 10)
        
        self.position_pub = self.create_publisher(PointStamped, '/mouth_position', 10)
        
        self.state_pub = self.create_publisher(String, '/FOBOT/movement_done', 10)
        
        self.interface_pub = self.create_publisher(String, '/FOBOT/interface', 10)
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_face_mesh = mp.solutions.face_mesh
        self.drawing_spec = self.mp_drawing.DrawingSpec(thickness=1, circle_radius=2)

        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, '/image_raw', self.detection_cb, 10)

        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
            
        self.iris_diameter = 0.0117
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        
        self.iris_history = deque(maxlen=5)
        
        self.frame_count = 0
        
        self.init_eyes_closed = None
        self.stop_time = 2.5
        
        self.actual_state = 'INICIO'
        
    def state_cb(self, msg):
        self.actual_state = msg.data
            
    def camera_info_cb(self, msg):
        if msg:
            k_matrix = np.array(msg.k).reshape((3, 3))
            
            self.fx = k_matrix[0, 0]
            self.cx = k_matrix[0, 2]
            self.fy = k_matrix[1, 1]
            self.cy = k_matrix[1, 2]
        
        self.destroy_subscription(self.cam_info_sub)
        self.cam_info_sub = None
        
    def detection_cb(self, msg):
        self.frame_count += 1
        if self.frame_count % 3 != 0:
            return
    	
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo la imagen: {e}")
            return
            
        if self.fx == 0.0:
            return
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB color
        results = self.face_mesh.process(image)
        image.flags.writeable = True
        
        h, w = image.shape[:2]
        
        point = PointStamped()
        point.header.stamp = msg.header.stamp
        point.header.frame_id = 'link_camara'

        if results.multi_face_landmarks:

          for face_landmarks in results.multi_face_landmarks:

            # gets lips landmarks
            self.mp_drawing.draw_landmarks(
                image=image,
                landmark_list=face_landmarks,
                connections=self.mp_face_mesh.FACEMESH_LIPS,
                landmark_drawing_spec=None,
                connection_drawing_spec=self.mp_drawing_styles
                .get_default_face_mesh_contours_style())

                # gets Eyes landmarks
            self.mp_drawing.draw_landmarks(
                image=image,
                landmark_list=face_landmarks,
                connections=self.mp_face_mesh.FACEMESH_IRISES,
                landmark_drawing_spec=None,
                connection_drawing_spec=self.mp_drawing_styles
                .get_default_face_mesh_iris_connections_style())
                
            #  Right eye
            right_eye_diam = iris_diameter_px(face_landmarks, w, h, 469, 470, 471, 472)
            # Left eye
            left_eye_diam = iris_diameter_px(face_landmarks, w, h, 474, 475, 476, 477)
            # mean of both eyes
            iris_px_diam = (right_eye_diam + left_eye_diam) / 2.0
            
            self.iris_history.append(iris_px_diam)
            iris_px_diam_smoothed = float(np.median(self.iris_history))
            
            if right_eye_diam > 0 and left_eye_diam > 0:
                ratio = max(right_eye_diam, left_eye_diam) / min(right_eye_diam, left_eye_diam)
                if ratio > 1.3:  # eyes disagree by more than ~30% — don't trust this frame
                    return
            
            # self.get_logger().info(f"IRIS DERECHO: {right_eye_diam} | IRIS IZQUIERDO {left_eye_diam} | MEDIA IRIS {iris_px_diam}")
            
            mouth_idx = [13, 14, 61, 291]
            
            u, v = mouth_center(face_landmarks, w, h, mouth_idx)
            
            Z_opt = (self.iris_diameter * self.fx) / iris_px_diam_smoothed
            Y_opt = (v - self.cy) * Z_opt / self.fy
            X_opt = (u - self.cx) * Z_opt / self.fx
            
            self.get_logger().info(f"X: {Z_opt} | Y: {-X_opt} | Z: {-Y_opt}")
            
            # self.get_logger().info(f"DISTANCIA Z {Z_opt}")
            
            point.point.x = Z_opt
            point.point.y = -X_opt
            point.point.z = -Y_opt
            
            # Condición de movimiento si se abre la boca
            lips_dist = lips_distance(face_landmarks, w, h)
            
            '''if lips_dist > 20.0:
                self.position_pub.publish(point)'''
                
            if lips_dist > 20.0 and self.actual_state == 'MODO_SERVOING':
                self.position_pub.publish(point)
            elif lips_dist > 20.0 and self.actual_state == 'ESPERA':
                auto_msg = String()
                auto_msg.data = 'auto'
                self.interface_pub.publish(auto_msg)
                
            # Condición de parada por cierre de ojos
            p_sup_left = landmark_px(face_landmarks.landmark[159], w, h)
            p_inf_left = landmark_px(face_landmarks.landmark[145], w, h)
            p_sup_right = landmark_px(face_landmarks.landmark[386], w, h)
            p_inf_right = landmark_px(face_landmarks.landmark[374], w, h)
            
            op_left = np.linalg.norm(p_sup_left - p_inf_left)
            op_right = np.linalg.norm(p_sup_right - p_inf_right)
            
            mean_op = (op_left + op_right) / 2.0
            
            if mean_op < 5.0 and self.actual_state == 'MODO_SERVOING':
                if self.init_eyes_closed is None:
                    self.init_eyes_closed = time.time()
                else:
                    elapsed_time = time.time() - self.init_eyes_closed
                    if elapsed_time > self.stop_time:
                        state_msg = String()
                        state_msg.data = "stop"
                        self.state_pub.publish(state_msg)
                        self.init_eyes_closed = None
            else:
                self.init_eyes_closed = None
                
        else:
            self.init_eyes_closed = None

        if self.show_debug_window:
            display_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.putText(display_image, "Mouth", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('MediaPipe Face Mesh', display_image)
            cv2.waitKey(1)
          
def main(args=None):
    rclpy.init(args=args)
    node = MouthDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
