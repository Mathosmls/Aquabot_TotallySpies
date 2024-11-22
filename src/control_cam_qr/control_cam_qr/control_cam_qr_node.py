#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseArray, Point
from nav_msgs.msg import Odometry
import cv2
import json
import math
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion
import numpy as np


class ControlCamQRNode(Node):
    """Nœud ROS2 combinant le contrôle de la caméra et le décodage des QR codes."""

    def __init__(self):
        super().__init__('control_cam_qr_node')
        self.get_logger().info('ControlCamQRNode initialized successfully!')

        # Publishers
        self.camera_angle_publisher = self.create_publisher(Float64, '/aquabot/thrusters/main_camera_sensor/pos', 10)
        self.windturbines_status_publisher = self.create_publisher(String, 'windturbines_status', 10)
        self.pos_gps_align_qr_publisher = self.create_publisher(PoseArray, 'pos_gps_align_qr', 10)

        # Subscribers
        self.image_subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)
        self.position_subscriber = self.create_subscription(Odometry, '/odometry/filtered/map', self.pos_callback, 10)

        # OpenCV 
        self.br = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()

        # Variables caméra
        self.current_angle = 0.0
        self.target_position = [10, 0]
        self.camera_position = [0, 0]
        
        self.pos_gps_align_qr = [0, 0]

        # Initialiser l'angle total de la caméra que l'on a modifié
        self.total_modified_angle = 0.0


        # Dictionnaire pour stocker l'état des éoliennes
        self.dico_windturbines = {}

    def pos_callback(self, msg):
        """Callback pour mettre à jour la position et l'orientation de la caméra."""
        self.camera_position[0] = msg.pose.pose.position.x
        self.camera_position[1] = msg.pose.pose.position.y
        self.current_angle = self.Quat2yaw(msg.pose.pose.orientation)

        # Ajuster l'angle entre 0 et 2pi
        if self.current_angle < 0:
            self.current_angle += 2 * math.pi

    def image_callback(self, msg):
        """Callback pour traiter les images de la caméra."""
        # Convertir le message ROS Image en image OpenCV
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Afficher l'image
        cv2.imshow("Camera View", cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB))
        cv2.waitKey(1)

        # Décoder les QR codes
        self.decode_qr_codes(current_frame)

        # Si les positions cibles et actuelles sont disponibles, ajuster la caméra
        if self.target_position and self.camera_position:
            target_x, target_y = self.target_position
            cam_x, cam_y = self.camera_position

            # Calculer l'angle cible
            target_angle = math.atan2(target_y - cam_y, target_x - cam_x)
            self.set_camera_angle(target_angle)

    def set_camera_angle(self, target_angle):
        """Calcule et publie l'angle optimal pour orienter la caméra."""
        delta_angle = target_angle - self.current_angle
        
        if delta_angle < 0:
            delta_angle += 2 * math.pi

        self.total_modified_angle += delta_angle
        print("self.total_modified_angle ",self.total_modified_angle )

        # Publier l'angle ajusté
        msg = Float64()
        msg.data = delta_angle
        self.camera_angle_publisher.publish(msg)
        self.get_logger().info(f'Camera angle adjusted to: {delta_angle:.2f} radians')
        

    def decode_qr_codes(self, frame):
        """Décoder les QR codes et mettre à jour l'état des éoliennes."""
        data, bbox, _ = self.qr_decoder.detectAndDecode(frame)

        if len(data) > 0:
            try:
                data = json.loads(data)
            except json.JSONDecodeError:
                self.get_logger().warn("Erreur de décodage JSON.")
                return

            turbine_id = data.get("id")
            turbine_state = data.get("state")

            if turbine_id not in self.dico_windturbines:
                self.dico_windturbines[turbine_id] = turbine_state
                #On ne publie le dico que quand une nouvelle éolienne est détectée
                self.publish_windturbine_status()

    def publish_windturbine_status(self):
        """Publier l'état des éoliennes sous forme de JSON."""
        msg = String()
        msg.data = json.dumps(self.dico_windturbines)
        self.windturbines_status_publisher.publish(msg)
        self.get_logger().info(f"État des éoliennes publié: {msg.data}")

    def Quat2yaw(self,quat_msg) :
        quaternion = (  quat_msg.x,
                        quat_msg.y,
                        quat_msg.z,
                        quat_msg.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        return yaw
    
    def publish_pos_gps_align_qr(self):
        """Publier la nouvelle position GPS pour être détecter le Qr code en face."""
        # Créer un objet PoseArray
        pose_array = PoseArray()

        pos_x = self.pos_gps_align_qr[0]
        pos_y = self.pos_gps_align_qr[1]
    
        pose_array.poses.append(Point(x=pos_x, y=pos_y, z=0.0)) #la cote z n'est pas utile ici


        #on publie les coordonnées gps pour être en face du Qr Code
        self.pos_gps_align_qr_publisher.publish(pose_array)
        self.get_logger().info(f"Nouvelle position GPS publiée: {pos_x, pos_y}")
        


def main(args=None):
    rclpy.init(args=args)

    control_cam_qr_node = ControlCamQRNode()

    rclpy.spin(control_cam_qr_node)

    control_cam_qr_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
