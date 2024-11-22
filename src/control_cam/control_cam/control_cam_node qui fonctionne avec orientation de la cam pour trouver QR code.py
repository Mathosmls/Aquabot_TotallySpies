#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
import cv2
from cv_bridge import CvBridge
import math

class ControlCamNode(Node):

    def __init__(self):
        super().__init__('control_cam_node')
        self.get_logger().info('Control_cam_node initialized successfully!')

        # Créer un éditeur pour contrôler l'angle de la caméra
        self.publisher_ = self.create_publisher(Float64, '/aquabot/thrusters/main_camera_sensor/pos', 10)

        # Créer un souscripteur pour recevoir les images
        self.subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)
        
        # Create a subscriber on the topic "/aquabot/sensors/cameras/main_camera_sensor/image_raw" which is the topic where the camera publishes images
        self.subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)

        # Créer un souscripteur pour recevoir les coordonnées GPS converties en coordonnées locales
        #self.position_subscriber = self.create_subscription(PoseArray, '/local_wind_turbine_positions', self.position_callback, 10)
        
        # Outil pour convertir entre les images ROS et OpenCV
        self.br = CvBridge()

        # Used to decode QR codes
        self.qr_decoder = cv2.QRCodeDetector()

        # Initialiser l'angle courant de la caméra
        self.current_angle = 0.0
        self.angle_increment = math.radians(1)  # Incrément d'angle de 1 degré


        # Initialiser les coordonnées du point cible (par défaut à None)
        self.target_position = None
        self.camera_position = None

        # Gain de régulation proportionnelle
        self.kp = 0.1  # à ajuster selon la réactivité souhaitée

    def set_camera_angle(self, target_angle):
        """Calcule et publie l'angle optimal pour orienter la caméra"""

        # Calculer la différence d'angle
        delta_angle = target_angle - self.current_angle

        # Ajuster pour minimiser la rotation (angle parcouru max : 180 degrés)
        if delta_angle > math.pi:
            target_angle -= 2 * math.pi
        elif delta_angle < -math.pi:
            target_angle += 2 * math.pi

        # Mettre à jour l'angle courant
        self.current_angle = target_angle

        # Publier l'angle sur le topic
        msg = Float64()
        msg.data = target_angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Camera angle set to: {target_angle} radians')


    def position_callback(self, msg):
        self.get_logger().info("position_callback called")
        if msg is not None:
            self.get_logger().info(f'Received target position: x={msg.position.x}, y={msg.position.y}')
        else:
            self.get_logger().warn("Received an empty message in position_callback")


    

    def image_callback(self, msg):
        #self.get_logger().info('Received image!')

        # Convertir le message ROS Image en image OpenCV
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Convertir l'image de BGR à RGB
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
    
        # Afficher l'image
        cv2.imshow("Image", current_frame)
        cv2.waitKey(1)

        # Decode image
        data,bbox,rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)

        if len(data) > 0:
            self.get_logger().info('Decoded data: ' + data)

        else:
            self.get_logger().info('No QR code detected')

            ###On ajuste l'angle de la caméra pour qu'elle puisse lire le QR code
            # Mettre à jour l'angle de la caméra
            new_angle = self.current_angle + self.angle_increment

            # Boucle pour ramener l'angle entre 0 et 2*pi
            if new_angle > 2 * math.pi:
                new_angle -= 2 * math.pi

            self.set_camera_angle(new_angle)

            """
        # Si la position cible et celle de la caméra sont disponibles, calculer l'angle
        if self.target_position and self.camera_position:
            target_x, target_y = self.target_position
            cam_x, cam_y = self.camera_position

            # Calcul de l'angle cible en radians
            target_angle = math.atan2(target_y - cam_y, target_x - cam_x)
            
            # Ajuster l'angle de la caméra pour qu'il corresponde à l'angle cible
            self.set_camera_angle(target_angle)
            """

def main(args=None):
    rclpy.init(args=args)

    # Initialisation du nœud ControlCamNode
    control_cam_node = ControlCamNode()

    # Boucle d'exécution du nœud
    rclpy.spin(control_cam_node)

    # Détruire le nœud et éteindre ROS2
    control_cam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
