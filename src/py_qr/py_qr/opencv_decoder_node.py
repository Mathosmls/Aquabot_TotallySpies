#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Importation du message String
import cv2
import json
from cv_bridge import CvBridge 

class OpenCvDecoder(Node):
    """Classe pour le nœud ROS2 qui décode les images de la caméra, décode les QR codes et publie l'état des éoliennes."""

    def __init__(self):
        super().__init__('opencv_decoder_node')
        self.get_logger().info('Hello world from opencv_decoder_node !')

        # Créez un éditeur (publisher) pour publier l'état des éoliennes
        self.publisher_ = self.create_publisher(String, 'windturbines_status', 10)

        # Créez un abonné au sujet de la caméra
        self.subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)

        # Utilisé pour la conversion entre les images ROS et OpenCV
        self.br = CvBridge()

        # Utilisé pour décoder les codes QR
        self.qr_decoder = cv2.QRCodeDetector()

        # Dictionnaire pour stocker l'état des éoliennes
        self.dico_windturbines = {}


    def image_callback(self, msg):
        # Convertir le message Image ROS en image OpenCV
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Décoder l'image
        data, bbox, rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)

        # if len(data) > 0:
        #     self.get_logger().info(f"QR code détecté: {data}")
        # else:
        #     self.get_logger().info("Aucun QR code détecté.")

        self.status_windturbin(msg)

    def status_windturbin(self, msg):
        """Permet de récupérer les informations de l'état des éoliennes et de les stocker dans un dictionnaire"""

        # Convertir le message Image ROS en image OpenCV
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Décoder l'image
        data, bbox, rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)

        if len(data) > 0:
            # Convertir la chaîne JSON en dictionnaire
            if isinstance(data, str):
                try:
                    data = json.loads(data)
                except json.JSONDecodeError:
                    self.get_logger().warn("Erreur de décodage des données JSON.")
                    return
            
            turbine_id = data.get("id")
            turbine_state = data.get("state")

            if not turbine_id in self.dico_windturbines.keys():
                # Ajouter une nouvelle éolienne
                self.dico_windturbines[turbine_id] = turbine_state
                self.get_logger().info(f"Nouvelle éolienne détectée: {turbine_id} avec état {turbine_state}")
                # Publier l'état des éoliennes si on a une nouvelle éolienne
                self.publish_windturbine_status()

            # Affichage de l'état actuel de toutes les éoliennes
            #self.get_logger().info(f"État actuel des éoliennes: {self.dico_windturbines}")

            

    def publish_windturbine_status(self):
        """Convertit le dictionnaire d'état des éoliennes en JSON et le publie."""
        # Convertir le dictionnaire en une chaîne JSON
        json_data = json.dumps(self.dico_windturbines)
        
        # Créer un message String ROS et le publier
        msg = String()
        msg.data = json_data
        self.publisher_.publish(msg)
        self.get_logger().info(f"État des éoliennes publié: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    opencv_decoder_node = OpenCvDecoder()
    rclpy.spin(opencv_decoder_node)

    # Détruire le nœud explicitement (facultatif)
    opencv_decoder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



