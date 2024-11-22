#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


import cv2

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge 

class OpenCvDecoder(Node):

    def __init__(self):
        super().__init__('opencv_decoder_node')
        self.get_logger().info('Hello world from opencv_decoder_node !')

        # Create a subscriber on the topic "/aquabot/sensors/cameras/main_camera_sensor/image_raw" which is the topic where the camera publishes images
        self.subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Used to decode QR codes
        self.qr_decoder = cv2.QRCodeDetector()

        # Create a dictionary to store the id and state of the windturbines
        self.dico_windturbines = {}

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Decode image
        data,bbox,rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)

        if len(data) > 0:
            self.get_logger().info('Decoded data: ' + data)
        else:
            self.get_logger().info('No QR code detected')


     #### FONCTION A VÉRIFIER, REPRENDRE ICI ####
    def status_windturbin(self, msg):
        """Permet de récupérer les informations de l'état des éoliennes et de les stocker dans un dictionnaire"""

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Decode image
        data, bbox, rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)

        # Forme de data : data = {"id": 0, "state": "KO"}
        if len(data) > 0:
            # Convertir le string data en dictionnaire si nécessaire (si c'est sous forme de string JSON)
            if isinstance(data, str):
                import json
                try:
                    data = json.loads(data)
                except json.JSONDecodeError:
                    self.get_logger().warn("Erreur de décodage des données JSON.")
                    return
            
            # Vérification si l'id du QR code est déjà dans le dictionnaire d'éoliennes
            turbine_id = data.get("id")
            turbine_state = data.get("state")

            if turbine_id in self.dico_windturbines:
                # Si l'id est déjà présent dans le dictionnaire, on compare l'état
                if self.dico_windturbines[turbine_id] != turbine_state:
                    self.get_logger().info(f"État changé pour l'éolienne {turbine_id}: {turbine_state}")
                    # Mettez à jour l'état si nécessaire
                    self.dico_windturbines[turbine_id] = turbine_state
                else:
                    self.get_logger().info(f"État de l'éolienne {turbine_id} inchangé: {turbine_state}")
            else:
                # Si l'id n'est pas présent, on ajoute l'éolienne au dictionnaire
                self.dico_windturbines[turbine_id] = turbine_state
                self.get_logger().info(f"Nouvelle éolienne détectée: {turbine_id} avec état {turbine_state}")
            
            # Affichage de l'état actuel de toutes les éoliennes
            self.get_logger().info(f"État actuel des éoliennes: {self.dico_windturbines}")

        


def main(args=None):
    rclpy.init(args=args)

    opencv_decoder_node = OpenCvDecoder()

    rclpy.spin(opencv_decoder_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    opencv_decoder_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
