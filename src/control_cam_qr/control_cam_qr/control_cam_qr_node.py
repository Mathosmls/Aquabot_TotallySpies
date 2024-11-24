#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseArray, Point,PoseStamped
from nav_msgs.msg import Odometry
import cv2
import json
import math
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


class ControlCamQRNode(Node):
    """Nœud ROS2 combinant le contrôle de la caméra et le décodage des QR codes."""

    def __init__(self):
        super().__init__('control_cam_qr_node')
        self.get_logger().info('ControlCamQRNode initialized successfully!')

        # Publishers
        self.camera_angle_publisher = self.create_publisher(Float64, '/aquabot/thrusters/main_camera_sensor/pos', 10)
        self.windturbines_status_publisher = self.create_publisher(String, '/vrx/windturbinesinspection/windturbine_checkup', 10)
        self.pos_gps_align_qr_publisher = self.create_publisher(PoseStamped, '/pos_gps_align_qr', 10)

        # Subscribers
        self.image_subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)
        self.position_subscriber = self.create_subscription(Odometry, '/odometry/filtered/map', self.pos_callback, 10)
        self.target_pos_subscriber=self.create_subscription(Point, '/current_wt', self.target_pos_callback, 10)

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
        self.angle0=False
        self.pos_qr=np.array([0.,0.,0.])
        self.poses_qr=[]
        self.best_poses_qr=[]


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
        cv2.waitKey(5)

        # Décoder les QR codes
        self.decode_qr_codes(current_frame)

        # Si les positions cibles et actuelles sont disponibles, ajuster la caméra
        # if self.target_position and self.camera_position:
        #     target_x, target_y = self.target_position
        #     cam_x, cam_y = self.camera_position

        #     # Calculer l'angle cible
        #     target_angle = math.atan2(target_y - cam_y, target_x - cam_x)
        #     self.set_camera_angle(target_angle)

    def set_camera_angle(self, target_angle):
        """Calcule et publie l'angle optimal pour orienter la caméra."""
        delta_angle = target_angle - self.current_angle
        
        if delta_angle < 0:
            delta_angle += 2 * math.pi


        # Publier l'angle ajusté
        msg = Float64()
        msg.data = delta_angle
        # self.camera_angle_publisher.publish(msg)
        

    def decode_qr_codes(self, frame):
        """Décoder les QR codes et mettre à jour l'état des éoliennes."""

        if frame is None or frame.size == 0:
            print("Erreur : L'image est vide ou invalide.")
            return
        
        if frame.shape[0] == 0 or frame.shape[1] == 0:
            print("Erreur : L'image a une taille invalide.")
            return
            # Essayez de détecter et décoder les QR codes
        try:
            data, bbox, _ = self.qr_decoder.detectAndDecode(frame)
        except cv2.error as e:
            # Capturer l'erreur OpenCV et gérer l'exception
            print(f"Erreur lors de la détection du QR code avec OpenCV")
            return  # Sortir de la fonction sans continuer
        
        if len(data) > 0:
            angle=self.calculate_angle_from_bbox(bbox)
            print(angle)
            # si le qr code est aligné, on enregistre la pos calculée pour être à 10m de l'éolienne 
            if abs(angle)<0.02 :
                # print(bbox)
                self.angle0=True
                if abs(angle)<=0.01021 :
                    self.pos_qr=self.calculate_position_at_10m_from_eolienne(self.target_position, self.camera_position)
                    self.poses_qr.append(self.pos_qr)
                    if len(self.poses_qr)>len(self.best_poses_qr):
                        self.best_poses_qr=self.poses_qr
                else :
                    self.poses_qr=[]
                # print("here",angle,self.pos_qr,self.camera_position,self.target_position)
                return
            if abs(angle)>0.1 and self.angle0 :
                self.angle0=False
                print(self.best_poses_qr)
                middle_index = (len(self.best_poses_qr) - 1) // 2  
                self.pos_qr=self.best_poses_qr[middle_index]
                print("pos_opti" ,self.pos_qr)
                try:
                    data = json.loads(data)
                except json.JSONDecodeError:
                    self.get_logger().warn("Erreur de décodage JSON.")
                    return

                turbine_id = data.get("id")
                turbine_state = data.get("state")

                if turbine_id not in self.dico_windturbines:
                    self.dico_windturbines[turbine_id] = turbine_state
                    pose_qr=self.create_pose_stamped(self.pos_qr,turbine_id)
                    print("published pose_qr", pose_qr)
                    #On ne publie le dico que quand une nouvelle éolienne est détectée
                    self.publish_windturbine_status()
                    self.pos_gps_align_qr_publisher.publish(pose_qr)

    def publish_windturbine_status(self):
        """Publier l'état des éoliennes sous le format attendu par le récepteur."""
        for turbine_id, turbine_state in self.dico_windturbines.items():
            # Construire le JSON directement
            msg = String()
            msg.data = json.dumps({"id": turbine_id, "state": turbine_state})  # Pas de champ "data" externe
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
    
        
    def target_pos_callback(self,msg) :
        self.target_position[0]=msg.x
        self.target_position[1]=msg.y

    
    def calculate_angle_from_bbox(self,bbox):
        """
        Calcule l'angle de rotation du QR code par rapport à l'axe X (horizontal).
        Supposition : le QR code est toujours aligné avec le sol (plan XY).
        """
        if bbox is not None and len(bbox) == 1 and len(bbox[0]) == 4:  # Vérification que bbox a 4 coins
            # Extraire les coordonnées des coins du QR code
            pt1 = np.array(bbox[0][0])  # Coin supérieur gauche
            pt2 = np.array(bbox[0][1])  # Coin supérieur droit
            
            # Calculer le vecteur du coin supérieur gauche au coin supérieur droit
            vector_top = pt2 - pt1  # Vecteur horizontal
            
            # Calculer l'angle de ce vecteur par rapport à l'axe horizontal (X)
            angle_rad = math.atan2(vector_top[1], vector_top[0])  # atan2 donne l'angle par rapport à X
            angle_rad-=np.pi
            angle_rad = (angle_rad + np.pi) % (2 * np.pi) - np.pi
            return angle_rad
        else:
            print("Erreur : bbox mal formée ou vide.")
            return 0  # Si bbox est mal formée ou vide, renvoyer 0 comme angle
    

    def calculate_position_at_10m_from_eolienne(self, pos_eolienne, pos_bateau, distance=10):
        """
        Calcule la position du bateau à 10m de l'éolienne, en face du QR code.

        :param x_eolienne: Coordonnée x de l'éolienne
        :param y_eolienne: Coordonnée y de l'éolienne
        :param x_bateau: Coordonnée x du bateau
        :param y_bateau: Coordonnée y du bateau
        :param distance: Distance à laquelle le bateau doit être positionné (par défaut 10m)
        :return: Nouvelle position du bateau (x, y)
        """
        # Calcul de l'angle entre l'éolienne et le bateau actuel
        dx = pos_bateau[0] - pos_eolienne[0]
        dy = pos_bateau[1] - pos_eolienne[1]
        angle_eolienne_bateau = math.atan2(dy, dx)  # L'angle entre l'éolienne et le bateau
        
        # Calcul de la nouvelle position du bateau à 10 mètres de l'éolienne, en face
        x_new = pos_eolienne[0] + distance * np.cos(angle_eolienne_bateau)
        y_new = pos_eolienne[1] + distance * np.sin(angle_eolienne_bateau)
        angle=angle_eolienne_bateau-np.pi
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return np.array([x_new,y_new,angle])
    
    def create_pose_stamped(self,array,id):
        if len(array) != 3:
            raise ValueError("L'entrée doit être un tableau de longueur 3 : [x, y, theta].")
        
        # Création du message PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'  # Référence du cadre
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp actuel
        
        # Position
        pose_msg.pose.position.x = array[0]
        pose_msg.pose.position.y = array[1]
        pose_msg.pose.position.z = float(id)  # on triche pour envoyer l'id
        
        # Orientation : conversion de theta (yaw) en quaternion
        quaternion = quaternion_from_euler(0, 0, array[2])  
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        return pose_msg

def main(args=None):
    rclpy.init(args=args)

    control_cam_qr_node = ControlCamQRNode()

    rclpy.spin(control_cam_qr_node)

    control_cam_qr_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
