import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class QRCodeAngleEstimator(Node):
    def __init__(self):
        super().__init__('qrcode_angle_estimator')
        self.subscription = self.create_subscription(
            Image,
            '/aquabot/sensors/cameras/main_camera_sensor/image_raw',  
            self.image_callback,
            10
        )
        self.gps_subscription = self.create_subscription(
            PoseStamped,
            "/loc_gps", 
            self.gps_callback,
            10
        )
        self.gps_publication = self.create_publisher(PoseStamped, "/gps_qr_code", 10)

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.angle_radians = 0  # Correction ici
        self.angle_degrees = 0
        self.qr_L = 1.5 
        self.windturbine = [10., 0.]
        self.my_pose= [0, 0]
        self.alpha = 0
        self.dist = 0
    

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        data, points, _ = self.detector.detectAndDecode(frame)

        if points is not None:
            points = points[0].astype(int)

            top_length = np.linalg.norm(points[0] - points[1])
            bottom_length = np.linalg.norm(points[3] - points[2])
            dist = np.sqrt((self.windturbine[0] - self.my_pose[0])**2 + (self.windturbine[1] - self.my_pose[1])**2)  
            print("dist", dist)
            self.dist = dist
            self.alpha = np.arctan2(self.windturbine[1] - self.my_pose[1], self.windturbine[0] - self.my_pose[0])
            print("alpha", self.alpha)   
            f_image = top_length *dist/self.qr_L
            self.angle_radians = math.atan(top_length / f_image) 
            self.angle_degrees = math.degrees(self.angle_radians)   
            
            self.get_logger().info(f"Angle horizontal estimé : {self.angle_degrees:.2f}°")

            for i in range(4):
                cv2.line(frame, tuple(points[i]), tuple(points[(i + 1) % 4]), (0, 255, 0), 2)
            cv2.putText(frame, f"Angle: {self.angle_degrees:.2f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
        # Afficher l'image traitée
        cv2.imshow("QR Code Detection", frame)
        cv2.waitKey(1)

    def gps_callback(self, msg):
        self.get_logger().info(f"Position GPS : lx={msg.pose.position.x:.6f}, y={msg.pose.position.y:.6f}")
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.my_pose = [x, y]   
        print("angles", self.angle_radians+self.alpha)

        x_bar = self.windturbine[0] +10*math.cos(self.angle_radians) 
        y_bar = self.windturbine[1] +10*math.sin(self.angle_radians) 

        # v_hat = np.array([self.windturbine[0]-x_bar, self.windturbine[1]-y_bar])
        # v_hat = v_hat / np.linalg.norm(v_hat)

        # x_bar = self.windturbine[0] + 10*v_hat[0]   
        # y_bar = self.windturbine[1] + 10*v_hat[1]

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()  
        pose_stamped.header.frame_id = "map"  
        pose_stamped.pose.position.x = x_bar
        pose_stamped.pose.position.y = y_bar
        pose_stamped.pose.position.z = 0.0  

        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        self.gps_publication.publish(pose_stamped)
        self.get_logger().info(f"Position GPS corrigée : lx={x_bar:.6f}, y={y_bar:.6f}")

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeAngleEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
