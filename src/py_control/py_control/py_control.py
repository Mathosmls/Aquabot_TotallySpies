

#------------------------------------------------------------------------------------------------
# ROS NODE
#------------------------------------------------------------------------------------------------
from mppi_utils import MPPIControllerForAquabot
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion
from scipy.spatial import KDTree
import math
class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        print("Node controller")
        self.publisherMotorL = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 10)
        self.publisherMotorR = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 10)
        self.publisherAngleMotorL = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 10)
        self.publisherAngleMotorR = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 10)

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered/map/processed',
            self.odom_callback,
            10)

        self.subscription_path = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)

        self.subscription_mode = self.create_subscription(
            UInt8,
            '/mode',
            self.mode_callback,
            10)
        
        self.current_state = np.array([30.0, 0.0, 0.0, 0.0,0.0,0.0])
        self.target_state=np.array([-9999,-9999,-9999])
        self.final_state=np.array([30.0,0.0,0.0])
        self.mppi0=MPPIControllerForAquabot(horizon_step_T=90,sigma=np.array([40, 40, 0.1, 0.1]), dt=0.1,
                                            max_thrust=800,stage_cost_weight=np.array([45.0, 45.0, 300.0]) )
        self.mppi1=MPPIControllerForAquabot(horizon_step_T=25,sigma=np.array([80, 80, 0.1, 0.1]), dt=0.1,
                                            max_thrust=2500,stage_cost_weight=np.array([50.0, 2000.0, 15.0]) )
        self.mppi2=MPPIControllerForAquabot(horizon_step_T=55,sigma=np.array([80, 80, 0.1, 0.1]), dt=0.1,
                                            max_thrust=1250,stage_cost_weight=np.array([1.0, 60.0, 1.5]) )
        self.mppi3=MPPIControllerForAquabot(horizon_step_T=55,sigma=np.array([60, 60, 0.1, 0.1]), dt=0.05,
                                            max_thrust=700,stage_cost_weight=np.array([10.0, 11.0, 150.0]) )
        self.mppis=np.array([self.mppi0,self.mppi1,self.mppi2,self.mppi3])
        self.plan=Path()
        self.plan_array=np.array([[0,0]])
        self.close_points=np.array([[0,0]])
        self.mode=3 # 0 :go to point | 1: follow path | 2: do a circle for qr search | 3: do the bonus phase circle
  


    #--------Callbacks-----------
    def timer_callback(self):
        motorL=Float64()
        motorR=Float64()
        angleMotorL=Float64()
        angleMotorR=Float64()
        # print(self.target_state)
        if self.mode==1 and len(self.plan.poses)<=1 :
            control, _ = self.mppis[0].calc_control_input(self.current_state, self.target_state,self.mode,self.close_points)
            self.get_logger().info('Controller in path mode but no plan given' )
        else :
            control, _ = self.mppis[self.mode].calc_control_input(self.current_state, self.target_state,self.mode,self.close_points)
        motorL.data=control[0]
        motorR.data=control[1]
        angleMotorL.data=control[2]
        angleMotorR.data=control[3]
        if math.isnan(control[0]):
            self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))
            print("err, nan detected, reseting the controller...")

        self.publisherMotorL.publish(motorL)
        self.publisherMotorR.publish(motorR)
        self.publisherAngleMotorL.publish(angleMotorL)
        self.publisherAngleMotorR.publish(angleMotorR)
        self.mode_cmd()

        self.get_logger().info('thrust:  "%f"  "%f"' % (control[0], control[1]))
        self.get_logger().info('angle:  "%f"  "%f"' % (angleMotorL.data,  angleMotorR.data))
        

    def odom_callback(self, msg):
        self.current_state[0]=msg.pose.pose.position.x
        self.current_state[1]=msg.pose.pose.position.y
        yaw = self.Quat2yaw(msg.pose.pose.orientation)
        self.current_state[2]=yaw
        self.current_state[3]=msg.twist.twist.linear.x
        self.current_state[4]=msg.twist.twist.linear.y
        self.current_state[5]=msg.twist.twist.angular.z
        if self.target_state[0]==-9999:
            #Jsp pas pourquoi mais  self.target_state =self.plan.poses[:3] fait un passage par référence
            self.target_state=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,yaw])
        
        
        
    def plan_callback(self, msg):
        print("Plan received")
        self.plan=msg
        yaw = self.Quat2yaw(self.plan.poses[-1].pose.orientation)
        x,y=self.plan.poses[-1].pose.position.x,self.plan.poses[-1].pose.position.y
        self.final_state=np.array([x,y,yaw])
        self.extract_positions_from_path()
        self.mode=1
        self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))

    def mode_callback(self,msg):
        if 0<=msg.data<=3 :
            self.mode=int(msg.data)
            self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))
            self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))
            self.get_logger().info('mode :"%f"'% (self.mode))



    #--------Utils-----------
    def Quat2yaw(self,quat_msg) :
        quaternion = (  quat_msg.x,
                        quat_msg.y,
                        quat_msg.z,
                        quat_msg.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        return yaw
    
    #to configure the mppi in function of the mode
    def mode_cmd(self):
        if self.mode ==0  : #follow point
            self.timer_period = 0.1

        elif self.mode ==1 : #follow path/
            self.timer_period = 0.1
            d_final =np.linalg.norm(np.array([self.current_state[0], self.current_state[1]]) 
                                    - np.array([self.final_state[0], self.final_state[1]]))

            if len(self.plan.poses)>1 :
              
                self.close_points,id_closest_point=self.get_closest_points(self.current_state[:2],self.plan_array,5)
                i=min(8,len(self.plan.poses)-id_closest_point-1)
                self.target_state[2]=self.Quat2yaw(self.plan.poses[id_closest_point+i].pose.orientation)

                if d_final <7 :
                    self.mode=0 
                    self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))
                    self.target_state[0]=self.plan.poses[-1].pose.position.x
                    self.target_state[1]=self.plan.poses[-1].pose.position.y
                    self.target_state[2]=self.Quat2yaw(self.plan.poses[-1].pose.orientation)
            # else :
            #     #Jsp pas pourquoi mais  self.target_state =self.current_state[:3] fait un passage par référence
            #     self.target_state=np.array([self.current_state[0],self.current_state[1],self.current_state[2]])

        elif self.mode ==2 : #follow cicrle for qr code
            self.timer_period = 0.1
     

        elif self.mode ==3 : #follow cicrle for bonus phase
            self.timer_period = 0.05
            
    # Retourner un tableau numpy avec toutes les positions [x, y] du plan, plsu pratique pour le traitement
    def extract_positions_from_path(self) -> np.ndarray:
        positions = []
        
        for pose_stamped in self.plan.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            positions.append([x, y])
        self.plan_array=np.array(positions)

    def get_closest_points(self,USV_position, all_points, threshold):
        # Créer un KD-Tree avec tous les points
        tree = KDTree(all_points)
        
        # Trouver les indices des points proches (dans un rayon donné)
        indices_proches = tree.query_ball_point(USV_position, threshold)
        _, index = tree.query(USV_position)
        
        # Extraire les points proches
        closest_points = all_points[indices_proches]
        
        return closest_points,index




def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

