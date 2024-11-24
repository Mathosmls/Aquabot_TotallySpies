

#------------------------------------------------------------------------------------------------
# ROS NODE, used to control the AQUABOT thanks to a mppi
#------------------------------------------------------------------------------------------------
from py_control.mppi_utils import MPPIControllerForAquabot
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial import KDTree
import math
import time
from py_control.mppi_pythran_normal import compute_lateral_error

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Controller initialized successfully!')
        self.publisherMotorL = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 10)
        self.publisherMotorR = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 10)
        self.publisherAngleMotorL = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 10)
        self.publisherAngleMotorR = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 10)
        self.publisherGoalPose= self.create_publisher(PoseStamped, '/goal_pose', 10)

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
        
        self.subscription_goal_target= self.create_subscription(
            PoseStamped,
            '/goal_target',
            self.goal_target_callback,
            10)
        
        self.current_state = np.array([0.0, 0.0, 0.0, 0.0,0.0,0.0])
        self.target_state=np.array([-9999.0,-9999.0,-9999.0])
        self.goal_target=self.target_state
        self.mppi0=MPPIControllerForAquabot(horizon_step_T=75,sigma=np.array([30, 30, 0.1, 0.1]), dt=0.1,
                                            max_thrust=250.0,stage_cost_weight=np.array([45.0, 45.0, 300.0]) )
        self.mppi1=MPPIControllerForAquabot(horizon_step_T=25,sigma=np.array([60, 60, 0.1, 0.1]), dt=0.1,
                                            max_thrust=2500.0,stage_cost_weight=np.array([60.0, 2000.0, 69.0]) )
        self.mppi2=MPPIControllerForAquabot(horizon_step_T=55,sigma=np.array([40, 40, 0.05, 0.05]), dt=0.1,
                                            max_thrust=1250.0,stage_cost_weight=np.array([1.0, 10.0, 0.5]) )
        self.mppi3=MPPIControllerForAquabot(horizon_step_T=50,sigma=np.array([200, 200, 0.1, 0.1]), dt=0.1,
                                            max_thrust=450.0,stage_cost_weight=np.array([20.0, 15.0, 50.0]) )
        self.mppi4=MPPIControllerForAquabot(horizon_step_T=25,sigma=np.array([60, 60, 0.1, 0.1]), dt=0.1,
                                    max_thrust=300.0,stage_cost_weight=np.array([600.0, 1500.0, 5.0]) )
        
        self.mppis=np.array([self.mppi0,self.mppi1,self.mppi2,self.mppi3,self.mppi4])
        self.plan=Path()
        self.plan_array=np.array([[0,0]])
        self.close_points=np.array([[0.0,0.0]])
        self.mode=0 # 0 :go to point | 1: follow path | 2: do a circle for qr search | 3: do the bonus phase circle


    #--------Callbacks-----------
    def timer_callback(self):
        if self.target_state[0]!=-9999.0:
            motorL=Float64()
            motorR=Float64()
            angleMotorL=Float64()
            angleMotorR=Float64()
            self.mode_cmd()
            if self.mode==1 and len(self.plan.poses)<=1 :
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


        
    #update the usv pose
    def odom_callback(self, msg):
        self.current_state[0]=msg.pose.pose.position.x
        self.current_state[1]=msg.pose.pose.position.y
        yaw = self.Quat2yaw(msg.pose.pose.orientation)
        self.current_state[2]=yaw
        self.current_state[3]=msg.twist.twist.linear.x
        self.current_state[4]=msg.twist.twist.linear.y
        self.current_state[5]=msg.twist.twist.angular.z
        if self.target_state[0]==-9999.0:
            self.target_state=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,yaw])
            self.goal_target=self.target_state
        

    #handle path planning
    def plan_callback(self, msg):
        print("Plan received")
        self.plan=msg
        self.extract_positions_from_path()
        if self.mode != 4: 
            self.mode=1
        self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))

    #handle the mode callback
    def mode_callback(self,msg):
        if 0<=msg.data<=4 :
            self.mode=int(msg.data)
            self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))
            self.get_logger().info('mode :"%f"'% (self.mode))
        else :
            self.get_logger().info('Please provide a mode between 0 and 4')

        
    #hanlde goal_target (for circle and point)        
    def goal_target_callback(self,msg) :
        self.goal_target[0]=msg.pose.position.x
        self.goal_target[1]=msg.pose.position._y
        self.goal_target[2]=self.Quat2yaw(msg.pose.orientation)

    #--------Utils-----------
    def Quat2yaw(self,quat_msg) :
        quaternion = (  quat_msg.x,
                        quat_msg.y,
                        quat_msg.z,
                        quat_msg.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        return yaw
    
    #to handle the behavior of the controller in function of the mode
    def mode_cmd(self):

        if self.mode ==1 : #follow path fast
            d_final =np.linalg.norm(np.array([self.current_state[0], self.current_state[1]]) 
                                    - np.array([self.goal_target[0], self.goal_target[1]]))
            if len(self.plan.poses)>1 : #if path
                
                self.close_points,id_closest_point=self.get_closest_points(self.current_state[:2],self.plan_array,4.2)
                self.close_points=np.asarray(self.close_points, dtype=np.float64)
                err=compute_lateral_error(self.current_state,self.close_points)
                # print("err : ",err)
                if err==0.0: #if to far from path, ask new path
                    print("ask new path")
                    final_pose=self.create_pose_stamped(self.goal_target)
                    self.publisherGoalPose.publish(final_pose)
                i=min(15,len(self.plan.poses)-id_closest_point-1) #take the angle 15 position above current
                self.target_state[2]=self.Quat2yaw(self.plan.poses[id_closest_point+i].pose.orientation)

                if d_final <10 : #if close to target, go directly to the point
                    self.target_state=self.goal_target
                    self.mode=0 
                    self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))

        elif self.mode ==4 : #follow path slow, same than before with some changes in parameters
            d_final =np.linalg.norm(np.array([self.current_state[0], self.current_state[1]]) 
                                    - np.array([self.goal_target[0], self.goal_target[1]]))

            if len(self.plan.poses)>1 :
                
                self.close_points,id_closest_point=self.get_closest_points(self.current_state[:2],self.plan_array,4.0)
                self.close_points=np.asarray(self.close_points, dtype=np.float64)
                err=compute_lateral_error(self.current_state,self.close_points)
                if err==0.0:
                    print("ask new path")
                    final_pose=self.create_pose_stamped(self.goal_target)
                    self.publisherGoalPose.publish(final_pose)
                i=min(6,len(self.plan.poses)-id_closest_point-1)
                self.target_state[2]=self.Quat2yaw(self.plan.poses[id_closest_point+i].pose.orientation)

                if d_final <12 :
                    self.target_state=self.goal_target
                    self.mode=0 
                    self.mppis[self.mode].u_prev = np.zeros((self.mppis[self.mode].T, self.mppis[self.mode].control_var))

        elif self.mode ==2 : #follow cicrle for qr code
            self.target_state=self.goal_target
            self.target_state[2]=10.0 #radius of circle
          

        elif self.mode ==3 : #follow cicrle for bonus phase
            self.target_state=self.goal_target
            self.target_state[2]=1.2 # lateral speed 
           
        
        else : #mode 0, go to point
            self.target_state=self.goal_target
    #--------------------------------------------------------------
    #-----------------------utils----------------------------------
    #--------------------------------------------------------------
            
    # Retourner un tableau numpy avec toutes les positions [x, y] du plan, plsu pratique pour le traitement
    def extract_positions_from_path(self) -> np.ndarray:
        positions = []
        
        for pose_stamped in self.plan.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            positions.append([x, y])
        self.plan_array=np.array(positions)
    
    #get the points of the around the current pose of the usv, used to be efficient 
    def get_closest_points(self,USV_position, all_points, threshold):
        # Créer un KD-Tree avec tous les points
        tree = KDTree(all_points)
        
        # Trouver les indices des points proches (dans un rayon donné)
        indices_proches = tree.query_ball_point(USV_position, threshold)
        _, index = tree.query(USV_position)
        
        # Extraire les points proches
        closest_points = all_points[indices_proches]
        
        return closest_points,index

    #create a message pose stamped from a array (x,y,yaw)
    def create_pose_stamped(self,array):
        if len(array) != 3:
            raise ValueError("L'entrée doit être un tableau de longueur 3 : [x, y, theta].")
        
        # Création du message PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'  # Référence du cadre
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp actuel
        
        # Position
        pose_msg.pose.position.x = array[0]
        pose_msg.pose.position.y = array[1]
        pose_msg.pose.position.z = 0.0  # Par défaut, z est à 0
        
        # Orientation : conversion de theta (yaw) en quaternion
        quaternion = quaternion_from_euler(0, 0, array[2])  # Roll = 0, Pitch = 0, Yaw = theta
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        return pose_msg

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

