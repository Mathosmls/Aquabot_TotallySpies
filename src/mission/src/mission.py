from mission_utils import( tsp_solver,calculate_offset_target,Quat2yaw,extract_id,add_or_update_turbine,is_within_distance,
                            find_best_matching_wind_turbine_id)
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseArray,Pose
from std_msgs.msg import UInt8, String
from tf_transformations import quaternion_from_euler
from ros_gz_interfaces.msg import ParamVec


class Mission(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.get_logger().info('Mission node has started !')

        self.publisherGoalTarget= self.create_publisher(PoseStamped, '/goal_target', 10)
        self.publisherGoalPose= self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.publisherMode= self.create_publisher(UInt8, '/mode', 10)

        self.subscription_pos_wt = self.create_subscription(
            PoseArray,'/local_wind_turbine_positions',
            self.pos_wt_callback,10  )
        
        self.subscription_pos4Qr = self.create_subscription(
            PoseArray,'/pos4Qr',
            self.pos4Qr_callback,10  )
        
        self.subscription_odom = self.create_subscription(
            Odometry,'/odometry/filtered/map/processed',
            self.odom_callback,10)
        self.subscription_wt_checkup= self.create_subscription(
            String,'/vrx/windturbinesinspection/windturbine_checkup',
            self.wt_checkup_callback,10)
        self.subscription_range_bearing = self.create_subscription(
            ParamVec, '/aquabot/sensors/acoustics/receiver/range_bearing', 
            self.range_bearing_callback,  10  )
        
        self.wind_turbines_dic = {}
        self.pos_wt=np.zeros((3, 2))
        self.pos_wts_2go=np.zeros((3, 3))
        self.current_pos=np.zeros(3)
        self.current_goal=np.zeros(3)

        self.number_wt=0
        self.current_wt=0
        self.current_mode=0
        self.start=False
        self.phase=1
        self.start_phase3=None
        self.id_wt2fix =-1
        self.have_wt=False

#----------------end init------------------


    def pos_wt_callback(self,msg) :
        if not self.have_wt :
            print("pos_wt_callback")
            self.number_wt=len(msg.poses)
            for i in range(len(msg.poses)) :
                    self.pos_wt[i] = np.array([msg.poses[i].position.x,msg.poses[i].position.y])
            print("pos_wt_callback", "pos_wt before tsp : ",self.pos_wt)
            self.pos_wt,_=tsp_solver(self.current_pos[:2],self.pos_wt)
            print("pos_wt_callback", "pos_wt after tsp : ",self.pos_wt)
            for i in range(len(self.pos_wts_2go)-1) :
                if i==0 :
                    self.pos_wts_2go[i]=calculate_offset_target(self.current_pos,self.pos_wt[i])
                else :
                    self.pos_wts_2go[i]=calculate_offset_target(self.pos_wts_2go[i-1],self.pos_wt[i]) 
            print("pos_wt_callback", "pos_wt2go after offset : ",self.pos_wts_2go)
            self.have_wt=True


    def odom_callback(self, msg):
        # print("odom callback")
        self.current_pos[0]=msg.pose.pose.position.x
        self.current_pos[1]=msg.pose.pose.position.y
        yaw = Quat2yaw(msg.pose.pose.orientation)
        self.current_pos[2]=yaw
        # print("odom callback", "pos : ", self.current_pos)
        if not self.start and self.have_wt:
            print("odom callback", "go to first wt pos : ", self.pos_wts_2go[self.current_wt])
            self.current_mode=1
            self.start=True
            self.go2wt(self.pos_wts_2go[self.current_wt])
        if self.current_mode==1 and is_within_distance(self.current_pos,self.current_goal,1.0) and self.phase==1:
            self.current_mode=2
            print("odom callback", "do circle around wt pos : ", self.current_goal)
            self.current_goal[0]=self.pos_wt[self.current_wt][0]
            self.current_goal[1]=self.pos_wt[self.current_wt][1]
            goal_target=self.create_pose_stamped(self.current_goal)
            self.publisherGoalTarget.publish(goal_target)
            mode=UInt8()
            mode.data=self.current_mode
            self.publisherMode.publish(mode)
        if self.phase ==2 and is_within_distance(self.current_pos,self.current_goal,0.1):
            if self.start_phase3==None :
                self.start_phase3=self.get_clock().now().to_msg()
            if (self.get_clock().now().to_msg()-self.start_phase3)>35 :
                self.phase=3
                self.current_mode=3
                self.current_goal= np.array([self.wind_turbines_dic[self.id_wt2fix]["position"][0],self.wind_turbines_dic[self.id_wt2fix]["position"][1],0])
                goal_target=self.create_pose_stamped(self.current_goal)
                self.publisherGoalTarget.publish(goal_target)
                mode=UInt8()
                mode.data=self.current_mode
                self.publisherMode.publish(mode)



    #----------Cam part----------
    def wt_checkup_callback(self,msg):
        self.current_mode=1
        id=extract_id(msg.data)
        self.wind_turbines_dic=add_or_update_turbine(id,self.wind_turbines_dic,self.pos_wt[self.current_wt],self.current_wt)
        if self.current_wt<self.number_wt :
            self.current_wt+=1
            self.go2wt(self.pos_wts_2go[self.current_wt])
        else :
            self.current_mode=0
            print("all wt checked, waiting for the one to inspect...")

    def pos4Qr_callback(self,msg) :
        id=msg.position.z
        pos_qr_array=np.array([msg.position.x,msg.position.y,Quat2yaw(msg.orientation)])
        self.wind_turbines_dic=add_or_update_turbine(id,self.wind_turbines_dic,pos_qr=pos_qr_array)
    #--------------------------


    def range_bearing_callback(self,msg):
        # if self.id_wt2fix==-1 :
        #     distance=msg.params[2].value.double_value
        #     self.id_wt2fix=find_best_matching_wind_turbine_id(self.current_pos,self.wind_turbines_dic,distance)
        #     self.current_mode=1
        #     self.phase=2
        #     pos_qr2go= self.wind_turbines_dic[self.id_wt2fix]["pos_qr"]
        #     self.go2wt(pos_qr2go)     
        None

    def go2wt(self,goal)  :
        self.current_goal=goal

        goal_pose=self.create_pose_stamped(self.current_goal)
        self.publisherGoalPose.publish(goal_pose)

        mode=UInt8()
        mode.data=self.current_mode
        self.publisherMode.publish(mode)



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

    mission = Mission()

    rclpy.spin(mission)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
     


# Ce type de message string fonctionne "{data: '{\"id\":1,\"state\":\"OK\"}'}"
# Tu peux le tester avec la commande suivante : ros2 topic pub --once /vrx/windturbinesinspection/windturbine_checkup std_msgs/String "{data: '{\"id\":1,\"state\":\"OK\"}'}"