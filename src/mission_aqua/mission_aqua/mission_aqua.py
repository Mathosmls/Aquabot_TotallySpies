from mission_aqua_utils import( tsp_solver,calculate_offset_target,Quat2yaw,extract_id,add_or_update_turbine,is_within_distance,
                            find_best_matching_wind_turbine_id,update_turbine_qr_position)
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseArray,Pose,Point
from std_msgs.msg import UInt8, String
from tf_transformations import quaternion_from_euler
from ros_gz_interfaces.msg import ParamVec


class MissionAqua(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.get_logger().info('MissionAqua node has started !')

        self.publisherGoalTarget= self.create_publisher(PoseStamped, '/goal_target', 10)
        self.publisherGoalPose= self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.publisherMode= self.create_publisher(UInt8, '/mode', 10)
        self.publisherCurrentWt= self.create_publisher(Point, '/current_wt', 10)

        self.subscription_pos_wt = self.create_subscription(
            PoseArray,'/local_wind_turbine_positions',
            self.pos_wt_callback,10  )
        
        self.subscription_pos4Qr = self.create_subscription(
            PoseStamped,'/pos_gps_align_qr',
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
        self.last_qr_pos=np.array([0.,0.,0])
        self.last_qr_id=-1

#----------------end init------------------

    #callback to process the wt positions
    def pos_wt_callback(self,msg) :
        if not self.have_wt :
            print("pos_wt_callback")
            self.number_wt=len(msg.poses)
            for i in range(len(msg.poses)) :
                    self.pos_wt[i] = np.array([msg.poses[i].position.x,msg.poses[i].position.y])
            print("pos_wt_callback", "pos_wt before tsp : ",self.pos_wt)
            self.pos_wt,_=tsp_solver(self.current_pos[:2],self.pos_wt) #order them in the best order to minimize the total travel distance
            print("pos_wt_callback", "pos_wt after tsp : ",self.pos_wt)
            for i in range(len(self.pos_wt)) : #calculate the actual position to go 
                if i==0 :
                    self.pos_wts_2go[i]=calculate_offset_target(self.current_pos,self.pos_wt[i],radius=13)
                else :
                    self.pos_wts_2go[i]=calculate_offset_target(self.pos_wts_2go[i-1],self.pos_wt[i],radius=13) 
            print("pos_wt_callback", "pos_wt2go after offset : ",self.pos_wts_2go)
            self.have_wt=True


    def odom_callback(self, msg):
        self.current_pos[0]=msg.pose.pose.position.x
        self.current_pos[1]=msg.pose.pose.position.y
        yaw = Quat2yaw(msg.pose.pose.orientation)
        self.current_pos[2]=yaw
        #------------phase 1-----------------
        if not self.start and self.have_wt: #go the fisrt wt and start the mission
            print("odom callback", "go to first wt pos : ", self.pos_wts_2go[self.current_wt])
            self.start=True
            self.current_mode=1
            current_wt_pos=Point()
            current_wt_pos.x=self.pos_wt[self.current_wt][0]
            current_wt_pos.y=self.pos_wt[self.current_wt][1]
            self.publisherCurrentWt.publish(current_wt_pos)
            self.go2wt(self.pos_wts_2go[self.current_wt])

        #if the USV is close from the wt he just explored, go slowly
        if self.current_mode==1 and self.current_wt>0 and is_within_distance(self.current_pos,self.pos_wts_2go[self.current_wt-1],20.0) :
            print("odom callback", "slow down")
            self.current_mode=4
            mode=UInt8()
            mode.data=self.current_mode
            self.publisherMode.publish(mode)
        #if the USV is not close from the wt he just explored, go fast
        if self.current_mode==4 and not is_within_distance(self.current_pos,self.pos_wts_2go[self.current_wt-1],20.0) :
            print("odom callback", "speed up")
            self.current_mode=1
            mode=UInt8()
            mode.data=self.current_mode
            self.publisherMode.publish(mode)
        #if the USV is close to the wt he wants to explore, turn around until finding the qr code
        if self.current_mode==1 and is_within_distance(self.current_pos,self.current_goal,1.0) and self.phase==1:
            # print("odom callback", "normal speed")
            self.current_mode=3
            self.current_goal[0]=self.pos_wt[self.current_wt][0]
            self.current_goal[1]=self.pos_wt[self.current_wt][1]
            goal_target=self.create_pose_stamped(self.current_goal)
            print("odom callback", "do circle around wt pos : ", self.current_goal)
            self.publisherGoalTarget.publish(goal_target)
            mode=UInt8()
            mode.data=self.current_mode
            self.publisherMode.publish(mode)
        #------------phase 2-----------------
        #if the USV is close to the critical target point, start the countdown
        if self.phase ==2 and is_within_distance(self.current_pos,self.current_goal,0.1):
            print("odom callback", "don't move infront of qr : ", self.current_goal)
            if self.start_phase3 is None :
                self.start_phase3=self.get_clock().now()
                current_wt_pos=Point()
                current_wt_pos.x=self.wind_turbines_dic[self.id_wt2fix]["position"][0]
                current_wt_pos.y=self.wind_turbines_dic[self.id_wt2fix]["position"][1]
                self.publisherCurrentWt.publish(current_wt_pos)
            #if the countdown is over, do the turn around the critical wt
            elif (self.get_clock().now()-self.start_phase3)>Duration(seconds=180) :
                print("odom callback", "round trip : ")
                self.phase=3
                self.current_mode=3
                self.current_goal= np.array([self.wind_turbines_dic[self.id_wt2fix]["position"][0],self.wind_turbines_dic[self.id_wt2fix]["position"][1],0])
                goal_target=self.create_pose_stamped(self.current_goal)
                print("odom callback", "round trip : ","goal",self.current_goal)
                self.publisherGoalTarget.publish(goal_target)
                mode=UInt8()
                mode.data=self.current_mode
                self.publisherMode.publish(mode)



    #----------Cam part----------------------------------------------------------
    #callback for handling the qrcode data
    def wt_checkup_callback(self,msg):
        if self.phase <2 :
            print("wt_checkup_callback", "received qr code info ",msg.data)
            id=extract_id(msg.data)
            print("wt_checkup_callback", "id ", id)
            if self.last_qr_id==id :
                pose_qr=self.last_qr_pos
            else :
                pose_qr=np.array([0.,0.,0.])
            self.wind_turbines_dic,already_checked=add_or_update_turbine(id,self.wind_turbines_dic,self.pos_wt[self.current_wt],self.current_wt,pose_qr)
            print("wt_checkup_callback", "new dic ", self.wind_turbines_dic)
            
            if not already_checked :
                #if we didn't checked all the wt, we go to next
                if self.current_wt<self.number_wt-1 :
                    self.current_wt+=1
                    print("wt_checkup_callback", "got to wt nb ", self.current_wt)
                    self.current_mode=1
                    self.go2wt(self.pos_wts_2go[self.current_wt])
                    current_wt_pos=Point()
                    current_wt_pos.x=self.pos_wt[self.current_wt][0]
                    current_wt_pos.y=self.pos_wt[self.current_wt][1]
                    self.publisherCurrentWt.publish(current_wt_pos)
                #else we change current phase
                else :
                    self.current_mode=0
                    self.phase=2
                    print("wt_checkup_callback","wt_dic",self.wind_turbines_dic)
                    print("all wt checked, waiting for the one to inspect...")
            else :
                print("This one has already been checked")

    #callback for the critical target point (in front of Qr, 10m from wt)
    def pos4Qr_callback(self,msg) :
        print("pos4Qr_callback","wt_dic",self.wind_turbines_dic)
        self.last_qr_id =int(msg.pose.position.z)
        self.last_qr_pos=np.array([msg.pose.position.x,msg.pose.position.y,Quat2yaw(msg.pose.orientation)])
        print("pos4Qr_callback","id , pos",self.last_qr_id,self.last_qr_pos)
        self.wind_turbines_dic=update_turbine_qr_position(self.last_qr_id ,self.wind_turbines_dic,pos_qr=self.last_qr_pos)
    #--------------------------

    #callback for the phase2
    def range_bearing_callback(self,msg):
        if self.id_wt2fix==-1 and self.phase==2:
            distance=0.
            bearing=0.
            for param in msg.params:
                if param.name == "range":
                    distance=param.value.double_value
                if param.name=="bearing":
                    bearing=param.value.double_value
            print("range_bearing_callback ","distance and bearing",distance,bearing)
            #find the best wt in function of range and bearing
            self.id_wt2fix=find_best_matching_wind_turbine_id(self.current_pos,self.wind_turbines_dic,distance,bearing)
            print("range_bearing_callback ","id",self.id_wt2fix)
            self.current_mode=1
            self.phase=2
            self.current_wt+=1
            pos_qr2go= self.wind_turbines_dic[self.id_wt2fix]["pos_qr"]
            print("range_bearing_callback", "pos_qr2go" ,pos_qr2go)
            self.go2wt(pos_qr2go)     
        
    #send a new path to go to wt
    def go2wt(self,goal)  :
        self.current_goal=goal
        print("go2wt"," goal : ",self.current_goal)
        goal_pose=self.create_pose_stamped(self.current_goal)
        self.publisherGoalPose.publish(goal_pose)

        mode=UInt8()
        mode.data=self.current_mode
        self.publisherMode.publish(mode)


    #utils to create a pose stamped msg
    def create_pose_stamped(self,array):
        if len(array) != 3:
            raise ValueError("L'entrée doit être un tableau de longueur 3 : [x, y, theta].")
        
        # Création du message PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'  
        pose_msg.header.stamp = self.get_clock().now().to_msg()  
        
        # Position
        pose_msg.pose.position.x = array[0]
        pose_msg.pose.position.y = array[1]
        pose_msg.pose.position.z = 0.0  
        
        # Orientation : conversion de theta (yaw) en quaternion
        quaternion = quaternion_from_euler(0, 0, array[2])  
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        return pose_msg


def main(args=None):
    rclpy.init(args=args)

    mission = MissionAqua()

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