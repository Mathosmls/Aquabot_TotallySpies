#include "nav2aqua.h"


using std::placeholders::_1;

Nav2aqua::Nav2aqua() : Node("nav2aqua") {
  // Souscriptions

  RCLCPP_INFO(this->get_logger(), "\nNode 'nav2aqua' has started!\n");


    subscription_goal =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/goal_pose", 10,
          std::bind(&Nav2aqua::goal_callback, this,
                    std::placeholders::_1));
    subscription_odom_ekf =
      this->create_subscription<nav_msgs::msg::Odometry>(
          "/odometry/filtered/map", 10,
          std::bind(&Nav2aqua::odom_ekf_callback, this,
                    std::placeholders::_1));


    current_goal.pose.position.x = NAN;

        client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        this, "/compute_path_to_pose");

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

}

// These functions are not used anymore but keeped just in case as it's a very easy way to control the USV
// void Nav2aqua::nav_control_callback(const geometry_msgs::msg::Twist &msg) const{
//   std_msgs::msg::Float64 motorL;
//   std_msgs::msg::Float64 motorR;
//   auto cmd_a_vel=  msg.angular.z;
//   int sign_cmd_a_vel =(cmd_a_vel > 0) ? 1 : ((cmd_a_vel < 0) ? -1 : 0);
//   motorL.data = linearVel2Thrust(msg.linear.x) - sign_cmd_a_vel*angularVel2Thrust(msg.angular.z);
//   motorR.data = linearVel2Thrust(msg.linear.x) + sign_cmd_a_vel*angularVel2Thrust(msg.angular.z);
//   publisher_motorL->publish(motorL);
//   publisher_motorR->publish(motorR);
// }

// float Nav2aqua::linearVel2Thrust(float l_vel) const
// {
//     if (current_goal.pose.position.x != NAN)
//     {
//         auto distance_goal = calculateDistance(current_odom,current_goal);
//         l_vel = (distance_goal>3.0)? l_vel : l_vel/3.0;
//         RCLCPP_INFO(this->get_logger(), "\ndistance 2 goal : %f\n",distance_goal);
//     }
//     int sign_l_vel =(l_vel > 0) ? 1 : ((l_vel < 0) ? -1 : 0);
//     l_vel=abs(l_vel);
//     if(l_vel<0.62178)
//     {
//         return sign_l_vel*(0.0361 + 90.8*l_vel + 112*pow(l_vel,2) + 1.64*pow(l_vel,3) -2.09*pow(l_vel,4));
//     }
//     else if (l_vel<2.6066)
//     {
//         return sign_l_vel*(-3.56 + 104*l_vel + 97.2*pow(l_vel,2) + 7.41*pow(l_vel,3) -1.25*pow(l_vel,4));
//     }
//     else if (l_vel<6.28)
//     {
//         return sign_l_vel*(6040 + -6025*l_vel + 2335*pow(l_vel,2) + -343*pow(l_vel,3) + 19.1*pow(l_vel,4));
//     }
//     else
//     {
//         return 0.0;
//     }

    
// }

// float Nav2aqua::angularVel2Thrust(float a_vel) const
// {
//     // if (current_goal.pose.position.x != NAN)
//     // {   auto distance_goal = calculateDistance(current_odom,current_goal);
//     //     auto angle_goal = calculateAngleDifference(current_odom,current_goal);
//     //     a_vel = (distance_goal>0.75)? a_vel : a_vel/2.0;
//     //     RCLCPP_INFO(this->get_logger(), "\n angle 2 goal : %f\n",angle_goal);
//     // }
//     a_vel=abs(a_vel);
//     if(a_vel<0.092997)
//     {
//         return (4.55e-03 + 999*a_vel + 836*pow(a_vel,2) -238*pow(a_vel,3) + 1109*pow(a_vel,4));
//     }
//     else if (a_vel<0.65249)
//     {
//         return (-0.029 + 999*a_vel + 819*pow(a_vel,2) -4.05*pow(a_vel,3) + 1.31*pow(a_vel,4));
//     }
//     else if (a_vel<6.28)
//     {
//         return (-1460 + 4116*a_vel + -1658*pow(a_vel,2) + 2038*pow(a_vel,3) -471*pow(a_vel,4));
//     }
//     else
//     {
//         return 0.0;
//     }
// }

// double Nav2aqua::calculateDistance(const nav_msgs::msg::Odometry& odom,
//                          const geometry_msgs::msg::PoseStamped& goal_pose) const{
//     double dx = goal_pose.pose.position.x - odom.pose.pose.position.x;
//     double dy = goal_pose.pose.position.y - odom.pose.pose.position.y;
//     return std::sqrt(dx * dx + dy * dy);
// }

// double Nav2aqua::calculateAngleDifference(const nav_msgs::msg::Odometry& odom,
//                                 const geometry_msgs::msg::PoseStamped& goal_pose)  const{
//     // Extract quaternions from odometry and goal pose
//     auto &q1 = odom.pose.pose.orientation;
//     auto &q2 = goal_pose.pose.orientation;

//     // Calculate the dot product of the two quaternions
//     double dot_product = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
//     dot_product = std::clamp(dot_product, -1.0, 1.0);  // Ensure within [-1, 1]

//     // Calculate and normalize the angle difference
//     double angle_diff = 2 * std::acos(dot_product);
//     if (angle_diff > M_PI) {
//         angle_diff = 2 * M_PI - angle_diff;
//     }

//     return angle_diff;
// }



void Nav2aqua::goal_callback(const geometry_msgs::msg::PoseStamped &msg)
{
    current_goal = msg;
    current_pose.header = msg.header;
    current_pose.pose = current_odom.pose.pose;

    auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
    goal_msg.start = current_pose;  // Définir la pose de départ
    goal_msg.goal = current_goal;    // Définir la pose objectif

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Chemin calculé par le planner !");
            
            // Publier le chemin calculé
            nav_msgs::msg::Path path;
            path.header.frame_id = "map";  // Assurez-vous que le cadre est approprié
            for (const auto &pose : result.result->path.poses) {
                path.poses.push_back(pose);
            }
            path_publisher_->publish(path);  // Publier le chemin sur le topic
        } else {
            RCLCPP_ERROR(this->get_logger(), "Le planner n'a pas pu générer de chemin.");
        }
    };

    client_->async_send_goal(goal_msg, send_goal_options);
}
void Nav2aqua::odom_ekf_callback(const nav_msgs::msg::Odometry &msg) 
{
    current_odom=msg;
    
}



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2aqua>());
  rclcpp::shutdown();
  return 0;
}
