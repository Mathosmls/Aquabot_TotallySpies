#ifndef NAV2AQUA_HPP
#define NAV2AQUA_HPP


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Pour les conversions entre tf2 et geometry_msgs


class Nav2aqua : public rclcpp::Node {
public:
  Nav2aqua();  // Constructeur

private:
  // Callbacks

    // void nav_control_callback(const geometry_msgs::msg::Twist &msg) const;
    void goal_callback(const geometry_msgs::msg::PoseStamped &msg) ;
    void odom_ekf_callback(const nav_msgs::msg::Odometry &msg);
    void search_qr_callback(const geometry_msgs::msg::PoseStamped &msg);

    // float linearVel2Thrust(float l_vel) const;
    // float angularVel2Thrust(float a_vel) const;
    // double calculateDistance(const nav_msgs::msg::Odometry& odom,
    //                      const geometry_msgs::msg::PoseStamped& goal_pose) const;
    // double calculateAngleDifference(const nav_msgs::msg::Odometry& odom,
    //       const geometry_msgs::msg::PoseStamped& goal_pose)  const;

  // Publishers et Subscribers
    
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_nav_control;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_goal;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_search_qr;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_ekf;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motorL;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motorR;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr client_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr client_compute_path_;

    geometry_msgs::msg::PoseStamped current_goal;
    nav_msgs::msg::Odometry current_odom;
    geometry_msgs::msg::PoseStamped current_pose;

   
};

#endif // NAV2AQUA_HPP
