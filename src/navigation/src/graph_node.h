#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include <vector>


class Graph : public rclcpp::Node
{   public:
  Graph();  // Constructeur

private:
  // Callbacks
  void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const;
  
  // Publishers et Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_pose_array;
  };
