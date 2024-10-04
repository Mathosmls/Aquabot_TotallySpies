#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "aqua_utils.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

class Navigation : public rclcpp::Node {
public:
  Navigation();  // Constructeur

private:
  // Callbacks
  void gps_callback(const sensor_msgs::msg::NavSatFix &msg) const;
  void imu_callback(const sensor_msgs::msg::Imu &msg) const;
  void keyboard_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;

  // Publishers et Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_keyboard;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motorL;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motorR;
};

#endif // NAVIGATION_HPP
