#ifndef SENSORSPROCESSING_HPP
#define SENSORSPROCESSING_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <deque>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>



class SensorsProcessing : public rclcpp::Node {
public:
  SensorsProcessing();  // Constructeur

private:
  // Callbacks
  void gps_callback(const sensor_msgs::msg::NavSatFix &msg) const;
  void imu_callback(const sensor_msgs::msg::Imu &msg) const;
  void nedToEnuQuaternion(const tf2::Quaternion ned_quat, tf2::Quaternion &enu_quat) const;
  void odom_ekf_callback(const nav_msgs::msg::Odometry &msg);

  // Publishers et Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_ekf;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_keyboard;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_gps_cov;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_cov;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_ekf_processed;


  int window_size_ = 7;  // Nombre de mesures pour la moyenne glissante
  std::deque<geometry_msgs::msg::Point> position_history_;
  std::deque<geometry_msgs::msg::Vector3> velocity_history_;

};

#endif // SENSORSPROCESSING_HPP
