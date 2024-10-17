#ifndef SENSORSPROCESSING_HPP
#define SENSORSPROCESSING_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>


class SensorsProcessing : public rclcpp::Node {
public:
  SensorsProcessing();  // Constructeur

private:
  // Callbacks
  void gps_callback(const sensor_msgs::msg::NavSatFix &msg) const;
  void imu_callback(const sensor_msgs::msg::Imu &msg) const;
  void nedToEnuQuaternion(const tf2::Quaternion ned_quat, tf2::Quaternion &enu_quat) const;

  // Publishers et Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_keyboard;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_gps_cov;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_cov;
};

#endif // SENSORSPROCESSING_HPP
