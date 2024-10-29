#ifndef _MTR_CONTROL_H
#define _MTR_CONTROL_H


#include "aqua_utils.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

class MtrControl : public rclcpp::Node {
    public:
    MtrControl();  // Constructeur

    private:    
    // Callbacks
    void gps_callback(const sensor_msgs::msg::NavSatFix &msg) const;
    void imu_callback(const sensor_msgs::msg::Imu &msg) const;

    void motors_control();


    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr r_motor_angle;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr l_motor_angle;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr r_motor_speed;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr l_motor_speed;
    

};
#endif
