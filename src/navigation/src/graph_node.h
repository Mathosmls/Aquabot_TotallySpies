#ifndef GRAPH_NODE_H
#define GRAPH_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include <vector>

class Graph : public rclcpp::Node
{
public:
    Graph();  // Constructeur
    const std::vector<geometry_msgs::msg::Pose>& get_poses() const; // Modifié ici
    double get_pose_x() const;
    double get_pose_y() const;
    void set_pose_xy(double x, double y);

private:
    // Attributs
    std::vector<geometry_msgs::msg::Pose> _wt_poses;
    std::vector<double> _wt_loc_poses_x;
    std::vector<double> _wt_loc_poses_y;
    double _pose_x;
    double _pose_y;

    // Callbacks
    void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg); // Supprimer const ici
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) ;

    // Publishers et Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_pose_array;
};

#endif