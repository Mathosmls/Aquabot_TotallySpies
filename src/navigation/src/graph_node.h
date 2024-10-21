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

private:
    // Callbacks
    void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg); // Supprimer const ici
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const;

    std::vector<geometry_msgs::msg::Pose> _wt_poses;
    // Publishers et Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_pose_array;
};
