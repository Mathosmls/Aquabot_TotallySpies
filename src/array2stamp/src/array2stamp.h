#ifndef ARRAY2STAMP_NODE_HPP
#define ARRAY2STAMP_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"


struct PoseComparator {
    bool operator()(const geometry_msgs::msg::Pose& lhs, const geometry_msgs::msg::Pose& rhs) const {
        if (lhs.position.x != rhs.position.x) return lhs.position.x < rhs.position.x;
        if (lhs.position.y != rhs.position.y) return lhs.position.y < rhs.position.y;
        return lhs.position.z < rhs.position.z;
    }
};


class Array2StampNode : public rclcpp::Node
{
public:
    // Constructeur
    Array2StampNode();
    std::optional<double> _x;
    std::optional<double> _y;
    std::map<geometry_msgs::msg::Pose, double, PoseComparator> _dist_poses;
    std::vector<geometry_msgs::msg::Pose> _sorted_poses;


private:
    // Fonction de callback pour le topic PoseArray
    void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void gps_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg); ;

    // Abonnement au topic PoseArray
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_gps;

    // Publication des PoseStamped
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

};

#endif  // ARRAY2STAMP_NODE_HPP
