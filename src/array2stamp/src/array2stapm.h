#ifndef ARRAY2STAMP_NODE_HPP
#define ARRAY2STAMP_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class Array2StampNode : public rclcpp::Node
{
public:
    // Constructeur
    Array2StampNode();

private:
    // Fonction de callback pour le topic PoseArray
    void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    // Abonnement au topic PoseArray
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    // Publication des PoseStamped
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

#endif  // ARRAY2STAMP_NODE_HPP
