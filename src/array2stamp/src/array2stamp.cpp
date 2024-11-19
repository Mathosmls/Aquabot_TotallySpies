#include "array2stamp.h"
#include <GeographicLib/LocalCartesian.hpp>




Array2StampNode::Array2StampNode() : Node("array2stamp_node")
{
    // Créer un abonnement au topic /local_wind_turbine_positions de type PoseArray
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/local_wind_turbine_positions", 10, 
        std::bind(&Array2StampNode::pose_array_callback, this, std::placeholders::_1));

    // Créer un éditeur pour publier des PoseStamped
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);
    
    subscription_gps = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/loc_gps",   
        10,             
        std::bind(&Array2StampNode::gps_callback, this, std::placeholders::_1) 
    );
}

void Array2StampNode::pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    geometry_msgs::msg::Pose closest_pose;
    double min_dist = std::numeric_limits<double>::max();

    // Vérifiez si _x et _y contiennent une valeur
    if (_x.has_value() && _y.has_value()) {
        for (const auto& pose : msg->poses) {
            // Calculer la distance entre le point de référence et les poses
            double dist = std::sqrt(std::pow(pose.position.x - _x.value(), 2) + std::pow(pose.position.y - _y.value(), 2));

            // Trouver la pose la plus proche
            if (dist < min_dist) {
                min_dist = dist;
                closest_pose = pose;
            }
        }

        // Publier la pose la plus proche sous forme de PoseStamped
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->get_clock()->now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose = closest_pose;

        RCLCPP_INFO(this->get_logger(), "Closest pose stamped position: x = %.2f, y = %.2f",
                    pose_stamped.pose.position.x, pose_stamped.pose.position.y);

        // Publier le message PoseStamped
        publisher_->publish(pose_stamped);
    } else {
        RCLCPP_WARN(this->get_logger(), "GPS data not received yet.");
    }
}



 void Array2StampNode::gps_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    _x = msg->pose.position.x;
    _y = msg->pose.position.y;

    RCLCPP_WARN(this->get_logger(), "Robot Pose: x = %.2f, y = %.2f",
                    msg->pose.position.x, msg->pose.position.y);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto array2stamp_node = std::make_shared<Array2StampNode>();

    // Exécuter le nœud ROS
    rclcpp::spin(array2stamp_node);
 

    rclcpp::shutdown();
    return 0;
}