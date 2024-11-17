#include "array2stamp_node.hpp"

Array2StampNode::Array2StampNode() : Node("array2stamp_node")
{
    // Créer un abonnement au topic /local_wind_turbine_positions de type PoseArray
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/local_wind_turbine_positions", 10, 
        std::bind(&Array2StampNode::pose_array_callback, this, std::placeholders::_1));

    // Créer un éditeur pour publier des PoseStamped
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/wind_turbine_pose_stamped", 10);
}

void Array2StampNode::pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // Pour chaque Pose dans le tableau PoseArray, convertir en PoseStamped
    for (const auto & pose : msg->poses)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->get_clock()->now();  // Ajouter un timestamp
        pose_stamped.header.frame_id = "map";  // Définir le frame_id (peut être ajusté)
        pose_stamped.pose = pose;  // Copier la Pose dans PoseStamped

        // Publier la PoseStamped
        publisher_->publish(pose_stamped);
    }
}
