#include "graph_node.h"

Graph::Graph() : Node("graph_node")
{
    // Créer une souscription au topic "pose_array"
    subscription_pose_array = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aquabot/ais_sensor/windturbines_positions",   
        10,             
        std::bind(&Graph::pose_array_callback, this, std::placeholders::_1)  // Callback
    );
}

// Callback pour traiter les messages
void Graph::pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());
    for (const auto &pose : msg->poses)
    {
        RCLCPP_INFO(this->get_logger(), "Pose - Position: (%f, %f, %f)", 
                     pose.position.x, pose.position.y, pose.position.z);
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Graph>());
    rclcpp::shutdown();
    return 0;
}