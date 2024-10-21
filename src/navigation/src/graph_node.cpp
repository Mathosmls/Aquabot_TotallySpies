#include "graph_node.h"
#include <cmath> // Pour std::sqrt

// Structure pour représenter une position en coordonnées locales
struct LocalPosition {
    double x;  // Coordonnée X
    double y;  // Coordonnée Y
};

// Fonction pour transformer les coordonnées GPS en coordonnées locales
LocalPosition transformToLocal(const sensor_msgs::msg::NavSatFix::SharedPtr& gps_msg, 
                                const geometry_msgs::msg::Pose& reference_pose) {
    LocalPosition local_pos;
    // Transformation simple en utilisant les différences
    local_pos.x = gps_msg->longitude - reference_pose.position.x; // Différence en Longitude
    local_pos.y = gps_msg->latitude - reference_pose.position.y;  // Différence en Latitude
    return local_pos;
}

Graph::Graph() : Node("graph_node")
{
    // Créer une souscription au topic "pose_array"
    subscription_pose_array = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/aquabot/ais_sensor/windturbines_positions",   
        10,             
        std::bind(&Graph::pose_array_callback, this, std::placeholders::_1) 
    );

    // Créer une souscription au topic GPS
    subscription_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/aquabot/sensors/gps/gps/fix",   
        10,             
        std::bind(&Graph::gps_callback, this, std::placeholders::_1) 
    );
}

// Callback pour traiter les messages PoseArray
void Graph::pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) // Supprimer const ici
{
    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());
    for (const auto &pose : msg->poses)
    {
        RCLCPP_INFO(this->get_logger(), "Pose - Position: (%f, %f, %f)", 
                     pose.position.x, pose.position.y, pose.position.z);
        
        _wt_poses.push_back(pose);  // Maintenant, cela fonctionnera
    }
}



// Callback pour traiter les messages GPS
void Graph::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const {
    double latitude = msg->latitude;    
    double longitude = msg->longitude;  
    double altitude = msg->altitude;    
    RCLCPP_INFO(this->get_logger(), "Position Robot (%f, %f, %f)", 
                 latitude, longitude, altitude);   

    if (!get_poses().empty()) {
        RCLCPP_INFO(this->get_logger(), "Position turbine 1 (%f, %f, %f)", 
                    get_poses()[0].position.x, get_poses()[0].position.y, get_poses()[0].position.z);
        
        // Transformation en coordonnées locales
        geometry_msgs::msg::Pose reference_pose = get_poses()[0]; // Utiliser la première pose comme référence
        LocalPosition local_position = transformToLocal(msg, reference_pose);
        
        // Afficher les coordonnées locales
        RCLCPP_INFO(this->get_logger(), "Position locale : X = %f, Y = %f", 
                     local_position.x, local_position.y);
    } else {
        RCLCPP_WARN(this->get_logger(), "No poses available.");
    }
}

const std::vector<geometry_msgs::msg::Pose>& Graph::get_poses() const
{
    return _wt_poses;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Graph>());
    rclcpp::shutdown();
    return 0;
}
