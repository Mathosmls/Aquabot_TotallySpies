#include "graph_node.h"
#include <fstream>
#include <cmath> // Pour std::sqrt
#include <GeographicLib/LocalCartesian.hpp>
#include <cairo.h>
#include <cstdlib> // Pour std::system
#include <geometry_msgs/msg/pose_stamped.hpp>

// Constante pour la conversion des degrés en radians
constexpr double DEG_TO_RAD = M_PI / 180.0;

// Rayon moyen de la Terre en mètres
constexpr double EARTH_RADIUS = 6378137.0;

// Structure pour représenter une position en coordonnées locales
struct LocalPosition {
    double x;  // Coordonnée X en mètres
    double y; 
    double z; 
};

// Fonction pour transformer les coordonnées GPS en coordonnées locales en mètres
LocalPosition transformToLocal(const double lat, const double lon, const double alt) {
    LocalPosition local_pos;

    // Référence des coordonnées (point de départ)
    double lon_ref = -4.97632;
    double lat_ref = 48.04630;

    GeographicLib::LocalCartesian local_cartesian(lat_ref, lon_ref, alt);
    
    // Conversion latitude/longitude/altitude -> local (x, y, z)
    local_cartesian.Forward(lat, lon, alt, local_pos.x, local_pos.y, local_pos.z);

    return local_pos;
}


double Graph::get_pose_x() const {
    return _pose_x;
}

double Graph::get_pose_y() const {
    return _pose_y;
}

void Graph::set_pose_xy(double x, double y) {
    _pose_x = x;
    _pose_y = y;
} 

const std::vector<geometry_msgs::msg::Pose>& Graph::get_poses() const {
    return _wt_poses;  // Ajout de cette définition pour résoudre l'erreur
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

    local_position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/local_wind_turbine_positions", 10
    );

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/wind_turbine_positions_pointcloud", 10
    );
    
    loc_gps_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/loc_gps", 10);
}

void Graph::publish_local_positions_pointcloud() {
    if (_wt_loc_received) {
        // Créer un message PointCloud2
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "local_frame"; // Nom du repère local
        cloud_msg.height = 1;
        cloud_msg.width = _wt_loc_poses_x.size();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        // Définir les champs pour x, y, et z dans le PointCloud2
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        // Remplir les données du nuage de points
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (size_t i = 0; i < _wt_loc_poses_x.size(); ++i) {
            *iter_x = _wt_loc_poses_x[i];
            *iter_y = _wt_loc_poses_y[i];
            *iter_z = 0.0;  // Si z est nul ou une autre valeur, selon le cas
            ++iter_x; ++iter_y; ++iter_z;
        }

        // Publier le PointCloud2
        pointcloud_publisher_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published wind turbine positions as PointCloud2 with %zu points", _wt_loc_poses_x.size());
    }
}



void Graph::publish_local_positions()
{
    if (_wt_loc_received) {
        geometry_msgs::msg::PoseArray local_positions;
        local_positions.header.stamp = this->now();
        local_positions.header.frame_id = "local_frame"; // Nom du repère local

        // Remplir le PoseArray avec les positions locales
        for (size_t i = 0; i < _wt_loc_poses_x.size(); ++i) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = _wt_loc_poses_x[i];
            pose.position.y = _wt_loc_poses_y[i];
            pose.position.z = 0.0; // Z si nécessaire
            local_positions.poses.push_back(pose);
        }

        // Publier les positions locales
        local_position_publisher_->publish(local_positions);
        RCLCPP_INFO(this->get_logger(), "Published local wind turbine positions with %zu poses", local_positions.poses.size());
    }
}


// Callback pour traiter les messages PoseArray
void Graph::pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
{/*
    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());*/
// Fichier temporaire pour stocker les données

    if (_wt_loc_received==false) {
        auto REF = transformToLocal(_lat_ref + 0.001, _lon_ref + 0.001, 0); 
        RCLCPP_INFO(this->get_logger(), "PoseArray received, ref poses x: %f, y: %f", REF.x * _resolution + _img_size / 2, REF.y* _resolution + _img_size / 2);


        cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, _img_size, _img_size);
        cairo_t *cr = cairo_create(surface);

        cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
        cairo_paint(cr);

        cairo_set_source_rgb(cr, 255.0, 0.0, 0.0);
        for (const auto &pose : msg->poses)
        {
            
            _wt_poses.push_back(pose);  
            double lon = pose.position.y;
            double lat = pose.position.x;
            double alt = pose.position.z;
            auto local_position = transformToLocal(lat, lon, alt);

            double x = local_position.x * _resolution + _img_size / 2;
            double y = local_position.y * _resolution + _img_size / 2;
            std::cout << "res : " << _resolution << std::endl;   
            RCLCPP_WARN(this->get_logger(), "Pose loc turbine: (%f, %f)", local_position.x, local_position.y);
            RCLCPP_WARN(this->get_logger(), "Pose glob turbine: (%f, %f)", lon, lat);

            
            _wt_loc_poses_x.push_back(local_position.x);
            _wt_loc_poses_y.push_back(local_position.y);

            cairo_arc(cr, local_position.x * _resolution + _img_size/2, local_position.y * _resolution + _img_size/2, 5.0, 0.0, 2.0 * M_PI);
            cairo_fill(cr);

        } 
        _wt_loc_received = true;

        cairo_surface_write_to_png(surface, "graph.png");

        // Libérer les ressources de Cairo
        cairo_destroy(cr);
        cairo_surface_destroy(surface);

        // Ouvrir l'image avec un programme externe (xdg-open sur Linux)
        std::system("xdg-open graph.png");

        publish_local_positions();
        publish_local_positions_pointcloud();
    }
    

}

void Graph::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        
    LocalPosition local_position = transformToLocal(msg->latitude, msg->longitude, msg->altitude);
    set_pose_xy(local_position.x, local_position.y);    
    
    geometry_msgs::msg::PoseStamped pose_stamped;

    // Remplir le header (timestamp et frame_id)
    pose_stamped.header.stamp = this->get_clock()->now();;
    pose_stamped.header.frame_id = "map";
    
    // Remplir la pose (position et orientation)
    pose_stamped.pose.position.x = local_position.x;
    pose_stamped.pose.position.y = local_position.y;
    pose_stamped.pose.position.z = 0; // Par défaut à 0 si inutile

    // Orientation en quaternion (par défaut, aucune rotation)
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;
    
    loc_gps_publisher_->publish(pose_stamped);

}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto graph_node = std::make_shared<Graph>();

    // Exécuter le nœud ROS
    rclcpp::spin(graph_node);
 

    rclcpp::shutdown();
    return 0;
}
