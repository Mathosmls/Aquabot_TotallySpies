#include "graph_node.h"
#include <fstream>
#include <cmath> // Pour std::sqrt
#include <GeographicLib/LocalCartesian.hpp>
#include <cairo.h>
#include <cstdlib> // Pour std::system

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


void Graph::drw_img() {
    // Créer une image au format PNG
    cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, _img_size, _img_size);
    cairo_t *cr = cairo_create(surface);

    // Dessiner un fond blanc
    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    cairo_paint(cr);

    // Dessiner les positions des éoliennes
    cairo_set_source_rgb(cr, 0.0, 0.0, 0.0);
    for (size_t i = 0; i < _wt_loc_poses_x.size(); i++) {
        double x = _wt_loc_poses_x[i] * _resolution + _img_size / 2;
        std::cout << "x: " << _wt_loc_poses_x[i] << std::endl;
        double y = _wt_loc_poses_y[i] * _resolution + _img_size / 2;
        std::cout << "y: " << _wt_loc_poses_y[i] << std::endl;
        cairo_arc(cr, x, y, 5.0, 0.0, 2.0 * M_PI);
        cairo_fill(cr);
    }

    // Sauvegarder l'image au format PNG
    cairo_surface_write_to_png(surface, "graph.png");

    // Libérer les ressources de Cairo
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    // Ouvrir l'image avec un programme externe (xdg-open sur Linux)
    std::system("xdg-open graph.png");
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
void Graph::pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
{/*
    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());*/
// Fichier temporaire pour stocker les données

    if (_wt_loc_received==false) {
        auto REF = transformToLocal(_lat_ref, _lon_ref, 0); 
        RCLCPP_INFO(this->get_logger(), "PoseArray received, ref poses x: %f, y: %f", REF.x, REF.y);


        cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, _img_size, _img_size);
        cairo_t *cr = cairo_create(surface);

        cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
        cairo_paint(cr);

        cairo_set_source_rgb(cr, 255.0, 0.0, 0.0);
        for (const auto &pose : msg->poses)
        {
            
            _wt_poses.push_back(pose);  
            double lon = pose.position.x;
            double lat = pose.position.y;
            double alt = pose.position.z;
            auto local_position = transformToLocal(lat, lon, alt);

            double x = local_position.x * _resolution + _img_size / 2;
            double y = local_position.y * _resolution + _img_size / 2;
            std::cout << "res : " << _resolution << std::endl;   
            RCLCPP_WARN(this->get_logger(), "Pose loc turbine: (%f, %f)", x, y);
            
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
    }
    

}

void Graph::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!get_poses().empty()) {

        
        LocalPosition local_position = transformToLocal(msg->latitude, msg->longitude, msg->altitude);
        set_pose_xy(local_position.x, local_position.y);    ;

        //RCLCPP_INFO(this->get_logger(), "Position loc Robot (%f, %f)", local_position.x, local_position.y);

    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto graph_node = std::make_shared<Graph>();

    // Exécuter le nœud ROS
    rclcpp::spin(graph_node);
 

    rclcpp::shutdown();
    return 0;
}
