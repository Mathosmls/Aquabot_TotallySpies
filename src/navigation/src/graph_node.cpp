#include "graph_node.h"
#include <fstream>
#include <cmath> // Pour std::sqrt

// Constante pour la conversion des degrés en radians
constexpr double DEG_TO_RAD = M_PI / 180.0;

// Rayon moyen de la Terre en mètres
constexpr double EARTH_RADIUS = 6378137.0;

// Structure pour représenter une position en coordonnées locales
struct LocalPosition {
    double x;  // Coordonnée X en mètres
    double y;  // Coordonnée Y en mètres
};

// Fonction pour transformer les coordonnées GPS en coordonnées locales en mètres
LocalPosition transformToLocal(const double lat, const double lon) {
    LocalPosition local_pos;

    // Référence pour les coordonnées locales (point d'origine)
    double lon_ref = -4.97632;
    double lat_ref = 48.04630;

    // Calcul de la différence en latitude et longitude en radians
    double d_lon = (lon - lon_ref) * DEG_TO_RAD;
    double d_lat = (lat - lat_ref) * DEG_TO_RAD;

    // Conversion des différences en mètres
    local_pos.x = EARTH_RADIUS * d_lon * std::cos(lat * DEG_TO_RAD);
    local_pos.y = EARTH_RADIUS * d_lat;

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
}

// Callback pour traiter les messages PoseArray
void Graph::pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
{
    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());

    std::ofstream file("poses_data.txt"); // Fichier temporaire pour stocker les données

    for (const auto &pose : msg->poses)
    {
        
        _wt_poses.push_back(pose);  
        double lon = pose.position.x;
        double lat = pose.position.y;
        auto local_position = transformToLocal(lat, lon);
        RCLCPP_WARN(this->get_logger(), "Pose loc turbine: (%f, %f)", local_position.x, local_position.y);
        
        _wt_loc_poses_x.push_back(local_position.x);
        _wt_loc_poses_y.push_back(local_position.y);

        // Écrire les positions dans le fichier
        file << local_position.x << " " << local_position.y << std::endl;
    }

    file.close();

    // Lancer Gnuplot pour visualiser les données avec une pause pour garder la fenêtre ouverte
    system("gnuplot -e \"plot 'poses_data.txt' using 1:2 with points; pause -1\"");
}

void Graph::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!get_poses().empty()) {

        
        LocalPosition local_position = transformToLocal(msg->latitude, msg->longitude);
        set_pose_xy(local_position.x, local_position.y);    ;

        RCLCPP_INFO(this->get_logger(), "Position loc Robot (%f, %f)", local_position.x, local_position.y);

        // Ajouter la position du robot au fichier
        std::ofstream file("robot_pose.txt");
        file << local_position.x << " " << local_position.y << std::endl;
        file.close();

        // Lancer Gnuplot avec une pause pour visualiser la position du robot et des turbines
        system("gnuplot -e \"plot 'poses_data.txt' using 1:2 with points, 'robot_pose.txt' using 1:2 with points pointtype 7; pause -1\"");
    } else {
        RCLCPP_WARN(this->get_logger(), "No poses available.");
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
