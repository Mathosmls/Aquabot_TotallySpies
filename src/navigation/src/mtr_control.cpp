#include "mtr_control.h"
#include <fstream>
#include <cmath> // Pour std::sqrt
#include <GeographicLib/LocalCartesian.hpp>
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

MtrControl::MtrControl() : Node("mtr_control")
{
    subscription_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/aquabot/sensors/gps/gps/fix",   
        10,             
        std::bind(&Graph::gps_callback, this, std::placeholders::_1) 
    );

    subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        "/aquabot/sensors/imu/imu/data",   
        10,             
        std::bind(&Graph::imu_callback, this, std::placeholders::_1) 
    );

    r_motor_angle = this->create_publisher<std_msgs::msg::Float64>(
        "/aquabot/thrusters/right/pos", 10
    );

    l_motor_angle = this->create_publisher<std_msgs::msg::Float64>(
        "/aquabot/thrusters/left/pos", 10
    );

    r_motor_speed = this->create_publisher<std_msgs::msg::Float64>(
        "/aquabot/thrusters/right/thrust", 10
    );

    l_motor_speed = this->create_publisher<std_msgs::msg::Float64>(
        "/aquabot/thrusters/left/thrust", 10
    );
}

