#include "mtr_control.h"
#include <fstream>
#include <cmath> // Pour std::sqrt
#include <GeographicLib/LocalCartesian.hpp>
#include <cstdlib> // Pour std::system


double sawtooth(double x, double T) {
    return (x - std::floor(x / T) * T) / T;
}

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
        std::bind(&MtrControl::gps_callback, this, std::placeholders::_1) 
    );

    subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        "/aquabot/sensors/imu/imu/data",   
        10,             
        std::bind(&MtrControl::imu_callback, this, std::placeholders::_1) 
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

void MtrControl::imu_callback(const sensor_msgs::msg::Imu &msg){

    double qx = msg.orientation.x;
    double qy = msg.orientation.y;
    double qz = msg.orientation.z;
    double qw = msg.orientation.w;

    /*double ax = msg.linear_acceleration.x;
    double ay = msg.linear_acceleration.y;
    double az = msg.linear_acceleration.z;

    double phi = std::atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
    double theta = std::asin(2*(qw*qy - qz*qx));*/
    double psi = std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));

    float speed = 1000;
    float kp = 50;

    double psi_obj = M_PI/6; 
    double eps = sawtooth(psi_obj - psi, 2*M_PI); 
    
    std_msgs::msg::Float64 r_speed_msg;
    std_msgs::msg::Float64 l_speed_msg;  

    r_speed_msg.data = speed + eps * kp;
    l_speed_msg.data = speed - eps * kp; 

    motors_control(r_speed_msg, l_speed_msg);
     
}

void MtrControl::motors_control(std_msgs::msg::Float64 right, std_msgs::msg::Float64 left) {
    r_motor_speed->publish(right);
    l_motor_speed->publish(left);
}

void MtrControl::gps_callback(const sensor_msgs::msg::NavSatFix &msg){
    double latitude = msg.latitude;
    double longitude = msg.longitude;
    double altitude = msg.altitude;

    // You can add any logic you want to implement with the GPS data here
    std::cout << "GPS Data - Latitude: " << latitude 
              << ", Longitude: " << longitude 
              << ", Altitude: " << altitude << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto mtr_node = std::make_shared<MtrControl>();

    // Exécuter le nœud ROS
    rclcpp::spin(mtr_node);
 

    rclcpp::shutdown();

    return 0;
}