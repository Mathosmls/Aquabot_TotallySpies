#ifndef GRAPH_NODE_H
#define GRAPH_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Graph : public rclcpp::Node
{
public:
    Graph();  // Constructeur
    const std::vector<geometry_msgs::msg::Pose>& get_poses() const; // Modifié ici
    double get_pose_x() const; //getter pose x robot
    double get_pose_y() const; //getter pose y robot
    void set_pose_xy(double x, double y); //setter pose robot

    void drw_img(); // Dessiner l'image sur Cairo

    // lat, lon ref
    double _lon_ref = -4.97632;
    double _lat_ref = 48.04630;

private:
    // Attributs

    // widturbine local position
    std::vector<geometry_msgs::msg::Pose> _wt_poses;
    std::vector<double> _wt_loc_poses_x;
    std::vector<double> _wt_loc_poses_y;

    double _pose_x;
    double _pose_y;
    bool _wt_loc_received = false; // verify if widturbine local position received
    
    int _img_size = 820;
    float _l_map = 600;
    double _resolution = _img_size / _l_map;  


    // Callbacks
    void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg); // Supprimer const ici
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) ;
    void publish_local_positions();
    void publish_local_positions_pointcloud();

    // Publishers et Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_pose_array;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr local_position_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr loc_gps_publisher_;
};

#endif
