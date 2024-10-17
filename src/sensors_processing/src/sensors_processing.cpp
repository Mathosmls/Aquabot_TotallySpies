#include "sensors_processing.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

using std::placeholders::_1;

SensorsProcessing::SensorsProcessing() : Node("sensors_processing") {

    RCLCPP_INFO(this->get_logger(), "\nNode 'sensors_processing' has started!\n");

  // Souscriptions
  subscription_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/aquabot/sensors/gps/gps/fix", 10,
      std::bind(&SensorsProcessing::gps_callback, this, _1));

  subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/aquabot/sensors/imu/imu/data", 10,
      std::bind(&SensorsProcessing::imu_callback, this, _1));

  // Publishers
  publisher_gps_cov = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/aquabot/sensors/gps/gps/fix/processed", 10);
  publisher_imu_cov = this->create_publisher<sensor_msgs::msg::Imu>(
      "/aquabot/sensors/imu/imu/data/processed", 10);
}

void SensorsProcessing::gps_callback(const sensor_msgs::msg::NavSatFix &msg) const {
  sensor_msgs::msg::NavSatFix msg_gps_processed = msg;
  msg_gps_processed.header.stamp = this->now();
  msg_gps_processed.position_covariance_type=2;
  msg_gps_processed.position_covariance= {pow(0.85,2), 0.0, 0.0, 0.0, pow(0.85,2), 0.0, 0.0, 0.0, pow(2.0,2)};
  publisher_gps_cov->publish(msg_gps_processed);
}

void SensorsProcessing::imu_callback(const sensor_msgs::msg::Imu &msg) const {
    sensor_msgs::msg::Imu msg_imu_processed = msg;
    msg_imu_processed.header.stamp = this->now();
    msg_imu_processed.orientation_covariance = {0.8*0.8, 0.0, 0.0
                                              , 0.0, 0.8*0.8, 0.0
                                              , 0.0, 0.0, 0.8*0.8};
    msg_imu_processed.angular_velocity_covariance = {0.08*0.08, 0.0, 0.0
                                                    , 0.0, 0.08*0.08, 0.0
                                                    , 0.0, 0.0, 0.08*0.08};
    msg_imu_processed.linear_acceleration_covariance = {0.275*9.81*0.275, 0.0, 0.0
                                                      , 0.0, 0.275*9.81*0.275, 0.0
                                                      , 0.0, 0.0, 0.275*9.81*0.275};
    tf2::Quaternion q(msg_imu_processed.orientation.x,msg_imu_processed.orientation.y,msg_imu_processed.orientation.z,msg_imu_processed.orientation.w);
    q.normalize();
    // tf2::Quaternion q_ENU;
    // nedToEnuQuaternion(q,q_ENU);
    // q_ENU.normalize();
    msg_imu_processed.orientation.w=q.getW();
    msg_imu_processed.orientation.x=q.getX();
    msg_imu_processed.orientation.y=q.getY();
    msg_imu_processed.orientation.z=q.getZ();
    publisher_imu_cov->publish(msg_imu_processed);
}

void SensorsProcessing::nedToEnuQuaternion(const tf2::Quaternion ned_quat, tf2::Quaternion &enu_quat) const
  {
    // Transformation matrix from NED to ENU frame
    tf2::Matrix3x3 ned_to_enu(0, 1, 0,
                              1, 0, 0,
                              0, 0, -1);

    // Convert quaternion to rotation matrix
    tf2::Matrix3x3 ned_rot_matrix(ned_quat);

    // Transform rotation matrix to ENU frame
    tf2::Matrix3x3 enu_rot_matrix = ned_to_enu * ned_rot_matrix;

    // Convert rotation matrix back to quaternion
    enu_rot_matrix.getRotation(enu_quat);
  }



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorsProcessing>());
  rclcpp::shutdown();
  return 0;
}
