#include "navigation.h"
#include "aqua_utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include <vector>

using std::placeholders::_1;

Navigation::Navigation() : Node("navigation") {
  // Souscriptions

  RCLCPP_INFO(this->get_logger(), "\nNode 'navigation' has started!\n");
  subscription_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/aquabot/sensors/gps/gps/fix", 10,
      std::bind(&Navigation::gps_callback, this, _1));

  subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/aquabot/sensors/imu/imu/data", 10,
      std::bind(&Navigation::imu_callback, this, _1));

  subscription_keyboard =
      this->create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10,
          std::bind(&Navigation::keyboard_callback, this,
                    std::placeholders::_1));

  // Publishers
  publisher_motorL = this->create_publisher<std_msgs::msg::Float64>(
      "/aquabot/thrusters/left/thrust", 10);
  publisher_motorR = this->create_publisher<std_msgs::msg::Float64>(
      "/aquabot/thrusters/right/thrust", 10);
}

void Navigation::gps_callback(const sensor_msgs::msg::NavSatFix &msg) const {

}

void Navigation::imu_callback(const sensor_msgs::msg::Imu &msg) const {

}

void Navigation::keyboard_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
  std_msgs::msg::Float64 motorL;
  std_msgs::msg::Float64 motorR;
  motorL.data = msg->linear.x + msg->angular.z;
  motorR.data = msg->linear.x - msg->angular.z;
  publisher_motorL->publish(motorL);
  publisher_motorR->publish(motorR);
  RCLCPP_INFO(this->get_logger(), "key pressed");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigation>());
  rclcpp::shutdown();
  return 0;
}
