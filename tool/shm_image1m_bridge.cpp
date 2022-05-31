#include "shm_image_bridge.hpp"

using Topic = typename shm_msgs::msg::Image1m;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<ShmImageBridge<Topic>>("shm_image1m_bridge", options));
  rclcpp::shutdown();

  return 0;
}