#include "shm_image_bridge.hpp"

using Topic = typename shm_msgs::msg::Image8k;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<ShmImageBridge<Topic>>("shm_image8k_bridge", options));
  rclcpp::shutdown();

  return 0;
}