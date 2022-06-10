#include "shm_pcl_bridge.hpp"

using Topic = typename shm_msgs::msg::PointCloud8k;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<ShmPclBridge<Topic>>("shm_pcl8k_bridge", options));
  rclcpp::shutdown();

  return 0;
}