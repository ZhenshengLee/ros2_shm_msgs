#include "shm_pcl_bridge.hpp"

using Topic = typename shm_msgs::msg::PointCloud1m;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<ShmPclBridge<Topic>>("shm_pcl1m_bridge", options));
  rclcpp::shutdown();

  return 0;
}