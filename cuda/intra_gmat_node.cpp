#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "intra_gmat_talker.hpp"
#include "intra_gmat_listener.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto talker = std::make_shared<Talker>();
  auto listener = std::make_shared<Listener>();
  executor.add_node(talker);
  executor.add_node(listener);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}