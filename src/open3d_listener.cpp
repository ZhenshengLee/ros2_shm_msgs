// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstring>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "shm_msgs/msg/point_cloud2.hpp"
#include "shm_msgs/open3d_conversions.hpp"

using namespace std::chrono_literals;
using namespace open3d::io;
using namespace shm_msgs;
class Listener : public rclcpp::Node {
private:
  using Topic = shm_msgs::msg::PointCloud8k;

public:
  explicit Listener(const rclcpp::NodeOptions &options)
      : Node("pc_8k_listener", options) {

    // subscription callback to process arriving data
    auto callback = [this](const Topic::SharedPtr msg) -> void {

      rosToOpen3d(msg, *last_cloud);

      RCLCPP_INFO(this->get_logger(), "Received ");
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = create_subscription<Topic>("shm_pc_8k", qos, callback);
  }

private:
  rclcpp::Subscription<Topic>::SharedPtr m_subscription;

  std::shared_ptr<open3d::geometry::PointCloud> last_cloud{std::make_shared<open3d::geometry::PointCloud>()};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Listener>(options));
  rclcpp::shutdown();

  return 0;
}