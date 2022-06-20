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
  using Topic = shm_msgs::msg::PointCloud2m;

public:
  explicit Listener(const rclcpp::NodeOptions &options)
      : Node("shm_open3d2m_listener", options) {

    // subscription callback to process arriving data
    auto callback = [this](const Topic::SharedPtr msg) -> void {

      RCLCPP_INFO(this->get_logger(), "Received ");
      auto transport_time_ns = (now() - msg->header.stamp).nanoseconds();
      auto timestamp_offset_ns = (rclcpp::Time(msg->header.stamp) - m_last_cloud_ts).nanoseconds();
      auto transport_time_ms = transport_time_ns / 1000000.0F;
      auto timestamp_offset_ms = timestamp_offset_ns / 1000000.0F;
      RCLCPP_INFO(get_logger(), "get-open3d2m-transport-time: %.3f", transport_time_ms);
      if(m_last_cloud_ts.nanoseconds() > 0.0)
      {
        RCLCPP_INFO(get_logger(), "get-open3d2m-timestamp_offset-time: %.3f", timestamp_offset_ms);
      }
      m_last_cloud_ts = msg->header.stamp;

      rosToOpen3d(msg, *last_cloud);

    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = create_subscription<Topic>("shm_open3d_2m", qos, callback);
  }

private:
  rclcpp::Subscription<Topic>::SharedPtr m_subscription;

  std::shared_ptr<open3d::geometry::PointCloud> last_cloud{std::make_shared<open3d::geometry::PointCloud>()};
  rclcpp::Time m_last_cloud_ts{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Listener>(options));
  rclcpp::shutdown();

  return 0;
}