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

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;
using namespace pcl::io;
using namespace pcl;

class Talker : public rclcpp::Node {
private:
  using Topic = sensor_msgs::msg::PointCloud2;

public:
  explicit Talker(const rclcpp::NodeOptions &options)
      : Node("pcl_talker", options) {

    loadPCDFile("./res/pcd/1m.pcd", *m_input_cloud);

    auto publishMessage = [this]() -> void {

      auto msg = std::make_shared<Topic>();
      toROSMsg(*m_input_cloud, *msg);
      msg->header.stamp = now();
      msg->header.frame_id = "pcl";
      RCLCPP_INFO(this->get_logger(), "Publishing ");
      m_publisher->publish(std::move(*msg));

      m_count++;
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_publisher = this->create_publisher<Topic>("/rslidar_points", qos);

    // Use a timer to schedule periodic message publishing.
    m_timer = this->create_wall_timer(0.1s, publishMessage);
  }

private:
  uint64_t m_count = 1;
  rclcpp::Publisher<Topic>::SharedPtr m_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_input_cloud{new pcl::PointCloud<pcl::PointXYZ>};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Talker>(options));
  rclcpp::shutdown();

  return 0;
}