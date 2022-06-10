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
#include "shm_msgs/pcl_conversions.h"

// #include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;
using namespace pcl::io;
using namespace shm_msgs;

class Listener : public rclcpp::Node {
private:
  using Topic = shm_msgs::msg::PointCloud2m;

public:
  explicit Listener(const rclcpp::NodeOptions &options)
      : Node("shm_pcl2m_listener", options) {

    // subscription callback to process arriving data
    auto callback = [this](const Topic::SharedPtr msg) -> void {

      RCLCPP_INFO(this->get_logger(), "Received ");
      auto time_offset_ns = (now() - msg->header.stamp).nanoseconds();
      auto time_offset_ms = time_offset_ns / 1000000.0F;
      RCLCPP_INFO(get_logger(), "get-pcl2m-transport-time: %.3f", time_offset_ms);

      fromROSMsg(*msg, *m_last_cloud);
      // moveFromROSMsg(*msg, *m_last_cloud);

      // will block
    //   m_viewer.showCloud(m_last_cloud);
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = create_subscription<Topic>("shm_pc_2m", qos, callback);
  }

private:
  rclcpp::Subscription<Topic>::SharedPtr m_subscription;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_last_cloud{new pcl::PointCloud<pcl::PointXYZ>};
//   pcl::visualization::CloudViewer m_viewer{"Cloud Viewer"};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Listener>(options));
  rclcpp::shutdown();

  return 0;
}