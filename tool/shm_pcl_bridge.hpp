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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "shm_msgs/msg/point_cloud2.hpp"
#include "shm_msgs/pcl_conversions.h"

using namespace std::chrono_literals;
using namespace shm_msgs;

template<typename Topic>
class ShmPclBridge : public rclcpp::Node {
private:
    // using Topic = shm_msgs::msg::Image2m;
public:
  explicit ShmPclBridge(std::string node_name, const rclcpp::NodeOptions &options)
      : Node(node_name, options) {

    // subscription callback to process arriving data
    auto callback = [this](const typename Topic::SharedPtr msg_in) -> void {
      // fromROSMsg(*msg_in, *m_last_cloud);
      moveFromROSMsg(*msg_in, *m_last_cloud);
      // copy
      auto msg_out = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*m_last_cloud, *msg_out);
      m_publisher->publish(std::move(*msg_out));
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = this->create_subscription<Topic>("shm_pc_input", qos, callback);
    m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_pc_output", qos);
  }

private:
  typename rclcpp::Subscription<Topic>::SharedPtr m_subscription;
  typename rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher;

  pcl::PointCloud<pcl::PointXYZ>::Ptr m_last_cloud{new pcl::PointCloud<pcl::PointXYZ>};
};

