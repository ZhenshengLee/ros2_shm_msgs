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

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
using namespace shm_msgs;

template<typename Topic>
class ShmImageBridge : public rclcpp::Node {
private:
    // using Topic = shm_msgs::msg::Image2m;
public:
  explicit ShmImageBridge(std::string node_name, const rclcpp::NodeOptions &options)
      : Node(node_name, options) {

    // subscription callback to process arriving data
    auto callback = [this](const typename Topic::SharedPtr msg_in) -> void {
      m_last_cvimage = shm_msgs::toCvShare(msg_in);
      // copy
      auto msg_out = cv_bridge::CvImage(m_last_cvimage->header, m_last_cvimage->encoding, m_last_cvimage->image).toImageMsg();
      m_publisher->publish(std::move(*msg_out));
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = this->create_subscription<Topic>("shm_image_input", qos, callback);
    m_publisher = this->create_publisher<sensor_msgs::msg::Image>("sensor_image_output", qos);
  }

private:
  typename rclcpp::Subscription<Topic>::SharedPtr m_subscription;
  typename rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;

  shm_msgs::CvImageConstPtr m_last_cvimage;
};

