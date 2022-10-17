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
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>

using namespace std::chrono_literals;
using namespace cv_bridge;

class Talker : public rclcpp::Node {
private:
  using Topic = sensor_msgs::msg::Image;

public:
  explicit Talker()
      : Node("intra_gmat_talker", rclcpp::NodeOptions().use_intra_process_comms(true)) {

    // m_input_cvimage->image = cv::imread("./res/img/205_182.png");
    // m_input_cvimage->image = cv::imread("./res/img/1024_768.jpeg");
    m_input_cvimage->image = cv::imread("./res/img/1920_1080.jpg");
    m_input_cvimage->header.frame_id = "camera_link";
    m_input_cvimage->encoding = "bgr8";
    // cv::imshow("input image", m_input_cvimage->image);
    // cv::waitKey(0);
    // copy from cpu to gpu and processing
    m_input_gmatimage->upload(m_input_cvimage->image);

    auto publishMessage = [this]() -> void {
      m_input_cvimage->header.stamp = now();

      // copy from gpu to cpu
      m_input_gmatimage->download(m_input_cvimage->image);

      auto msg = std::make_unique<Topic>();
      // a copy from capture to published
      m_input_cvimage->toImageMsg(*msg);

      // RCLCPP_INFO(this->get_logger(), "Publishing with ts: %u.%u", m_input_cvimage->header.stamp.sec, m_input_cvimage->header.stamp.nanosec);
      RCLCPP_INFO(this->get_logger(), "Publishing with address: 0x%" PRIXPTR "", reinterpret_cast<std::uintptr_t>(msg.get()));

      m_publisher->publish(std::move(msg));
      // We gave up ownership

      m_count++;
    };

    // rclcpp::QoS qos(rclcpp::KeepLast(10));
    // rclcpp::QoS qos(rclcpp::SensorDataQoS());
    rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(5)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      .avoid_ros_namespace_conventions(false);

    m_publisher = this->create_publisher<Topic>("intra_gmat", custom_qos_profile);

    // Use a timer to schedule periodic message publishing.
    m_timer = this->create_wall_timer(1s, publishMessage);
  }

private:
  uint64_t m_count = 1;
  rclcpp::Publisher<Topic>::SharedPtr m_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;
  std::shared_ptr<cv_bridge::CvImage> m_input_cvimage{std::make_shared<cv_bridge::CvImage>()};
  std::shared_ptr<cv::cuda::GpuMat> m_input_gmatimage{std::make_shared<cv::cuda::GpuMat>()};
  std::unique_ptr<sensor_msgs::msg::Image> m_input_rosimage{std::make_unique<sensor_msgs::msg::Image>()};
};
