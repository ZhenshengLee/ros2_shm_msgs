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
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"

#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

#include "gmat_image_container.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
using namespace cv_bridge;

class Listener : public rclcpp::Node {
private:
  using Topic = rclcpp::TypeAdapter<shm_msgs::ROSGpuMatContainer, sensor_msgs::msg::Image>;
  using DataType = shm_msgs::ROSGpuMatContainer;

public:
  explicit Listener()
      : Node("gmat_image_listener", rclcpp::NodeOptions().use_intra_process_comms(true)) {

    // subscription callback to process arriving data
    auto callback = [this](std::unique_ptr<DataType> msg) -> void {

      RCLCPP_INFO(this->get_logger(), "received with address: 0x%" PRIXPTR "", reinterpret_cast<std::uintptr_t>(msg.get()));

      // no conversion
      // last_cvimage = cv_bridge::toCvShare(msg);
      // last_cvimage = cv_bridge::toCvCopy(msg);

      // move to the private ptr
      m_last_container = std::move(msg);
      RCLCPP_INFO(this->get_logger(), "container address: 0x%" PRIXPTR "", reinterpret_cast<std::uintptr_t>(m_last_container.get()));

      // directly process gpumat, no copy from gpu to cpu
      m_last_container->cv_gpu_mat();

      auto time_offset_ns = (now() - m_last_container->header().stamp).nanoseconds();
      auto timestamp_offset_ns = (rclcpp::Time(m_last_container->header().stamp) - m_last_image_ts).nanoseconds();
      auto time_offset_ms = time_offset_ns / 1000000.0F;
      auto timestamp_offset_ms = timestamp_offset_ns / 1000000.0F;
      RCLCPP_INFO(get_logger(), "get-image-transport-time: %.3f", time_offset_ms);
      if(m_last_image_ts.nanoseconds() > 0.0)
      {
        RCLCPP_INFO(get_logger(), "get-image-timestamp_offset-time: %.3f", timestamp_offset_ms);
      }
      m_last_image_ts = m_last_container->header().stamp;
      // cv::imshow("im show", last_cvimage->image);
      // cv::waitKey(0);
    };

    // rclcpp::QoS qos(rclcpp::KeepLast(10));
    rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(5)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      .avoid_ros_namespace_conventions(false);

    m_subscription = create_subscription<Topic>("gmat_image", custom_qos_profile, callback);
  }

private:
  rclcpp::Subscription<Topic>::SharedPtr m_subscription;

  std::unique_ptr<DataType> m_last_container;
  rclcpp::Time m_last_image_ts{0, 0, RCL_ROS_TIME};
};
