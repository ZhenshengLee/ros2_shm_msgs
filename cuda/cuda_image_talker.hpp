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
#include <cinttypes>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"

#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/opencv.hpp>

#include "cuda_image_container.hpp"

using namespace std::chrono_literals;
using namespace cv_bridge;

class Talker : public rclcpp::Node
{
  private:
    using Topic = rclcpp::TypeAdapter<shm_msgs::CudaImageContainer, sensor_msgs::msg::Image>;
    using DataType = shm_msgs::CudaImageContainer;

  public:
    explicit Talker() : Node("cuda_image_talker", rclcpp::NodeOptions().use_intra_process_comms(true))
    {

        // m_input_cvimage->image = cv::imread("./res/img/205_182.png");
        // m_input_cvimage->image = cv::imread("./res/img/1024_768.jpeg");
        m_input_cvimage->image = cv::imread("./res/img/1920_1080.jpg");
        m_input_cvimage->header.frame_id = "camera_link";
        m_input_cvimage->encoding = "bgr8";
        // cv::imshow("input image", m_input_cvimage->image);
        // cv::waitKey(0);

        auto publishMessage = [this]() -> void {
            // conversion from mat to sensor_msg
            m_input_rosimage = std::make_unique<sensor_msgs::msg::Image>();
            // todo: from cvmat to container
            m_input_cvimage->toImageMsg(*m_input_rosimage);
            // start here
            m_input_rosimage->header.stamp = now();

            // no deep copy from capture to published
            auto msg = std::make_unique<DataType>(std::move(m_input_rosimage));

            // RCLCPP_INFO(this->get_logger(), "Publishing with ts: %u.%u", msg->header().stamp.sec,
            // msg->header().stamp.nanosec);
            RCLCPP_INFO(this->get_logger(), "Publishing with address: 0x%" PRIXPTR "",
                        reinterpret_cast<std::uintptr_t>(msg.get()));

            m_publisher->publish(std::move(msg));
            // We gave up ownership

            m_count++;
        };

        // rclcpp::QoS qos(rclcpp::KeepLast(10));
        // rclcpp::QoS qos(rclcpp::SensorDataQoS());
        rclcpp::QoS custom_qos_profile =
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
                .keep_last(5)
                .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
                .avoid_ros_namespace_conventions(false);

        m_publisher = this->create_publisher<Topic>("cuda_image", custom_qos_profile);

        // Use a timer to schedule periodic message publishing.
        m_timer = this->create_wall_timer(1s, publishMessage);
    }

  private:
    uint64_t m_count = 1;
    rclcpp::Publisher<Topic>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<cv_bridge::CvImage> m_input_cvimage{std::make_shared<cv_bridge::CvImage>()};
    std::unique_ptr<sensor_msgs::msg::Image> m_input_rosimage{std::make_unique<sensor_msgs::msg::Image>()};
};
