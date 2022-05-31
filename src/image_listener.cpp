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

#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
using namespace shm_msgs;
class Listener : public rclcpp::Node {
private:
  using Topic = shm_msgs::msg::Image2m;

public:
  explicit Listener(const rclcpp::NodeOptions &options)
      : Node("shm_image2m_listener", options) {

    // subscription callback to process arriving data
    auto callback = [this](const Topic::SharedPtr msg) -> void {

      RCLCPP_INFO(this->get_logger(), "Received...");
      last_cvimage = shm_msgs::toCvShare(msg);
      // last_cvimage = shm_msgs::toCvCopy(msg);

      auto time_offset_ns = (now() - last_cvimage->header.stamp).nanoseconds();
      auto time_offset_ms = time_offset_ns / 1000000.0F;
      RCLCPP_INFO(get_logger(), "get-image2m-transport-time: %.3f", time_offset_ms);

      // cv::imshow("im show", last_cvimage->image);
      // cv::waitKey(0);
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = create_subscription<Topic>("shm_image_2m", qos, callback);
  }

private:
  rclcpp::Subscription<Topic>::SharedPtr m_subscription;

  shm_msgs::CvImageConstPtr last_cvimage;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Listener>(options));
  rclcpp::shutdown();

  return 0;
}