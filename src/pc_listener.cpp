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

#include "ros2_shm_demo/msg/shm_topic.hpp"

class Listener : public rclcpp::Node {
private:
  using Topic = ros2_shm_demo::msg::ShmTopic;

public:
  explicit Listener(const rclcpp::NodeOptions &options)
      : Node("shm_demo_listener", options) {

    // subscription callback to process arriving data
    auto callback = [this](const Topic::SharedPtr msg) -> void {
      // Read the message and perform operations accordingly.
      // Here we copy the data and display it.

      std::memcpy(m_lastData, msg->data.data(), msg->size);
      m_lastData[Topic::MAX_SIZE] =
          '\0'; // in case there was no zero termination

      RCLCPP_INFO(this->get_logger(), "Received %s %lu", m_lastData,
                  msg->counter);
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = create_subscription<Topic>("chatter", qos, callback);
  }

private:
  rclcpp::Subscription<Topic>::SharedPtr m_subscription;

  char m_lastData[256];
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Listener>(options));
  rclcpp::shutdown();

  return 0;
}