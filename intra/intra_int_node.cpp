#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

template<>
struct rclcpp::TypeAdapter<int32_t, std_msgs::msg::Int32>
{
  using is_specialized = std::true_type;
  using custom_type = int32_t;
  using ros_message_type = std_msgs::msg::Int32;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source;
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = source.data;
  }
};

// Node that produces messages.
struct Producer : public rclcpp::Node
{
using MyAdaptedType = rclcpp::TypeAdapter<int32_t, std_msgs::msg::Int32>;

  Producer(const std::string & name, const std::string & output)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<MyAdaptedType>(output, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub]() -> void {
        // get the shared_ptr of weakptr to ensure concurrency safety
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        static int32_t count = 0;
        auto msg = std::make_unique<int32_t>();
        *(msg) = count++;
        printf(
          "Published message with value: %d, and address: 0x%" PRIXPTR "\n", *(msg),
          reinterpret_cast<std::uintptr_t>(msg.get()));
        pub_ptr->publish(std::move(msg));
      };
    timer_ = this->create_wall_timer(1s, callback);
  }

  rclcpp::Publisher<MyAdaptedType>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Node that consumes messages.
struct Consumer : public rclcpp::Node
{
using MyAdaptedType = rclcpp::TypeAdapter<int32_t, std_msgs::msg::Int32>;

  Consumer(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<MyAdaptedType>(
      input,
      10,
      // [](std::shared_ptr<int32_t> msg) {
      [](std::unique_ptr<int32_t> msg) {
        printf(
          " Received message with value: %d, and address: 0x%" PRIXPTR "\n", *(msg),
          reinterpret_cast<std::uintptr_t>(msg.get()));
      });
  }

  rclcpp::Subscription<MyAdaptedType>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto producer = std::make_shared<Producer>("producer", "number");
  auto consumer = std::make_shared<Consumer>("consumer", "number");
  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}