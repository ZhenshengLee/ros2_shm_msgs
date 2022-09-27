#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

using Topic = sensor_msgs::msg::Image;

rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Time g_last_frame_ts{0, 0, RCL_ROS_TIME};

void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg0,
                                       const sensor_msgs::msg::Image::ConstSharedPtr & msg1,
                                       const sensor_msgs::msg::Image::ConstSharedPtr & msg2,
                                       const sensor_msgs::msg::Image::ConstSharedPtr & msg3,
                                       const sensor_msgs::msg::Image::ConstSharedPtr & msg4,
                                       const sensor_msgs::msg::Image::ConstSharedPtr & msg5,
                                       const sensor_msgs::msg::Image::ConstSharedPtr & msg6,
                                       const sensor_msgs::msg::Image::ConstSharedPtr & msg7)
    {
      RCLCPP_INFO(g_node->get_logger(),
                  "timestamps of 8 images: %u.%u, %u.%u, %u.%u, %u.%u, %u.%u, %u.%u, %u.%u, %u.%u",
                  msg0->header.stamp.sec, msg0->header.stamp.nanosec,
                  msg1->header.stamp.sec, msg1->header.stamp.nanosec,
                  msg2->header.stamp.sec, msg2->header.stamp.nanosec,
                  msg3->header.stamp.sec, msg3->header.stamp.nanosec,
                  msg4->header.stamp.sec, msg4->header.stamp.nanosec,
                  msg5->header.stamp.sec, msg4->header.stamp.nanosec,
                  msg6->header.stamp.sec, msg4->header.stamp.nanosec,
                  msg7->header.stamp.sec, msg4->header.stamp.nanosec);

      auto time_offset_ns = (g_node->now() - msg0->header.stamp).nanoseconds();
      auto timestamp_offset_ns = (rclcpp::Time(msg0->header.stamp) - g_last_frame_ts).nanoseconds();
      auto time_offset_ms = time_offset_ns / 1000000.0F;
      auto timestamp_offset_ms = timestamp_offset_ns / 1000000.0F;
      RCLCPP_INFO(g_node->get_logger(), "get-frame-transport-time: %.3f", time_offset_ms);
      if(g_last_frame_ts.nanoseconds() > 0.0)
      {
        RCLCPP_INFO(g_node->get_logger(), "get-frame-timestamp_offset-time: %.3f", timestamp_offset_ms);
      }
      g_last_frame_ts = msg0->header.stamp;
      return;
    }


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("image_syncer");

  // define quality of service: all messages that you want to receive must have the same
//   rclcpp::QoS custom_qos_profile(rclcpp::KeepLast(20));
  rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(5)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      .avoid_ros_namespace_conventions(false);

  auto rmw_qos_profile_image = custom_qos_profile.get_rmw_qos_profile();

  std::string camera0_msg_name = "/image/image0";
  std::string camera1_msg_name = "/image/image1";
  std::string camera2_msg_name = "/image/image2";
  std::string camera3_msg_name = "/image/image3";
  std::string camera4_msg_name = "/image/image4";
  std::string camera5_msg_name = "/image/image5";
  std::string camera6_msg_name = "/image/image6";
  std::string camera7_msg_name = "/image/image7";

  // create subscribers to the topics of interest
  message_filters::Subscriber<Topic> camera0_sub(g_node, camera0_msg_name, rmw_qos_profile_image);
  message_filters::Subscriber<Topic> camera1_sub(g_node, camera1_msg_name, rmw_qos_profile_image);
  message_filters::Subscriber<Topic> camera2_sub(g_node, camera2_msg_name, rmw_qos_profile_image);
  message_filters::Subscriber<Topic> camera3_sub(g_node, camera3_msg_name, rmw_qos_profile_image);
  message_filters::Subscriber<Topic> camera4_sub(g_node, camera4_msg_name, rmw_qos_profile_image);
  message_filters::Subscriber<Topic> camera5_sub(g_node, camera5_msg_name, rmw_qos_profile_image);
  message_filters::Subscriber<Topic> camera6_sub(g_node, camera6_msg_name, rmw_qos_profile_image);
  message_filters::Subscriber<Topic> camera7_sub(g_node, camera7_msg_name, rmw_qos_profile_image);


  /**
   * create an exact time filter.
   * this can be done in two different ways
   */

  // method 1: exact time policy simplified API
  //message_filters::TimeSynchronizer<StampedStringMsg, StampedBooleanMsg> syncExact(string_sub, bool_sub, 10);

  // method 2: exact time policy
//   typedef message_filters::sync_policies::ExactTime<StampedStringMsg, StampedBooleanMsg> exact_policy;
//   message_filters::Synchronizer<exact_policy>syncExact(exact_policy(10), string_sub, bool_sub);

  // register the exact time callback
//   syncExact.registerCallback(exact_sync_callback);

  /**
   * create an approximate time filter.
   */

  typedef message_filters::sync_policies::ApproximateTime<Topic, Topic, Topic, Topic, Topic, Topic, Topic, Topic> approximate_policy;
  message_filters::Synchronizer<approximate_policy>syncApproximate(approximate_policy(20), camera0_sub, camera1_sub, camera2_sub,
                                                                            camera3_sub, camera4_sub, camera5_sub, camera6_sub, camera7_sub);

  // register the approximate time callback
  syncApproximate.registerCallback(cameraCallback);

// multiple executor?
  rclcpp::spin(g_node);

//   self clean

  rclcpp::shutdown();
  return 0;

}

