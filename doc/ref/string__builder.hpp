// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from shm_msgs:msg/String.idl
// generated code does not contain a copyright notice

#ifndef SHM_MSGS__MSG__DETAIL__STRING__BUILDER_HPP_
#define SHM_MSGS__MSG__DETAIL__STRING__BUILDER_HPP_

#include "shm_msgs/msg/detail/string__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace shm_msgs
{

namespace msg
{

namespace builder
{

class Init_String_data
{
public:
  explicit Init_String_data(::shm_msgs::msg::String & msg)
  : msg_(msg)
  {}
  ::shm_msgs::msg::String data(::shm_msgs::msg::String::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::shm_msgs::msg::String msg_;
};

class Init_String_size
{
public:
  Init_String_size()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_String_data size(::shm_msgs::msg::String::_size_type arg)
  {
    msg_.size = std::move(arg);
    return Init_String_data(msg_);
  }

private:
  ::shm_msgs::msg::String msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::shm_msgs::msg::String>()
{
  return shm_msgs::msg::builder::Init_String_size();
}

}  // namespace shm_msgs

#endif  // SHM_MSGS__MSG__DETAIL__STRING__BUILDER_HPP_
