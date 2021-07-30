// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from shm_msgs:msg/String.idl
// generated code does not contain a copyright notice

#ifndef SHM_MSGS__MSG__DETAIL__STRING__TRAITS_HPP_
#define SHM_MSGS__MSG__DETAIL__STRING__TRAITS_HPP_

#include "shm_msgs/msg/detail/string__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const shm_msgs::msg::String & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size: ";
    value_to_yaml(msg.size, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const shm_msgs::msg::String & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<shm_msgs::msg::String>()
{
  return "shm_msgs::msg::String";
}

template<>
inline const char * name<shm_msgs::msg::String>()
{
  return "shm_msgs/msg/String";
}

template<>
struct has_fixed_size<shm_msgs::msg::String>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<shm_msgs::msg::String>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<shm_msgs::msg::String>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SHM_MSGS__MSG__DETAIL__STRING__TRAITS_HPP_
