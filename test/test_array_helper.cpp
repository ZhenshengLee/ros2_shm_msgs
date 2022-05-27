/**
 * \file     test_array_helper.cpp
 * @author lizhensheng
 * \brief     My Param doc
 * @version 0.1
 * @date 2021-07-30
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <gtest/gtest.h>
#include "shm_msgs/array_helper.hpp"

TEST(shm_msgs, MemCpy)
{
  auto array_256 = std::array<unsigned char, 256>{'r','g','b'};
  std::string payload = "rgb";
  std::array<unsigned char, 256> array_0{0};

  std::memcpy(array_0.data(), payload.data(), payload.size());
  ASSERT_EQ(std::memcmp(array_0.data(), payload.data(), payload.size()), 0);
}

TEST(shm_msgs, ShmIsEqual)
{
  std::string payload = "rgb";
  shm_msgs::msg::String shm_str;
  std::memcpy(shm_str.data.data(), payload.data(), payload.size());
  shm_str.size = payload.size();
  ASSERT_TRUE(shm_msgs::is_equal(shm_str, payload));
  ASSERT_TRUE(shm_msgs::is_equal(shm_str, "rgb"));
  ASSERT_TRUE(shm_msgs::is_unequal(shm_str, "r"));
  ASSERT_TRUE(shm_msgs::is_unequal(shm_str, "g"));
  ASSERT_TRUE(shm_msgs::is_unequal(shm_str, "b"));
  ASSERT_TRUE(shm_msgs::is_unequal(shm_str, "rgba"));
  ASSERT_TRUE(shm_msgs::is_unequal(shm_str, "bgr"));
  ASSERT_TRUE(shm_msgs::is_unequal(shm_str, "bgra"));
  ASSERT_TRUE(shm_msgs::is_unequal(shm_str, ""));
}

TEST(shm_msgs, ShmSetStr)
{
  shm_msgs::msg::String shm_str;
  shm_msgs::set_str(shm_str, "rgb");
  ASSERT_EQ(shm_str.size, 3);
  ASSERT_TRUE(shm_msgs::is_equal(shm_str, "rgb"));
}

TEST(shm_msgs, ShmGetStr)
{
  shm_msgs::msg::String shm_str;
  shm_msgs::set_str(shm_str, "rgb");
  std::string std_string = shm_msgs::get_str(shm_str);
  ASSERT_EQ(std_string, "rgb");
}
