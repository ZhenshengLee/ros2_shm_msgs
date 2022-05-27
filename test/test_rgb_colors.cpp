#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "shm_msgs/rgb_colors.hpp"

TEST(RGBColors, testGetRGBColor)
{
  cv::Vec3d color;
  // red
  color = shm_msgs::rgb_colors::getRGBColor(shm_msgs::rgb_colors::RED);
  EXPECT_EQ(1, color[0]);
  EXPECT_EQ(0, color[1]);
  EXPECT_EQ(0, color[2]);
  // gray
  color = shm_msgs::rgb_colors::getRGBColor(shm_msgs::rgb_colors::GRAY);
  EXPECT_EQ(0.502, color[0]);
  EXPECT_EQ(0.502, color[1]);
  EXPECT_EQ(0.502, color[2]);
}
