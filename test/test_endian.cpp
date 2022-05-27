#include <boost/endian/conversion.hpp>
#include <shm_msgs/opencv_conversions.hpp>
#include <shm_msgs/array_helper.hpp>
#include <gtest/gtest.h>
#include <memory>

TEST(CvBridgeTest, endianness)
{
  using namespace boost::endian;

  // Create an image of the type opposite to the platform
  shm_msgs::msg::Image8k msg8k;
  msg8k.height = 1;
  msg8k.width = 1;
  shm_msgs::set_str(msg8k.encoding, "32SC2");
  msg8k.step = 8;

  // msg.data.resize(msg.step);
  // the size if fixed to shm_msgs::msg::Image8k::DATA_MAX_SIZE
  int32_t * data = reinterpret_cast<int32_t *>(&msg8k.data[0]);

  // Write 1 and 2 in order, but with an endianness opposite to the platform
  if (order::native == order::little) {
    msg8k.is_bigendian = true;
    *(data++) = native_to_big(static_cast<int32_t>(1));
    *data = native_to_big(static_cast<int32_t>(2));
  } else {
    msg8k.is_bigendian = false;
    *(data++) = native_to_little(static_cast<int32_t>(1));
    *data = native_to_little(static_cast<int32_t>(2));
  }

  // Make sure the values are still the same
  shm_msgs::CvImageConstPtr img =
    shm_msgs::toCvShare(std::make_shared<shm_msgs::msg::Image8k>(msg8k));
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[0], 1);
  EXPECT_EQ(img->image.at<cv::Vec2i>(0, 0)[1], 2);
  // Make sure we cannot share data
  EXPECT_NE(img->image.data, &msg8k.data[0]);
}
