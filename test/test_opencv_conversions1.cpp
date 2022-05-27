// Copyright (c) 2022, ZhenshengLee.
// All rights reserved.

#include <gtest/gtest.h>
#include <sensor_msgs/image_encodings.hpp>

#include "shm_msgs/opencv_conversions.hpp"

// Tests conversion of non-continuous cv::Mat. #5206
TEST(CvBridgeTest, NonContinuous8k)
{
  cv::Mat full = cv::Mat::eye(8, 8, CV_16U);
  cv::Mat partial = full.colRange(2, 5);

  shm_msgs::CvImage cvi;
  cvi.encoding = sensor_msgs::image_encodings::MONO16;
  cvi.image = partial;

  shm_msgs::msg::Image8k::SharedPtr msg8k = cvi.toImageMsg8k();
  EXPECT_EQ(static_cast<int>(msg8k->height), 8);
  EXPECT_EQ(static_cast<int>(msg8k->width), 3);
  EXPECT_TRUE(shm_msgs::is_equal(msg8k->encoding, cvi.encoding));
  EXPECT_EQ(static_cast<int>(msg8k->step), 6);
}
TEST(CvBridgeTest, NonContinuous512k)
{
  cv::Mat full = cv::Mat::eye(8, 8, CV_16U);
  cv::Mat partial = full.colRange(2, 5);

  shm_msgs::CvImage cvi;
  cvi.encoding = sensor_msgs::image_encodings::MONO16;
  cvi.image = partial;

  shm_msgs::msg::Image512k::SharedPtr msg512k = cvi.toImageMsg512k();
  EXPECT_EQ(static_cast<int>(msg512k->height), 8);
  EXPECT_EQ(static_cast<int>(msg512k->width), 3);
  EXPECT_TRUE(shm_msgs::is_equal(msg512k->encoding, cvi.encoding));
  EXPECT_EQ(static_cast<int>(msg512k->step), 6);
}

TEST(CvBridgeTest, ChannelOrder1m)
{
  cv::Mat_<uint16_t> mat(200, 200);
  mat.setTo(cv::Scalar(1000, 0, 0, 0));
  shm_msgs::msg::Image1m::SharedPtr image1m(new shm_msgs::msg::Image1m());

  image1m = shm_msgs::CvImage(shm_msgs::get_header(image1m->header), sensor_msgs::image_encodings::MONO16, mat).toImageMsg1m();

  shm_msgs::CvImageConstPtr cv_ptr = shm_msgs::toCvShare(image1m);

  shm_msgs::CvImagePtr res = shm_msgs::cvtColor(cv_ptr, sensor_msgs::image_encodings::BGR8);
  EXPECT_EQ(res->encoding, sensor_msgs::image_encodings::BGR8);
  EXPECT_EQ(res->image.type(), shm_msgs::getCvType(res->encoding));
  EXPECT_EQ(res->image.channels(), sensor_msgs::image_encodings::numChannels(res->encoding));
  EXPECT_EQ(res->image.depth(), CV_8U);

  // The matrix should be the following
  cv::Mat_<cv::Vec3b> gt(200, 200);
  gt.setTo(cv::Scalar(1, 1, 1) * 1000. * 255. / 65535.);

  ASSERT_EQ(res->image.type(), gt.type());
  EXPECT_EQ(cv::norm(res->image, gt, cv::NORM_INF), 0);
}

TEST(CvBridgeTest, initialization512k)
{
  shm_msgs::msg::Image512k image512k;
  shm_msgs::CvImagePtr cv_ptr;

  shm_msgs::set_str(image512k.encoding, "bgr8");
  image512k.height = 200;
  image512k.width = 200;

  try {
    cv_ptr = shm_msgs::toCvCopy(image512k, "mono8");
    // Before the fix, it would never get here, as it would segfault
    EXPECT_EQ(1, 0);
  } catch (shm_msgs::Exception & e) {
    EXPECT_EQ(1, 1);
  }

  // Check some normal images with different ratios
  for (int height = 100; height <= 300; ++height) {
    shm_msgs::set_str(image512k.encoding, shm_msgs::image_encodings::RGB8);
    image512k.step = image512k.width * 3;
    // image.data.resize(image.height * image.step);
    cv_ptr = shm_msgs::toCvCopy(image512k, "mono8");
  }
}

TEST(CvBridgeTest, imageMessageStep512k)
{
  // Test 1: image step is padded
  shm_msgs::msg::Image512k image512k;
  shm_msgs::CvImagePtr cv_ptr;

  shm_msgs::set_str(image512k.encoding, "mono8");
  image512k.height = 220;
  image512k.width = 200;
  image512k.is_bigendian = false;
  image512k.step = 208;

  // image.data.resize(image.height * image.step);

  ASSERT_NO_THROW(cv_ptr = shm_msgs::toCvCopy(image512k, "mono8"));
  ASSERT_EQ(220, cv_ptr->image.rows);
  ASSERT_EQ(200, cv_ptr->image.cols);
  // OpenCV copyTo argument removes the stride
  ASSERT_EQ(200, static_cast<int>(cv_ptr->image.step[0]));

  // Test 2: image512k step is invalid
  image512k.step = 199;

  ASSERT_THROW(cv_ptr = shm_msgs::toCvCopy(image512k, "mono8"), shm_msgs::Exception);

  // Test 3: image512k step == image512k.width * element size * number of channels
  image512k.step = 200;
  // image512k.data.resize(image512k.height * image512k.step);

  ASSERT_NO_THROW(cv_ptr = shm_msgs::toCvCopy(image512k, "mono8"));
  ASSERT_EQ(220, cv_ptr->image.rows);
  ASSERT_EQ(200, cv_ptr->image.cols);
  ASSERT_EQ(200, static_cast<int>(cv_ptr->image.step[0]));
}

TEST(CvBridgeTest, imageMessageConversion2m)
{
  shm_msgs::msg::Image2m imgmsg2m;
  shm_msgs::CvImagePtr cv_ptr;
  imgmsg2m.height = 220;
  imgmsg2m.width = 200;
  imgmsg2m.is_bigendian = false;

  // image with data type float32 and 1 channels
  shm_msgs::set_str(imgmsg2m.encoding, "32FC1");
  imgmsg2m.step = imgmsg2m.width * 32 / 8 * 1;
  // imgmsg.data.resize(imgmsg.height * imgmsg.step);
  ASSERT_NO_THROW(cv_ptr = shm_msgs::toCvCopy(imgmsg2m, shm_msgs::get_str(imgmsg2m.encoding)));
  ASSERT_EQ(static_cast<int>(imgmsg2m.height), cv_ptr->image.rows);
  ASSERT_EQ(static_cast<int>(imgmsg2m.width), cv_ptr->image.cols);
  ASSERT_EQ(1, cv_ptr->image.channels());
  ASSERT_EQ(imgmsg2m.step, cv_ptr->image.step[0]);

  // image with data type float32 and 10 channels
  shm_msgs::set_str(imgmsg2m.encoding, "32FC10");
  imgmsg2m.step = imgmsg2m.width * 32 / 8 * 10;
  // imgmsg.data.resize(imgmsg.height * imgmsg.step);
  ASSERT_NO_THROW(cv_ptr = shm_msgs::toCvCopy(imgmsg2m, shm_msgs::get_str(imgmsg2m.encoding)));
  ASSERT_EQ(static_cast<int>(imgmsg2m.height), cv_ptr->image.rows);
  ASSERT_EQ(static_cast<int>(imgmsg2m.width), cv_ptr->image.cols);
  ASSERT_EQ(10, cv_ptr->image.channels());
  ASSERT_EQ(imgmsg2m.step, cv_ptr->image.step[0]);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
