// Copyright (c) 2022, ZhenshengLee.
// All rights reserved.

#include <gtest/gtest.h>
#include <sensor_msgs/image_encodings.hpp>

#include "shm_msgs/opencv_conversions.hpp"

#include <string>
#include <vector>

#include "opencv2/core/core.hpp"

using namespace sensor_msgs::image_encodings;

bool isUnsigned(const std::string & encoding)
{
  return encoding == RGB8 || encoding == RGBA8 || encoding == RGB16 || encoding == RGBA16 ||
         encoding == BGR8 || encoding == BGRA8 || encoding == BGR16 || encoding == BGRA16 ||
         encoding == MONO8 || encoding == MONO16 ||
         encoding == MONO8 || encoding == MONO16 || encoding == TYPE_8UC1 ||
         encoding == TYPE_8UC2 || encoding == TYPE_8UC3 || encoding == TYPE_8UC4 ||
         encoding == TYPE_16UC1 || encoding == TYPE_16UC2 || encoding == TYPE_16UC3 ||
         encoding == TYPE_16UC4;
  // BAYER_RGGB8, BAYER_BGGR8, BAYER_GBRG8, BAYER_GRBG8, BAYER_RGGB16,
  // BAYER_BGGR16, BAYER_GBRG16, BAYER_GRBG16, YUV422
}
std::vector<std::string>
getEncodings()
{
// TODO(N/A) for Groovy, the following types should be uncommented
  std::string encodings[] = {RGB8, RGBA8, RGB16, RGBA16, BGR8, BGRA8, BGR16, BGRA16, MONO8, MONO16,
    TYPE_8UC1, /*TYPE_8UC2,*/ TYPE_8UC3, TYPE_8UC4,
    TYPE_8SC1, /*TYPE_8SC2,*/ TYPE_8SC3, TYPE_8SC4,
    TYPE_16UC1, /*TYPE_16UC2,*/ TYPE_16UC3, TYPE_16UC4,
    TYPE_16SC1, /*TYPE_16SC2,*/ TYPE_16SC3, TYPE_16SC4,
    TYPE_32SC1, /*TYPE_32SC2,*/ TYPE_32SC3, TYPE_32SC4,
    TYPE_32FC1, /*TYPE_32FC2,*/ TYPE_32FC3, TYPE_32FC4,
    TYPE_64FC1, /*TYPE_64FC2,*/ TYPE_64FC3, TYPE_64FC4,
    // BAYER_RGGB8, BAYER_BGGR8, BAYER_GBRG8, BAYER_GRBG8,
    // BAYER_RGGB16, BAYER_BGGR16, BAYER_GBRG16, BAYER_GRBG16,
    YUV422, YUV422_YUY2};
  return std::vector<std::string>(encodings, encodings + 30);
}

TEST(OpencvTests, testCase_encode_decode4m)
{
  std::vector<std::string> encodings = getEncodings();
  for (size_t i = 0; i < encodings.size(); ++i) {
    std::string src_encoding = encodings[i];
    bool is_src_color_format = isColor(src_encoding) || isMono(src_encoding) ||
      (src_encoding == sensor_msgs::image_encodings::YUV422) ||
      (src_encoding == sensor_msgs::image_encodings::YUV422_YUY2);
    cv::Mat image_original(cv::Size(400, 400), shm_msgs::getCvType(src_encoding));
    cv::RNG r(77);
    r.fill(image_original, cv::RNG::UNIFORM, 0, 127);

    shm_msgs::msg::Image4m image_message;
    shm_msgs::CvImage image_bridge(std_msgs::msg::Header(), src_encoding, image_original);

    // Convert to a shm_msgs::Image
    shm_msgs::msg::Image4m::SharedPtr image_msg = image_bridge.toImageMsg4m();

    for (size_t j = 0; j < encodings.size(); ++j) {
      std::string dst_encoding = encodings[j];
      bool is_dst_color_format = isColor(dst_encoding) || isMono(dst_encoding) ||
        (dst_encoding == sensor_msgs::image_encodings::YUV422) ||
        (dst_encoding == sensor_msgs::image_encodings::YUV422_YUY2);
      bool is_num_channels_the_same = (numChannels(src_encoding) == numChannels(dst_encoding));

      shm_msgs::CvImageConstPtr cv_image;
      cv::Mat image_back;
      // If the first type does not contain any color information
      if (!is_src_color_format) {
        // Converting from a non color type to a color type does no make sense
        if (is_dst_color_format) {
          EXPECT_THROW(shm_msgs::toCvShare(image_msg, dst_encoding), shm_msgs::Exception);
          continue;
        }
        // We can only convert non-color types with the same number of channels
        if (!is_num_channels_the_same) {
          EXPECT_THROW(shm_msgs::toCvShare(image_msg, dst_encoding), shm_msgs::Exception);
          continue;
        }
        cv_image = shm_msgs::toCvShare(image_msg, dst_encoding);
      } else {
        // If we are converting to a non-color, you cannot convert to a different number of channels
        if (!is_dst_color_format) {
          if (!is_num_channels_the_same) {
            EXPECT_THROW(shm_msgs::toCvShare(image_msg, dst_encoding), shm_msgs::Exception);
            continue;
          }
          cv_image = shm_msgs::toCvShare(image_msg, dst_encoding);
          // We cannot convert from non-color to color
          EXPECT_THROW((void)cvtColor(cv_image, src_encoding)->image, shm_msgs::Exception);
          continue;
        }
        // We do not support conversion to YUV422 for now, except from YUV422
        if (((dst_encoding == YUV422) && (src_encoding != YUV422)) ||
          ((dst_encoding == YUV422_YUY2) && (src_encoding != YUV422_YUY2))) {
          EXPECT_THROW(shm_msgs::toCvShare(image_msg, dst_encoding), shm_msgs::Exception);
          continue;
        }

        cv_image = shm_msgs::toCvShare(image_msg, dst_encoding);

        // We do not support conversion to YUV422 for now, except from YUV422
        if (((src_encoding == YUV422) && (dst_encoding != YUV422)) ||
            ((src_encoding == YUV422_YUY2) && (dst_encoding != YUV422_YUY2))){
          EXPECT_THROW((void)cvtColor(cv_image, src_encoding)->image, shm_msgs::Exception);
          continue;
        }
      }
      // And convert back to a cv::Mat
      image_back = cvtColor(cv_image, src_encoding)->image;

      // If the number of channels,s different some information
      // got lost at some point, so no possible test
      if (!is_num_channels_the_same) {
        continue;
      }
      if (bitDepth(src_encoding) >= 32) {
        // In the case where the input has floats, we will lose precision but no more than 1
        EXPECT_LT(cv::norm(image_original, image_back, cv::NORM_INF),
          1) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      } else if ((bitDepth(src_encoding) == 16) && (bitDepth(dst_encoding) == 8)) {
        // In the case where the input has floats, we
        // will lose precision but no more than 1 * max(127)
        EXPECT_LT(cv::norm(image_original, image_back, cv::NORM_INF),
          128) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      } else {
        EXPECT_EQ(cv::norm(image_original, image_back, cv::NORM_INF),
          0) << "problem converting from " << src_encoding << " to " << dst_encoding <<
          " and back.";
      }
    }
  }
}
