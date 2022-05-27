// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "shm_msgs/opencv_conversions.hpp"

#include <boost/endian/conversion.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <shm_msgs/image_encodings.hpp>
#include "rcpputils/endian.hpp"

#include <map>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>

namespace enc = shm_msgs::image_encodings;

namespace shm_msgs
{

static int depthStrToInt(const std::string depth)
{
  if (depth == "8U") {
    return 0;
  } else if (depth == "8S") {
    return 1;
  } else if (depth == "16U") {
    return 2;
  } else if (depth == "16S") {
    return 3;
  } else if (depth == "32S") {
    return 4;
  } else if (depth == "32F") {
    return 5;
  }
  return 6;
}

int getCvType(const std::string & encoding)
{
  // Check for the most common encodings first
  if (encoding == enc::BGR8) {return CV_8UC3;}
  if (encoding == enc::MONO8) {return CV_8UC1;}
  if (encoding == enc::RGB8) {return CV_8UC3;}
  if (encoding == enc::MONO16) {return CV_16UC1;}
  if (encoding == enc::BGR16) {return CV_16UC3;}
  if (encoding == enc::RGB16) {return CV_16UC3;}
  if (encoding == enc::BGRA8) {return CV_8UC4;}
  if (encoding == enc::RGBA8) {return CV_8UC4;}
  if (encoding == enc::BGRA16) {return CV_16UC4;}
  if (encoding == enc::RGBA16) {return CV_16UC4;}

  // For bayer, return one-channel
  if (encoding == enc::BAYER_RGGB8) {return CV_8UC1;}
  if (encoding == enc::BAYER_BGGR8) {return CV_8UC1;}
  if (encoding == enc::BAYER_GBRG8) {return CV_8UC1;}
  if (encoding == enc::BAYER_GRBG8) {return CV_8UC1;}
  if (encoding == enc::BAYER_RGGB16) {return CV_16UC1;}
  if (encoding == enc::BAYER_BGGR16) {return CV_16UC1;}
  if (encoding == enc::BAYER_GBRG16) {return CV_16UC1;}
  if (encoding == enc::BAYER_GRBG16) {return CV_16UC1;}

  // Miscellaneous
  if (encoding == enc::YUV422) {return CV_8UC2;}
  if (encoding == enc::YUV422_YUY2) {return CV_8UC2;}

  // Check all the generic content encodings
  std::cmatch m;

  if (std::regex_match(encoding.c_str(), m,
    std::regex("(8U|8S|16U|16S|32S|32F|64F)C([0-9]+)")))
  {
    return CV_MAKETYPE(depthStrToInt(m[1].str()), atoi(m[2].str().c_str()));
  }

  if (std::regex_match(encoding.c_str(), m,
    std::regex("(8U|8S|16U|16S|32S|32F|64F)")))
  {
    return CV_MAKETYPE(depthStrToInt(m[1].str()), 1);
  }

  throw Exception("Unrecognized image encoding [" + encoding + "]");
}

/// @cond DOXYGEN_IGNORE

enum Encoding { INVALID = -1, GRAY = 0, RGB, BGR, RGBA, BGRA, YUV422, YUV422_YUY2, BAYER_RGGB, BAYER_BGGR,
  BAYER_GBRG, BAYER_GRBG};

Encoding getEncoding(const std::string & encoding)
{
  if ((encoding == enc::MONO8) || (encoding == enc::MONO16)) {return GRAY;}
  if ((encoding == enc::BGR8) || (encoding == enc::BGR16)) {return BGR;}
  if ((encoding == enc::RGB8) || (encoding == enc::RGB16)) {return RGB;}
  if ((encoding == enc::BGRA8) || (encoding == enc::BGRA16)) {return BGRA;}
  if ((encoding == enc::RGBA8) || (encoding == enc::RGBA16)) {return RGBA;}
  if (encoding == enc::YUV422) {return YUV422;}
  if (encoding == enc::YUV422_YUY2) {return YUV422_YUY2;}

  if ((encoding == enc::BAYER_RGGB8) || (encoding == enc::BAYER_RGGB16)) {return BAYER_RGGB;}
  if ((encoding == enc::BAYER_BGGR8) || (encoding == enc::BAYER_BGGR16)) {return BAYER_BGGR;}
  if ((encoding == enc::BAYER_GBRG8) || (encoding == enc::BAYER_GBRG16)) {return BAYER_GBRG;}
  if ((encoding == enc::BAYER_GRBG8) || (encoding == enc::BAYER_GRBG16)) {return BAYER_GRBG;}

  // We don't support conversions to/from other types
  return INVALID;
}

static const int SAME_FORMAT = -1;

/** Return a lit of OpenCV conversion codes to get from one Format to the other
 * The key is a pair: <FromFormat, ToFormat> and the value a succession of OpenCV code conversion
 * It's not efficient code but it is only called once and the structure is small enough
 */
std::map<std::pair<Encoding, Encoding>, std::vector<int>> getConversionCodes()
{
  std::map<std::pair<Encoding, Encoding>, std::vector<int>> res;
  for (int i = 0; i <= 5; ++i) {
    res[std::pair<Encoding, Encoding>(Encoding(i), Encoding(i))].push_back(SAME_FORMAT);
  }

  res[std::make_pair(GRAY, RGB)].push_back(cv::COLOR_GRAY2RGB);
  res[std::make_pair(GRAY, BGR)].push_back(cv::COLOR_GRAY2BGR);
  res[std::make_pair(GRAY, RGBA)].push_back(cv::COLOR_GRAY2RGBA);
  res[std::make_pair(GRAY, BGRA)].push_back(cv::COLOR_GRAY2BGRA);

  res[std::make_pair(RGB, GRAY)].push_back(cv::COLOR_RGB2GRAY);
  res[std::make_pair(RGB, BGR)].push_back(cv::COLOR_RGB2BGR);
  res[std::make_pair(RGB, RGBA)].push_back(cv::COLOR_RGB2RGBA);
  res[std::make_pair(RGB, BGRA)].push_back(cv::COLOR_RGB2BGRA);

  res[std::make_pair(BGR, GRAY)].push_back(cv::COLOR_BGR2GRAY);
  res[std::make_pair(BGR, RGB)].push_back(cv::COLOR_BGR2RGB);
  res[std::make_pair(BGR, RGBA)].push_back(cv::COLOR_BGR2RGBA);
  res[std::make_pair(BGR, BGRA)].push_back(cv::COLOR_BGR2BGRA);

  res[std::make_pair(RGBA, GRAY)].push_back(cv::COLOR_RGBA2GRAY);
  res[std::make_pair(RGBA, RGB)].push_back(cv::COLOR_RGBA2RGB);
  res[std::make_pair(RGBA, BGR)].push_back(cv::COLOR_RGBA2BGR);
  res[std::make_pair(RGBA, BGRA)].push_back(cv::COLOR_RGBA2BGRA);

  res[std::make_pair(BGRA, GRAY)].push_back(cv::COLOR_BGRA2GRAY);
  res[std::make_pair(BGRA, RGB)].push_back(cv::COLOR_BGRA2RGB);
  res[std::make_pair(BGRA, BGR)].push_back(cv::COLOR_BGRA2BGR);
  res[std::make_pair(BGRA, RGBA)].push_back(cv::COLOR_BGRA2RGBA);

  res[std::make_pair(YUV422, GRAY)].push_back(cv::COLOR_YUV2GRAY_UYVY);
  res[std::make_pair(YUV422, RGB)].push_back(cv::COLOR_YUV2RGB_UYVY);
  res[std::make_pair(YUV422, BGR)].push_back(cv::COLOR_YUV2BGR_UYVY);
  res[std::make_pair(YUV422, RGBA)].push_back(cv::COLOR_YUV2RGBA_UYVY);
  res[std::make_pair(YUV422, BGRA)].push_back(cv::COLOR_YUV2BGRA_UYVY);

  res[std::make_pair(YUV422_YUY2, GRAY)].push_back(cv::COLOR_YUV2GRAY_YUY2);
  res[std::make_pair(YUV422_YUY2, RGB)].push_back(cv::COLOR_YUV2RGB_YUY2);
  res[std::make_pair(YUV422_YUY2, BGR)].push_back(cv::COLOR_YUV2BGR_YUY2);
  res[std::make_pair(YUV422_YUY2, RGBA)].push_back(cv::COLOR_YUV2RGBA_YUY2);
  res[std::make_pair(YUV422_YUY2, BGRA)].push_back(cv::COLOR_YUV2BGRA_YUY2);

  // Deal with Bayer
  res[std::make_pair(BAYER_RGGB, GRAY)].push_back(cv::COLOR_BayerBG2GRAY);
  res[std::make_pair(BAYER_RGGB, RGB)].push_back(cv::COLOR_BayerBG2RGB);
  res[std::make_pair(BAYER_RGGB, BGR)].push_back(cv::COLOR_BayerBG2BGR);

  res[std::make_pair(BAYER_BGGR, GRAY)].push_back(cv::COLOR_BayerRG2GRAY);
  res[std::make_pair(BAYER_BGGR, RGB)].push_back(cv::COLOR_BayerRG2RGB);
  res[std::make_pair(BAYER_BGGR, BGR)].push_back(cv::COLOR_BayerRG2BGR);

  res[std::make_pair(BAYER_GBRG, GRAY)].push_back(cv::COLOR_BayerGR2GRAY);
  res[std::make_pair(BAYER_GBRG, RGB)].push_back(cv::COLOR_BayerGR2RGB);
  res[std::make_pair(BAYER_GBRG, BGR)].push_back(cv::COLOR_BayerGR2BGR);

  res[std::make_pair(BAYER_GRBG, GRAY)].push_back(cv::COLOR_BayerGB2GRAY);
  res[std::make_pair(BAYER_GRBG, RGB)].push_back(cv::COLOR_BayerGB2RGB);
  res[std::make_pair(BAYER_GRBG, BGR)].push_back(cv::COLOR_BayerGB2BGR);

  return res;
}

const std::vector<int> getConversionCode(std::string src_encoding, std::string dst_encoding)
{
  Encoding src_encod = getEncoding(src_encoding);
  Encoding dst_encod = getEncoding(dst_encoding);
  bool is_src_color_format = enc::isColor(src_encoding) || enc::isMono(src_encoding) ||
    enc::isBayer(src_encoding) || (src_encoding == enc::YUV422) || (src_encoding == enc::YUV422_YUY2);
  bool is_dst_color_format = enc::isColor(dst_encoding) || enc::isMono(dst_encoding) ||
    enc::isBayer(dst_encoding) || (dst_encoding == enc::YUV422) || (dst_encoding == enc::YUV422_YUY2);
  bool is_num_channels_the_same =
    (enc::numChannels(src_encoding) == enc::numChannels(dst_encoding));

  // If we have no color info in the source, we can only convert to the same format which
  // was resolved in the previous condition. Otherwise, fail
  if (!is_src_color_format) {
    if (is_dst_color_format) {
      throw Exception("[" + src_encoding + "] is not a color format. but [" + dst_encoding +
              "] is. The conversion does not make sense");
    }
    if (!is_num_channels_the_same) {
      throw Exception(
              "[" + src_encoding + "] and [" + dst_encoding +
              "] do not have the same number of channel");
    }
    return std::vector<int>(1, SAME_FORMAT);
  }

  // If we are converting from a color type to a non color type, we can only do so if we stick
  // to the number of channels
  if (!is_dst_color_format) {
    if (!is_num_channels_the_same) {
      throw Exception("[" + src_encoding + "] is a color format but [" + dst_encoding + "] " +
              "is not so they must have the same OpenCV type, CV_8UC3, CV16UC1 ....");
    }
    return std::vector<int>(1, SAME_FORMAT);
  }

  // If we are converting from a color type to another type, then everything is fine
  static const std::map<std::pair<Encoding, Encoding>,
    std::vector<int>> CONVERSION_CODES = getConversionCodes();

  std::pair<Encoding, Encoding> key(src_encod, dst_encod);
  std::map<std::pair<Encoding, Encoding>,
    std::vector<int>>::const_iterator val = CONVERSION_CODES.find(key);
  if (val == CONVERSION_CODES.end()) {
    throw Exception("Unsupported conversion from [" + src_encoding +
            "] to [" + dst_encoding + "]");
  }

  // And deal with depth differences if the colors are different
  std::vector<int> res = val->second;
  if ((enc::bitDepth(src_encoding) != enc::bitDepth(dst_encoding)) &&
    (getEncoding(src_encoding) != getEncoding(dst_encoding)))
  {
    res.push_back(SAME_FORMAT);
  }

  return res;
}

/////////////////////////////////////// Image ///////////////////////////////////////////

// Converts a ROS Image to a cv::Mat by sharing the data or changing its endianness if needed
// zs: extend here
cv::Mat matFromImage(const shm_msgs::msg::Image8k & source)
{
  int source_type = getCvType(shm_msgs::get_str(source.encoding));
  int byte_depth = enc::bitDepth(shm_msgs::get_str(source.encoding)) / 8;
  int num_channels = enc::numChannels(shm_msgs::get_str(source.encoding));

  if (source.step < source.width * byte_depth * num_channels) {
    std::stringstream ss;
    ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " <<
      source.step << " != " <<
      source.width << " * " << byte_depth << " * " << num_channels;
    throw Exception(ss.str());
  }

  if (source.height * source.step > shm_msgs::msg::Image8k::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << source.height << " * " <<
      source.step << " > " << shm_msgs::msg::Image8k::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  // If the endianness is the same as locally, share the data
  cv::Mat mat(source.height, source.width, source_type, const_cast<uchar *>(&source.data[0]),
    source.step);

   if ((rcpputils::endian::native == rcpputils::endian::big && source.is_bigendian) ||
    (rcpputils::endian::native == rcpputils::endian::little && !source.is_bigendian) ||
    byte_depth == 1)
  {
    return mat;
  }

  // Otherwise, reinterpret the data as bytes and switch the channels accordingly
  mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels * byte_depth),
      const_cast<uchar *>(&source.data[0]), source.step);
  cv::Mat mat_swap(source.height, source.width, mat.type());

  std::vector<int> fromTo;
  fromTo.reserve(num_channels * byte_depth);
  for (int i = 0; i < num_channels; ++i) {
    for (int j = 0; j < byte_depth; ++j) {
      fromTo.push_back(byte_depth * i + j);
      fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
    }
  }
  cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

  // Interpret mat_swap back as the proper type
  mat_swap.reshape(num_channels);

  return mat_swap;
}
cv::Mat matFromImage(const shm_msgs::msg::Image512k & source)
{
  int source_type = getCvType(shm_msgs::get_str(source.encoding));
  int byte_depth = enc::bitDepth(shm_msgs::get_str(source.encoding)) / 8;
  int num_channels = enc::numChannels(shm_msgs::get_str(source.encoding));

  if (source.step < source.width * byte_depth * num_channels) {
    std::stringstream ss;
    ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " <<
      source.step << " != " <<
      source.width << " * " << byte_depth << " * " << num_channels;
    throw Exception(ss.str());
  }

  if (source.height * source.step > shm_msgs::msg::Image512k::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << source.height << " * " <<
      source.step << " > " << shm_msgs::msg::Image512k::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  // If the endianness is the same as locally, share the data
  cv::Mat mat(source.height, source.width, source_type, const_cast<uchar *>(&source.data[0]),
    source.step);

   if ((rcpputils::endian::native == rcpputils::endian::big && source.is_bigendian) ||
    (rcpputils::endian::native == rcpputils::endian::little && !source.is_bigendian) ||
    byte_depth == 1)
  {
    return mat;
  }

  // Otherwise, reinterpret the data as bytes and switch the channels accordingly
  mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels * byte_depth),
      const_cast<uchar *>(&source.data[0]), source.step);
  cv::Mat mat_swap(source.height, source.width, mat.type());

  std::vector<int> fromTo;
  fromTo.reserve(num_channels * byte_depth);
  for (int i = 0; i < num_channels; ++i) {
    for (int j = 0; j < byte_depth; ++j) {
      fromTo.push_back(byte_depth * i + j);
      fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
    }
  }
  cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

  // Interpret mat_swap back as the proper type
  mat_swap.reshape(num_channels);

  return mat_swap;
}
cv::Mat matFromImage(const shm_msgs::msg::Image1m & source)
{
  int source_type = getCvType(shm_msgs::get_str(source.encoding));
  int byte_depth = enc::bitDepth(shm_msgs::get_str(source.encoding)) / 8;
  int num_channels = enc::numChannels(shm_msgs::get_str(source.encoding));

  if (source.step < source.width * byte_depth * num_channels) {
    std::stringstream ss;
    ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " <<
      source.step << " != " <<
      source.width << " * " << byte_depth << " * " << num_channels;
    throw Exception(ss.str());
  }

  if (source.height * source.step > shm_msgs::msg::Image1m::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << source.height << " * " <<
      source.step << " > " << shm_msgs::msg::Image1m::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  // If the endianness is the same as locally, share the data
  cv::Mat mat(source.height, source.width, source_type, const_cast<uchar *>(&source.data[0]),
    source.step);

   if ((rcpputils::endian::native == rcpputils::endian::big && source.is_bigendian) ||
    (rcpputils::endian::native == rcpputils::endian::little && !source.is_bigendian) ||
    byte_depth == 1)
  {
    return mat;
  }

  // Otherwise, reinterpret the data as bytes and switch the channels accordingly
  mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels * byte_depth),
      const_cast<uchar *>(&source.data[0]), source.step);
  cv::Mat mat_swap(source.height, source.width, mat.type());

  std::vector<int> fromTo;
  fromTo.reserve(num_channels * byte_depth);
  for (int i = 0; i < num_channels; ++i) {
    for (int j = 0; j < byte_depth; ++j) {
      fromTo.push_back(byte_depth * i + j);
      fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
    }
  }
  cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

  // Interpret mat_swap back as the proper type
  mat_swap.reshape(num_channels);

  return mat_swap;
}
cv::Mat matFromImage(const shm_msgs::msg::Image2m & source)
{
  int source_type = getCvType(shm_msgs::get_str(source.encoding));
  int byte_depth = enc::bitDepth(shm_msgs::get_str(source.encoding)) / 8;
  int num_channels = enc::numChannels(shm_msgs::get_str(source.encoding));

  if (source.step < source.width * byte_depth * num_channels) {
    std::stringstream ss;
    ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " <<
      source.step << " != " <<
      source.width << " * " << byte_depth << " * " << num_channels;
    throw Exception(ss.str());
  }

  if (source.height * source.step > shm_msgs::msg::Image2m::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << source.height << " * " <<
      source.step << " > " << shm_msgs::msg::Image2m::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  // If the endianness is the same as locally, share the data
  cv::Mat mat(source.height, source.width, source_type, const_cast<uchar *>(&source.data[0]),
    source.step);

   if ((rcpputils::endian::native == rcpputils::endian::big && source.is_bigendian) ||
    (rcpputils::endian::native == rcpputils::endian::little && !source.is_bigendian) ||
    byte_depth == 1)
  {
    return mat;
  }

  // Otherwise, reinterpret the data as bytes and switch the channels accordingly
  mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels * byte_depth),
      const_cast<uchar *>(&source.data[0]), source.step);
  cv::Mat mat_swap(source.height, source.width, mat.type());

  std::vector<int> fromTo;
  fromTo.reserve(num_channels * byte_depth);
  for (int i = 0; i < num_channels; ++i) {
    for (int j = 0; j < byte_depth; ++j) {
      fromTo.push_back(byte_depth * i + j);
      fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
    }
  }
  cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

  // Interpret mat_swap back as the proper type
  mat_swap.reshape(num_channels);

  return mat_swap;
}
cv::Mat matFromImage(const shm_msgs::msg::Image4m & source)
{
  int source_type = getCvType(shm_msgs::get_str(source.encoding));
  int byte_depth = enc::bitDepth(shm_msgs::get_str(source.encoding)) / 8;
  int num_channels = enc::numChannels(shm_msgs::get_str(source.encoding));

  if (source.step < source.width * byte_depth * num_channels) {
    std::stringstream ss;
    ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " <<
      source.step << " != " <<
      source.width << " * " << byte_depth << " * " << num_channels;
    throw Exception(ss.str());
  }

  if (source.height * source.step > shm_msgs::msg::Image4m::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << source.height << " * " <<
      source.step << " > " << shm_msgs::msg::Image4m::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  // If the endianness is the same as locally, share the data
  cv::Mat mat(source.height, source.width, source_type, const_cast<uchar *>(&source.data[0]),
    source.step);

   if ((rcpputils::endian::native == rcpputils::endian::big && source.is_bigendian) ||
    (rcpputils::endian::native == rcpputils::endian::little && !source.is_bigendian) ||
    byte_depth == 1)
  {
    return mat;
  }

  // Otherwise, reinterpret the data as bytes and switch the channels accordingly
  mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels * byte_depth),
      const_cast<uchar *>(&source.data[0]), source.step);
  cv::Mat mat_swap(source.height, source.width, mat.type());

  std::vector<int> fromTo;
  fromTo.reserve(num_channels * byte_depth);
  for (int i = 0; i < num_channels; ++i) {
    for (int j = 0; j < byte_depth; ++j) {
      fromTo.push_back(byte_depth * i + j);
      fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
    }
  }
  cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

  // Interpret mat_swap back as the proper type
  mat_swap.reshape(num_channels);

  return mat_swap;
}
cv::Mat matFromImage(const shm_msgs::msg::Image8m & source)
{
  int source_type = getCvType(shm_msgs::get_str(source.encoding));
  int byte_depth = enc::bitDepth(shm_msgs::get_str(source.encoding)) / 8;
  int num_channels = enc::numChannels(shm_msgs::get_str(source.encoding));

  if (source.step < source.width * byte_depth * num_channels) {
    std::stringstream ss;
    ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " <<
      source.step << " != " <<
      source.width << " * " << byte_depth << " * " << num_channels;
    throw Exception(ss.str());
  }

  if (source.height * source.step > shm_msgs::msg::Image8m::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << source.height << " * " <<
      source.step << " > " << shm_msgs::msg::Image8m::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  // If the endianness is the same as locally, share the data
  cv::Mat mat(source.height, source.width, source_type, const_cast<uchar *>(&source.data[0]),
    source.step);

   if ((rcpputils::endian::native == rcpputils::endian::big && source.is_bigendian) ||
    (rcpputils::endian::native == rcpputils::endian::little && !source.is_bigendian) ||
    byte_depth == 1)
  {
    return mat;
  }

  // Otherwise, reinterpret the data as bytes and switch the channels accordingly
  mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels * byte_depth),
      const_cast<uchar *>(&source.data[0]), source.step);
  cv::Mat mat_swap(source.height, source.width, mat.type());

  std::vector<int> fromTo;
  fromTo.reserve(num_channels * byte_depth);
  for (int i = 0; i < num_channels; ++i) {
    for (int j = 0; j < byte_depth; ++j) {
      fromTo.push_back(byte_depth * i + j);
      fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
    }
  }
  cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

  // Interpret mat_swap back as the proper type
  mat_swap.reshape(num_channels);

  return mat_swap;
}

// Internal, used by toCvCopy and cvtColor
CvImagePtr toCvCopyImpl(
  const cv::Mat & source,
  const std_msgs::msg::Header & src_header,
  const std::string & src_encoding,
  const std::string & dst_encoding)
{
  // Copy metadata
  CvImagePtr ptr = std::make_shared<CvImage>();
  ptr->header = src_header;

  // Copy to new buffer if same encoding requested
  if (dst_encoding.empty() || dst_encoding == src_encoding) {
    ptr->encoding = src_encoding;
    source.copyTo(ptr->image);
  } else {
    // Convert the source data to the desired encoding
    const std::vector<int> conversion_codes = getConversionCode(src_encoding, dst_encoding);
    cv::Mat image1 = source;
    cv::Mat image2;
    for (size_t i = 0; i < conversion_codes.size(); ++i) {
      int conversion_code = conversion_codes[i];
      if (conversion_code == SAME_FORMAT) {
        // Same number of channels, but different bit depth
        int src_depth = enc::bitDepth(src_encoding);
        int dst_depth = enc::bitDepth(dst_encoding);
        // Keep the number of channels for now but changed to the final depth
        int image2_type = CV_MAKETYPE(CV_MAT_DEPTH(getCvType(dst_encoding)), image1.channels());

        // Do scaling between CV_8U [0,255] and CV_16U [0,65535] images.
        if (src_depth == 8 && dst_depth == 16) {
          image1.convertTo(image2, image2_type, 65535. / 255.);
        } else if (src_depth == 16 && dst_depth == 8) {
          image1.convertTo(image2, image2_type, 255. / 65535.);
        } else {
          image1.convertTo(image2, image2_type);
        }
      } else {
        // Perform color conversion
        cv::cvtColor(image1, image2, conversion_code);
      }
      image1 = image2;
    }
    ptr->image = image2;
    ptr->encoding = dst_encoding;
  }

  return ptr;
}

/// @endcond
// zs: extend here to other msg size
shm_msgs::msg::Image8k::SharedPtr CvImage::toImageMsg8k() const
{
  shm_msgs::msg::Image8k::SharedPtr ptr = std::make_shared<shm_msgs::msg::Image8k>();
  toImageMsg(*ptr);
  return ptr;
}
shm_msgs::msg::Image512k::SharedPtr CvImage::toImageMsg512k() const
{
  shm_msgs::msg::Image512k::SharedPtr ptr = std::make_shared<shm_msgs::msg::Image512k>();
  toImageMsg(*ptr);
  return ptr;
}
shm_msgs::msg::Image1m::SharedPtr CvImage::toImageMsg1m() const
{
  shm_msgs::msg::Image1m::SharedPtr ptr = std::make_shared<shm_msgs::msg::Image1m>();
  toImageMsg(*ptr);
  return ptr;
}
shm_msgs::msg::Image2m::SharedPtr CvImage::toImageMsg2m() const
{
  shm_msgs::msg::Image2m::SharedPtr ptr = std::make_shared<shm_msgs::msg::Image2m>();
  toImageMsg(*ptr);
  return ptr;
}
shm_msgs::msg::Image4m::SharedPtr CvImage::toImageMsg4m() const
{
  shm_msgs::msg::Image4m::SharedPtr ptr = std::make_shared<shm_msgs::msg::Image4m>();
  toImageMsg(*ptr);
  return ptr;
}
shm_msgs::msg::Image8m::SharedPtr CvImage::toImageMsg8m() const
{
  shm_msgs::msg::Image8m::SharedPtr ptr = std::make_shared<shm_msgs::msg::Image8m>();
  toImageMsg(*ptr);
  return ptr;
}

// zs: extend here
void CvImage::toImageMsg(shm_msgs::msg::Image8k & ros_image) const
{
  shm_msgs::set_header(ros_image.header, header);
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  shm_msgs::set_str(ros_image.encoding, encoding);
  ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  // ros_image.data.resize(size);
  if (size > shm_msgs::msg::Image8k::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << ros_image.height << " * " <<
      ros_image.step << " > " << shm_msgs::msg::Image8k::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  if (image.isContinuous()) {
    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
  } else {
    // Copy by row by row
    uchar * ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
    uchar * cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}
void CvImage::toImageMsg(shm_msgs::msg::Image512k & ros_image) const
{
  shm_msgs::set_header(ros_image.header, header);
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  shm_msgs::set_str(ros_image.encoding, encoding);
  ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  // ros_image.data.resize(size);
  if (size > shm_msgs::msg::Image512k::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << ros_image.height << " * " <<
      ros_image.step << " > " << shm_msgs::msg::Image512k::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  if (image.isContinuous()) {
    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
  } else {
    // Copy by row by row
    uchar * ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
    uchar * cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}
void CvImage::toImageMsg(shm_msgs::msg::Image1m & ros_image) const
{
  shm_msgs::set_header(ros_image.header, header);
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  shm_msgs::set_str(ros_image.encoding, encoding);
  ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  // ros_image.data.resize(size);
  if (size > shm_msgs::msg::Image1m::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << ros_image.height << " * " <<
      ros_image.step << " > " << shm_msgs::msg::Image1m::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  if (image.isContinuous()) {
    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
  } else {
    // Copy by row by row
    uchar * ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
    uchar * cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}
void CvImage::toImageMsg(shm_msgs::msg::Image2m & ros_image) const
{
  shm_msgs::set_header(ros_image.header, header);
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  shm_msgs::set_str(ros_image.encoding, encoding);
  ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  // ros_image.data.resize(size);
  if (size > shm_msgs::msg::Image2m::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << ros_image.height << " * " <<
      ros_image.step << " > " << shm_msgs::msg::Image2m::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  if (image.isContinuous()) {
    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
  } else {
    // Copy by row by row
    uchar * ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
    uchar * cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}
void CvImage::toImageMsg(shm_msgs::msg::Image4m & ros_image) const
{
  shm_msgs::set_header(ros_image.header, header);
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  shm_msgs::set_str(ros_image.encoding, encoding);
  ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  // ros_image.data.resize(size);
  if (size > shm_msgs::msg::Image4m::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << ros_image.height << " * " <<
      ros_image.step << " > " << shm_msgs::msg::Image4m::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  if (image.isContinuous()) {
    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
  } else {
    // Copy by row by row
    uchar * ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
    uchar * cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}
void CvImage::toImageMsg(shm_msgs::msg::Image8m & ros_image) const
{
  shm_msgs::set_header(ros_image.header, header);
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  shm_msgs::set_str(ros_image.encoding, encoding);
  ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  // ros_image.data.resize(size);
  if (size > shm_msgs::msg::Image8m::DATA_MAX_SIZE) {
    std::stringstream ss;
    ss << "Image is wrongly formed: height * step > size  or  " << ros_image.height << " * " <<
      ros_image.step << " > " << shm_msgs::msg::Image8m::DATA_MAX_SIZE;
    throw Exception(ss.str());
  }

  if (image.isContinuous()) {
    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
  } else {
    // Copy by row by row
    uchar * ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
    uchar * cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}

// zs: extend here
// Deep copy data, returnee is mutable
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image8k::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvCopy(*source, encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image512k::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvCopy(*source, encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image1m::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvCopy(*source, encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image2m::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvCopy(*source, encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image4m::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvCopy(*source, encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image8m::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvCopy(*source, encoding);
}

// zs: extend here
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image8k & source,
  const std::string & encoding)
{
  // Construct matrix pointing to source data
  return toCvCopyImpl(matFromImage(source), shm_msgs::get_header(source.header), shm_msgs::get_str(source.encoding), encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image512k & source,
  const std::string & encoding)
{
  // Construct matrix pointing to source data
  return toCvCopyImpl(matFromImage(source), shm_msgs::get_header(source.header), shm_msgs::get_str(source.encoding), encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image1m & source,
  const std::string & encoding)
{
  // Construct matrix pointing to source data
  return toCvCopyImpl(matFromImage(source), shm_msgs::get_header(source.header), shm_msgs::get_str(source.encoding), encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image2m & source,
  const std::string & encoding)
{
  // Construct matrix pointing to source data
  return toCvCopyImpl(matFromImage(source), shm_msgs::get_header(source.header), shm_msgs::get_str(source.encoding), encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image4m & source,
  const std::string & encoding)
{
  // Construct matrix pointing to source data
  return toCvCopyImpl(matFromImage(source), shm_msgs::get_header(source.header), shm_msgs::get_str(source.encoding), encoding);
}
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image8m & source,
  const std::string & encoding)
{
  // Construct matrix pointing to source data
  return toCvCopyImpl(matFromImage(source), shm_msgs::get_header(source.header), shm_msgs::get_str(source.encoding), encoding);
}

// Share const data, returnee is immutable
// zs: extend here
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image8k::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvShare(*source, source, encoding);
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image8k & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding)
{
  // If the encoding different or the endianness different, you have to copy
  if ((!encoding.empty() && shm_msgs::get_str(source.encoding) != encoding) || (!source.is_bigendian !=
    (rcpputils::endian::native != rcpputils::endian::big)))
  {
    return toCvCopy(source, encoding);
  }

  CvImagePtr ptr = std::make_shared<CvImage>();
  ptr->header = shm_msgs::get_header(source.header);
  ptr->encoding = shm_msgs::get_str(source.encoding);
  ptr->tracked_object_ = tracked_object;
  ptr->image = matFromImage(source);
  return ptr;
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image512k::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvShare(*source, source, encoding);
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image512k & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding)
{
  // If the encoding different or the endianness different, you have to copy
  if ((!encoding.empty() && shm_msgs::get_str(source.encoding) != encoding) || (!source.is_bigendian !=
    (rcpputils::endian::native != rcpputils::endian::big)))
  {
    return toCvCopy(source, encoding);
  }

  CvImagePtr ptr = std::make_shared<CvImage>();
  ptr->header = shm_msgs::get_header(source.header);
  ptr->encoding = shm_msgs::get_str(source.encoding);
  ptr->tracked_object_ = tracked_object;
  ptr->image = matFromImage(source);
  return ptr;
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image1m::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvShare(*source, source, encoding);
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image1m & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding)
{
  // If the encoding different or the endianness different, you have to copy
  if ((!encoding.empty() && shm_msgs::get_str(source.encoding) != encoding) || (!source.is_bigendian !=
    (rcpputils::endian::native != rcpputils::endian::big)))
  {
    return toCvCopy(source, encoding);
  }

  CvImagePtr ptr = std::make_shared<CvImage>();
  ptr->header = shm_msgs::get_header(source.header);
  ptr->encoding = shm_msgs::get_str(source.encoding);
  ptr->tracked_object_ = tracked_object;
  ptr->image = matFromImage(source);
  return ptr;
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image2m::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvShare(*source, source, encoding);
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image2m & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding)
{
  // If the encoding different or the endianness different, you have to copy
  if ((!encoding.empty() && shm_msgs::get_str(source.encoding) != encoding) || (!source.is_bigendian !=
    (rcpputils::endian::native != rcpputils::endian::big)))
  {
    return toCvCopy(source, encoding);
  }

  CvImagePtr ptr = std::make_shared<CvImage>();
  ptr->header = shm_msgs::get_header(source.header);
  ptr->encoding = shm_msgs::get_str(source.encoding);
  ptr->tracked_object_ = tracked_object;
  ptr->image = matFromImage(source);
  return ptr;
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image4m::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvShare(*source, source, encoding);
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image4m & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding)
{
  // If the encoding different or the endianness different, you have to copy
  if ((!encoding.empty() && shm_msgs::get_str(source.encoding) != encoding) || (!source.is_bigendian !=
    (rcpputils::endian::native != rcpputils::endian::big)))
  {
    return toCvCopy(source, encoding);
  }

  CvImagePtr ptr = std::make_shared<CvImage>();
  ptr->header = shm_msgs::get_header(source.header);
  ptr->encoding = shm_msgs::get_str(source.encoding);
  ptr->tracked_object_ = tracked_object;
  ptr->image = matFromImage(source);
  return ptr;
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image8m::ConstSharedPtr & source,
  const std::string & encoding)
{
  return toCvShare(*source, source, encoding);
}
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image8m & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding)
{
  // If the encoding different or the endianness different, you have to copy
  if ((!encoding.empty() && shm_msgs::get_str(source.encoding) != encoding) || (!source.is_bigendian !=
    (rcpputils::endian::native != rcpputils::endian::big)))
  {
    return toCvCopy(source, encoding);
  }

  CvImagePtr ptr = std::make_shared<CvImage>();
  ptr->header = shm_msgs::get_header(source.header);
  ptr->encoding = shm_msgs::get_str(source.encoding);
  ptr->tracked_object_ = tracked_object;
  ptr->image = matFromImage(source);
  return ptr;
}

CvImagePtr cvtColor(
  const CvImageConstPtr & source,
  const std::string & encoding)
{
  return toCvCopyImpl(source->image, source->header, source->encoding, encoding);
}

}  // namespace shm_msgs
