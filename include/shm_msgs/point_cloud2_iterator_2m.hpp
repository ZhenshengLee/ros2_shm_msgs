// Copyright (c) 2013, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Open Source Robotics Foundation, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally from:
// https://github.com/ros/common_msgs/blob/275b09a/shm_msgs/include/shm_msgs/point_cloud2_iterator.h

#ifndef SHM_MSGS__POINT_CLOUD2_ITERATOR_2M_HPP_
#define SHM_MSGS__POINT_CLOUD2_ITERATOR_2M_HPP_

#include <shm_msgs/msg/point_cloud2.hpp>
#include <shm_msgs/array_helper.hpp>
#include <shm_msgs/point_cloud2_iterator_2m.hpp>
#include <cstdarg>
#include <string>
#include <vector>

/**
 * \brief Tools for manipulating shm_msgs
 *
 * This file provides two sets of utilities to modify and parse PointCloud2
 * The first set allows you to conveniently set the fields by hand:
 * \verbatim
 *   #include <shm_msgs/point_cloud_iterator.h>
 *   // Create a PointCloud2
 *   shm_msgs::msg::PointCloud2m cloud_msg;
 *   // Fill some internals of the PoinCloud2 like the header/width/height ...
 *   cloud_msgs.height = 1;  cloud_msgs.width = 4;
 *   // Set the point fields to xyzrgb and resize the vector with the following command
 *   // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
 *   // the number of occurrences of the type in the PointField, the type of the PointField
 *   shm_msgs::msg::PointCloud2mModifier modifier(cloud_msg);
 *   modifier.setPointCloud2Fields(4, "x", 1, shm_msgs::msg::PointField::FLOAT32,
 *                                            "y", 1, shm_msgs::msg::PointField::FLOAT32,
 *                                            "z", 1, shm_msgs::msg::PointField::FLOAT32,
 *                                            "rgb", 1, shm_msgs::msg::PointField::FLOAT32);
 *   // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
 *   // You have to be aware that the following function does add extra padding for backward compatibility though
 *   // so it is definitely the solution of choice for PointXYZ and PointXYZRGB
 *   // 2 is for the number of fields to add
 *   modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
 *   // You can then reserve / resize as usual
 *   modifier.resize(100);
 * \endverbatim
 *
 * The second set allow you to traverse your PointCloud using an iterator:
 * \verbatim
 *   // Define some raw data we'll put in the PointCloud2
 *   float point_data[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
 *   uint8_t color_data[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
 *   // Define the iterators. When doing so, you define the Field you would like to iterate upon and
 *   // the type of you would like returned: it is not necessary the type of the PointField as sometimes
 *   // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
 *   shm_msgs::PointCloud2Iterator2m<float> iter_x(cloud_msg, "x");
 *   shm_msgs::PointCloud2Iterator2m<float> iter_y(cloud_msg, "y");
 *   shm_msgs::PointCloud2Iterator2m<float> iter_z(cloud_msg, "z");
 *   // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
 *   // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
 *   // and RGBA as A,R,G,B)
 *   shm_msgs::PointCloud2Iterator2m<uint8_t> iter_r(cloud_msg, "r");
 *   shm_msgs::PointCloud2Iterator2m<uint8_t> iter_g(cloud_msg, "g");
 *   shm_msgs::PointCloud2Iterator2m<uint8_t> iter_b(cloud_msg, "b");
 *   // Fill the PointCloud2
 *   for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
 *     *iter_x = point_data[3*i+0];
 *     *iter_y = point_data[3*i+1];
 *     *iter_z = point_data[3*i+2];
 *     *iter_r = color_data[3*i+0];
 *     *iter_g = color_data[3*i+1];
 *     *iter_b = color_data[3*i+2];
 *   }
 * \endverbatim
 */

namespace shm_msgs
{

/**
 * \brief Class that can iterate over a PointCloud2
 *
 * T type of the element being iterated upon
 * E.g, you create your PointClou2 message as follows:
 * \verbatim
 *   setPointCloud2FieldsByString(cloud_msg, 2, "xyz", "rgb");
 * \endverbatim
 *
 * For iterating over XYZ, you do :
 * \verbatim
 *   shm_msgs::msg::PointCloud2mIterator<float> iter_x(cloud_msg, "x");
 * \endverbatim
 * and then access X through iter_x[0] or *iter_x
 * You could create an iterator for Y and Z too but as they are consecutive,
 * you can just use iter_x[1] and iter_x[2]
 *
 * For iterating over RGB, you do:
 * \verbatim
 * shm_msgs::msg::PointCloud2mIterator<uint8_t> iter_rgb(cloud_msg, "rgb");
 * \endverbatim
 * and then access R,G,B through  iter_rgb[0], iter_rgb[1], iter_rgb[2]
 */
template<typename T>
class PointCloud2Iterator2m
  : public impl::PointCloud2IteratorBase<
    T, T, unsigned char, shm_msgs::msg::PointCloud2m, PointCloud2Iterator2m>
{
public:
  PointCloud2Iterator2m(
    shm_msgs::msg::PointCloud2m & cloud_msg,
    const std::string & field_name)
  : impl::PointCloud2IteratorBase<
      T, T, unsigned char,
      shm_msgs::msg::PointCloud2m,
      shm_msgs::PointCloud2Iterator2m
  >::PointCloud2IteratorBase(cloud_msg, field_name) {}
};

/**
 * \brief Same as a PointCloud2Iterator2m but for const data
 */
template<typename T>
class PointCloud2ConstIterator2m
  : public impl::PointCloud2IteratorBase<
    T, const T, const unsigned char, const shm_msgs::msg::PointCloud2m,
    PointCloud2ConstIterator2m>
{
public:
  PointCloud2ConstIterator2m(
    const shm_msgs::msg::PointCloud2m & cloud_msg,
    const std::string & field_name)
  : impl::PointCloud2IteratorBase<
      T, const T, const unsigned char,
      const shm_msgs::msg::PointCloud2m,
      shm_msgs::PointCloud2ConstIterator2m
  >::PointCloud2IteratorBase(cloud_msg, field_name) {}
};
}  // namespace shm_msgs

#include <shm_msgs/impl/point_cloud2_iterator.hpp> // NOLINT

#endif  // SHM_MSGS__POINT_CLOUD2_ITERATOR_2M_HPP_
