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

#ifndef SHM_MSGS__POINT_CLOUD2_ITERATOR_8K_HPP_
#define SHM_MSGS__POINT_CLOUD2_ITERATOR_8K_HPP_

#include <shm_msgs/msg/point_cloud2.hpp>
#include <shm_msgs/array_helper.hpp>
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
 *   shm_msgs::msg::PointCloud8k cloud_msg;
 *   // Fill some internals of the PoinCloud2 like the header/width/height ...
 *   cloud_msgs.height = 1;  cloud_msgs.width = 4;
 *   // Set the point fields to xyzrgb and resize the vector with the following command
 *   // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
 *   // the number of occurrences of the type in the PointField, the type of the PointField
 *   shm_msgs::msg::PointCloud8kModifier modifier(cloud_msg);
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
 *   shm_msgs::PointCloud2Iterator8k<float> iter_x(cloud_msg, "x");
 *   shm_msgs::PointCloud2Iterator8k<float> iter_y(cloud_msg, "y");
 *   shm_msgs::PointCloud2Iterator8k<float> iter_z(cloud_msg, "z");
 *   // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
 *   // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
 *   // and RGBA as A,R,G,B)
 *   shm_msgs::PointCloud2Iterator8k<uint8_t> iter_r(cloud_msg, "r");
 *   shm_msgs::PointCloud2Iterator8k<uint8_t> iter_g(cloud_msg, "g");
 *   shm_msgs::PointCloud2Iterator8k<uint8_t> iter_b(cloud_msg, "b");
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
 * @brief Enables modifying a msg like a container
 */
template <typename msg>
class PointCloud2Modifier
{
public:
  /**
   * @brief Default constructor
   * @param cloud_msg The msg to modify
   */
  explicit PointCloud2Modifier(msg & cloud_msg);

  /**
   * @return the number of T's in the original msg
   */
  size_t size() const;

  /**
   * @param size The number of T's to reserve in the original msg for
   */
  void reserve(size_t size);

  /**
   * @param size The number of T's to change the size of the original msg by
   */
  void resize(size_t size);

  /**
   * @brief remove all T's from the original msg
   */
  void clear();

  /**
   * @brief Function setting some fields in a PointCloud and adjusting the
   *        internals of the PointCloud2
   * @param n_fields the number of fields to add. The fields are given as
   *        triplets: name of the field as char*, number of elements in the
   *        field, the datatype of the elements in the field
   *
   * E.g, you create your PointCloud2 message with XYZ/RGB as follows:
   * \verbatim
   *   setPointCloud2Fields(cloud_msg, 4, "x", 1, shm_msgs::msg::PointField::FLOAT32,
   *                                              "y", 1, shm_msgs::msg::PointField::FLOAT32,
   *                                              "z", 1, shm_msgs::msg::PointField::FLOAT32,
   *                                              "rgb", 1, shm_msgs::msg::PointField::FLOAT32);
   * \endverbatim
   * WARNING: THIS DOES NOT TAKE INTO ACCOUNT ANY PADDING AS DONE UNTIL HYDRO
   * For simple usual cases, the overloaded setPointCloud2FieldsByString is what you want.
   */
  void setPointCloud2Fields(int n_fields, ...);

  /**
   * @brief Function setting some fields in a PointCloud and adjusting the
   *        internals of the PointCloud2
   * @param n_fields the number of fields to add. The fields are given as
   *        strings: "xyz" (3 floats), "rgb" (3 uchar stacked in a float),
   *        "rgba" (4 uchar stacked in a float)
   * @return void
   *
   * WARNING: THIS FUNCTION DOES ADD ANY NECESSARY PADDING TRANSPARENTLY
   */
  void setPointCloud2FieldsByString(int n_fields, ...);

protected:
  /** A reference to the original msg that we read */
  msg & cloud_msg_;
};

namespace impl
{
/** Private base class for PointCloud2Iterator8k and PointCloud2ConstIterator8k
 * T is the type of the value on which the child class will be templated
 * TT is the type of the value to be retrieved (same as T except for constness)
 * U is the type of the raw data in PointCloud2 (only uchar and const uchar are supported)
 * C is the type of the pointcloud to intialize from (const or not)
 * V is the derived class (yop, curiously recurring template pattern)
 */
template<typename T, typename TT, typename U, typename C, template<typename> class V>
class PointCloud2IteratorBase
{
public:
  /**
   */
  PointCloud2IteratorBase();

  /**
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  PointCloud2IteratorBase(C & cloud_msg, const std::string & field_name);

  /** Assignment operator
   * @param iter the iterator to copy data from
   * @return a reference to *this
   */
  V<T> & operator=(const V<T> & iter);

  /** Access the i th element starting at the current pointer (useful when a field has several elements of the same
   * type)
   * @param i
   * @return a reference to the i^th value from the current position
   */
  TT & operator[](size_t i) const;

  /** Dereference the iterator. Equivalent to accessing it through [0]
   * @return the value to which the iterator is pointing
   */
  TT & operator*() const;

  /** Increase the iterator to the next element
   * @return a reference to the updated iterator
   */
  V<T> & operator++();

  /** Basic pointer addition
   * @param i the amount to increase the iterator by
   * @return an iterator with an increased position
   */
  V<T> operator+(int i);

  /** Increase the iterator by a certain amount
   * @return a reference to the updated iterator
   */
  V<T> & operator+=(int i);

  /** Compare to another iterator
   * @return whether the current iterator points to a different address than the other one
   */
  bool operator!=(const V<T> & iter) const;

  /** Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++)
   */
  // V<T> end() const;

  // V<T> begin() const;

private:
  /** Common code to set the field of the PointCloud2
   * @param cloud_msg the PointCloud2 to modify
   * @param field_name the name of the field to iterate upon
   * @return the offset at which the field is found
   */
  int set_field(const C & cloud_msg, const std::string & field_name);

  /** The "point_step" of the original cloud */
  int point_step_;
  /** The raw data  in uchar* where the iterator is */
  U * data_char_;
  /** The cast data where the iterator is */
  TT * data_;
  /** The end() pointer of the iterator */
  TT * data_end_;
  /** Whether the fields are stored as bigendian */
  bool is_bigendian_;
};
}  // namespace impl

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
 *   shm_msgs::msg::PointCloud8kIterator<float> iter_x(cloud_msg, "x");
 * \endverbatim
 * and then access X through iter_x[0] or *iter_x
 * You could create an iterator for Y and Z too but as they are consecutive,
 * you can just use iter_x[1] and iter_x[2]
 *
 * For iterating over RGB, you do:
 * \verbatim
 * shm_msgs::msg::PointCloud8kIterator<uint8_t> iter_rgb(cloud_msg, "rgb");
 * \endverbatim
 * and then access R,G,B through  iter_rgb[0], iter_rgb[1], iter_rgb[2]
 */
template<typename T>
class PointCloud2Iterator8k
  : public impl::PointCloud2IteratorBase<
    T, T, unsigned char, shm_msgs::msg::PointCloud8k, PointCloud2Iterator8k>
{
public:
  PointCloud2Iterator8k(
    shm_msgs::msg::PointCloud8k & cloud_msg,
    const std::string & field_name)
  : impl::PointCloud2IteratorBase<
      T, T, unsigned char,
      shm_msgs::msg::PointCloud8k,
      shm_msgs::PointCloud2Iterator8k
  >::PointCloud2IteratorBase(cloud_msg, field_name) {}
};

/**
 * \brief Same as a PointCloud2Iterator8k but for const data
 */
template<typename T>
class PointCloud2ConstIterator8k
  : public impl::PointCloud2IteratorBase<
    T, const T, const unsigned char, const shm_msgs::msg::PointCloud8k,
    PointCloud2ConstIterator8k>
{
public:
  PointCloud2ConstIterator8k(
    const shm_msgs::msg::PointCloud8k & cloud_msg,
    const std::string & field_name)
  : impl::PointCloud2IteratorBase<
      T, const T, const unsigned char,
      const shm_msgs::msg::PointCloud8k,
      shm_msgs::PointCloud2ConstIterator8k
  >::PointCloud2IteratorBase(cloud_msg, field_name) {}
};
}  // namespace shm_msgs

#include <shm_msgs/impl/point_cloud2_iterator.hpp> // NOLINT

#endif  // SHM_MSGS__POINT_CLOUD2_ITERATOR_8K_HPP_
