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

// C++
#include <memory>
#include <string>

#include "shm_msgs/open3d_conversions.hpp"
#include "shm_msgs/array_helper.hpp"

namespace shm_msgs
{
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  shm_msgs::msg::PointCloud8k & ros_pc2, std::string frame_id)
{
  shm_msgs::PointCloud2Modifier<shm_msgs::msg::PointCloud8k> modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  shm_msgs::set_str(ros_pc2.header.frame_id, frame_id);
  shm_msgs::PointCloud2Iterator8k<float> ros_pc2_x(ros_pc2, "x");
  shm_msgs::PointCloud2Iterator8k<float> ros_pc2_y(ros_pc2, "y");
  shm_msgs::PointCloud2Iterator8k<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors()) {
    shm_msgs::PointCloud2Iterator8k<uint8_t> ros_pc2_r(ros_pc2, "r");
    shm_msgs::PointCloud2Iterator8k<uint8_t> ros_pc2_g(ros_pc2, "g");
    shm_msgs::PointCloud2Iterator8k<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x,
      ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
      ++ros_pc2_b)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      const Eigen::Vector3d & color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = static_cast<int>(255 * color(0));
      *ros_pc2_g = static_cast<int>(255 * color(1));
      *ros_pc2_b = static_cast<int>(255 * color(2));
    }
  } else {
    for (size_t i = 0; i < pointcloud.points_.size();
      i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  shm_msgs::msg::PointCloud512k & ros_pc2, std::string frame_id)
{
  shm_msgs::PointCloud2Modifier<shm_msgs::msg::PointCloud512k> modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  shm_msgs::set_str(ros_pc2.header.frame_id, frame_id);
  shm_msgs::PointCloud2Iterator512k<float> ros_pc2_x(ros_pc2, "x");
  shm_msgs::PointCloud2Iterator512k<float> ros_pc2_y(ros_pc2, "y");
  shm_msgs::PointCloud2Iterator512k<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors()) {
    shm_msgs::PointCloud2Iterator512k<uint8_t> ros_pc2_r(ros_pc2, "r");
    shm_msgs::PointCloud2Iterator512k<uint8_t> ros_pc2_g(ros_pc2, "g");
    shm_msgs::PointCloud2Iterator512k<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x,
      ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
      ++ros_pc2_b)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      const Eigen::Vector3d & color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = static_cast<int>(255 * color(0));
      *ros_pc2_g = static_cast<int>(255 * color(1));
      *ros_pc2_b = static_cast<int>(255 * color(2));
    }
  } else {
    for (size_t i = 0; i < pointcloud.points_.size();
      i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  shm_msgs::msg::PointCloud1m & ros_pc2, std::string frame_id)
{
  shm_msgs::PointCloud2Modifier<shm_msgs::msg::PointCloud1m> modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  shm_msgs::set_str(ros_pc2.header.frame_id, frame_id);
  shm_msgs::PointCloud2Iterator1m<float> ros_pc2_x(ros_pc2, "x");
  shm_msgs::PointCloud2Iterator1m<float> ros_pc2_y(ros_pc2, "y");
  shm_msgs::PointCloud2Iterator1m<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors()) {
    shm_msgs::PointCloud2Iterator1m<uint8_t> ros_pc2_r(ros_pc2, "r");
    shm_msgs::PointCloud2Iterator1m<uint8_t> ros_pc2_g(ros_pc2, "g");
    shm_msgs::PointCloud2Iterator1m<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x,
      ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
      ++ros_pc2_b)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      const Eigen::Vector3d & color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = static_cast<int>(255 * color(0));
      *ros_pc2_g = static_cast<int>(255 * color(1));
      *ros_pc2_b = static_cast<int>(255 * color(2));
    }
  } else {
    for (size_t i = 0; i < pointcloud.points_.size();
      i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  shm_msgs::msg::PointCloud2m & ros_pc2, std::string frame_id)
{
  shm_msgs::PointCloud2Modifier<shm_msgs::msg::PointCloud2m> modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  shm_msgs::set_str(ros_pc2.header.frame_id, frame_id);
  shm_msgs::PointCloud2Iterator2m<float> ros_pc2_x(ros_pc2, "x");
  shm_msgs::PointCloud2Iterator2m<float> ros_pc2_y(ros_pc2, "y");
  shm_msgs::PointCloud2Iterator2m<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors()) {
    shm_msgs::PointCloud2Iterator2m<uint8_t> ros_pc2_r(ros_pc2, "r");
    shm_msgs::PointCloud2Iterator2m<uint8_t> ros_pc2_g(ros_pc2, "g");
    shm_msgs::PointCloud2Iterator2m<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x,
      ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
      ++ros_pc2_b)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      const Eigen::Vector3d & color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = static_cast<int>(255 * color(0));
      *ros_pc2_g = static_cast<int>(255 * color(1));
      *ros_pc2_b = static_cast<int>(255 * color(2));
    }
  } else {
    for (size_t i = 0; i < pointcloud.points_.size();
      i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  shm_msgs::msg::PointCloud4m & ros_pc2, std::string frame_id)
{
  shm_msgs::PointCloud2Modifier<shm_msgs::msg::PointCloud4m> modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  shm_msgs::set_str(ros_pc2.header.frame_id, frame_id);
  shm_msgs::PointCloud2Iterator4m<float> ros_pc2_x(ros_pc2, "x");
  shm_msgs::PointCloud2Iterator4m<float> ros_pc2_y(ros_pc2, "y");
  shm_msgs::PointCloud2Iterator4m<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors()) {
    shm_msgs::PointCloud2Iterator4m<uint8_t> ros_pc2_r(ros_pc2, "r");
    shm_msgs::PointCloud2Iterator4m<uint8_t> ros_pc2_g(ros_pc2, "g");
    shm_msgs::PointCloud2Iterator4m<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x,
      ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
      ++ros_pc2_b)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      const Eigen::Vector3d & color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = static_cast<int>(255 * color(0));
      *ros_pc2_g = static_cast<int>(255 * color(1));
      *ros_pc2_b = static_cast<int>(255 * color(2));
    }
  } else {
    for (size_t i = 0; i < pointcloud.points_.size();
      i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  shm_msgs::msg::PointCloud8m & ros_pc2, std::string frame_id)
{
  shm_msgs::PointCloud2Modifier<shm_msgs::msg::PointCloud8m> modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  shm_msgs::set_str(ros_pc2.header.frame_id, frame_id);
  shm_msgs::PointCloud2Iterator8m<float> ros_pc2_x(ros_pc2, "x");
  shm_msgs::PointCloud2Iterator8m<float> ros_pc2_y(ros_pc2, "y");
  shm_msgs::PointCloud2Iterator8m<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors()) {
    shm_msgs::PointCloud2Iterator8m<uint8_t> ros_pc2_r(ros_pc2, "r");
    shm_msgs::PointCloud2Iterator8m<uint8_t> ros_pc2_g(ros_pc2, "g");
    shm_msgs::PointCloud2Iterator8m<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x,
      ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
      ++ros_pc2_b)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      const Eigen::Vector3d & color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = static_cast<int>(255 * color(0));
      *ros_pc2_g = static_cast<int>(255 * color(1));
      *ros_pc2_b = static_cast<int>(255 * color(2));
    }
  } else {
    for (size_t i = 0; i < pointcloud.points_.size();
      i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}

void rosToOpen3d(
  const shm_msgs::msg::PointCloud8k::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc, bool skip_colors)
{
  shm_msgs::PointCloud2ConstIterator8k<float> ros_pc2_x(*ros_pc2, "x");
  shm_msgs::PointCloud2ConstIterator8k<float> ros_pc2_y(*ros_pc2, "y");
  shm_msgs::PointCloud2ConstIterator8k<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
  if (ros_pc2->fields_size == 3 || skip_colors == true) {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
      ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(
        Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  } else {
    o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
    if (shm_msgs::is_equal(ros_pc2->fields[3].name, "rgb")) {
      shm_msgs::PointCloud2ConstIterator8k<uint8_t> ros_pc2_r(*ros_pc2, "r");
      shm_msgs::PointCloud2ConstIterator8k<uint8_t> ros_pc2_g(*ros_pc2, "g");
      shm_msgs::PointCloud2ConstIterator8k<uint8_t> ros_pc2_b(*ros_pc2, "b");

      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x,
        ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(
            (static_cast<int>(*ros_pc2_r)) / 255.0,
            (static_cast<int>(*ros_pc2_g)) / 255.0,
            (static_cast<int>(*ros_pc2_b)) / 255.0));
      }
    } else if (shm_msgs::is_equal(ros_pc2->fields[3].name, "intensity")) {
      shm_msgs::PointCloud2ConstIterator8k<uint8_t> ros_pc2_i(*ros_pc2,
        "intensity");
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
        ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
      }
    }
  }
}
void rosToOpen3d(
  const shm_msgs::msg::PointCloud512k::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc, bool skip_colors)
{
  shm_msgs::PointCloud2ConstIterator512k<float> ros_pc2_x(*ros_pc2, "x");
  shm_msgs::PointCloud2ConstIterator512k<float> ros_pc2_y(*ros_pc2, "y");
  shm_msgs::PointCloud2ConstIterator512k<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
  if (ros_pc2->fields_size == 3 || skip_colors == true) {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
      ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(
        Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  } else {
    o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
    if (shm_msgs::is_equal(ros_pc2->fields[3].name, "rgb")) {
      shm_msgs::PointCloud2ConstIterator512k<uint8_t> ros_pc2_r(*ros_pc2, "r");
      shm_msgs::PointCloud2ConstIterator512k<uint8_t> ros_pc2_g(*ros_pc2, "g");
      shm_msgs::PointCloud2ConstIterator512k<uint8_t> ros_pc2_b(*ros_pc2, "b");

      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x,
        ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(
            (static_cast<int>(*ros_pc2_r)) / 255.0,
            (static_cast<int>(*ros_pc2_g)) / 255.0,
            (static_cast<int>(*ros_pc2_b)) / 255.0));
      }
    } else if (shm_msgs::is_equal(ros_pc2->fields[3].name, "intensity")) {
      shm_msgs::PointCloud2ConstIterator512k<uint8_t> ros_pc2_i(*ros_pc2,
        "intensity");
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
        ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
      }
    }
  }
}
void rosToOpen3d(
  const shm_msgs::msg::PointCloud1m::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc, bool skip_colors)
{
  shm_msgs::PointCloud2ConstIterator1m<float> ros_pc2_x(*ros_pc2, "x");
  shm_msgs::PointCloud2ConstIterator1m<float> ros_pc2_y(*ros_pc2, "y");
  shm_msgs::PointCloud2ConstIterator1m<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
  if (ros_pc2->fields_size == 3 || skip_colors == true) {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
      ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(
        Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  } else {
    o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
    if (shm_msgs::is_equal(ros_pc2->fields[3].name, "rgb")) {
      shm_msgs::PointCloud2ConstIterator1m<uint8_t> ros_pc2_r(*ros_pc2, "r");
      shm_msgs::PointCloud2ConstIterator1m<uint8_t> ros_pc2_g(*ros_pc2, "g");
      shm_msgs::PointCloud2ConstIterator1m<uint8_t> ros_pc2_b(*ros_pc2, "b");

      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x,
        ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(
            (static_cast<int>(*ros_pc2_r)) / 255.0,
            (static_cast<int>(*ros_pc2_g)) / 255.0,
            (static_cast<int>(*ros_pc2_b)) / 255.0));
      }
    } else if (shm_msgs::is_equal(ros_pc2->fields[3].name, "intensity")) {
      shm_msgs::PointCloud2ConstIterator1m<uint8_t> ros_pc2_i(*ros_pc2,
        "intensity");
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
        ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
      }
    }
  }
}
void rosToOpen3d(
  const shm_msgs::msg::PointCloud2m::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc, bool skip_colors)
{
  shm_msgs::PointCloud2ConstIterator2m<float> ros_pc2_x(*ros_pc2, "x");
  shm_msgs::PointCloud2ConstIterator2m<float> ros_pc2_y(*ros_pc2, "y");
  shm_msgs::PointCloud2ConstIterator2m<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
  if (ros_pc2->fields_size == 3 || skip_colors == true) {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
      ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(
        Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  } else {
    o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
    if (shm_msgs::is_equal(ros_pc2->fields[3].name, "rgb")) {
      shm_msgs::PointCloud2ConstIterator2m<uint8_t> ros_pc2_r(*ros_pc2, "r");
      shm_msgs::PointCloud2ConstIterator2m<uint8_t> ros_pc2_g(*ros_pc2, "g");
      shm_msgs::PointCloud2ConstIterator2m<uint8_t> ros_pc2_b(*ros_pc2, "b");

      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x,
        ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(
            (static_cast<int>(*ros_pc2_r)) / 255.0,
            (static_cast<int>(*ros_pc2_g)) / 255.0,
            (static_cast<int>(*ros_pc2_b)) / 255.0));
      }
    } else if (shm_msgs::is_equal(ros_pc2->fields[3].name, "intensity")) {
      shm_msgs::PointCloud2ConstIterator2m<uint8_t> ros_pc2_i(*ros_pc2,
        "intensity");
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
        ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
      }
    }
  }
}
void rosToOpen3d(
  const shm_msgs::msg::PointCloud4m::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc, bool skip_colors)
{
  shm_msgs::PointCloud2ConstIterator4m<float> ros_pc2_x(*ros_pc2, "x");
  shm_msgs::PointCloud2ConstIterator4m<float> ros_pc2_y(*ros_pc2, "y");
  shm_msgs::PointCloud2ConstIterator4m<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
  if (ros_pc2->fields_size == 3 || skip_colors == true) {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
      ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(
        Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  } else {
    o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
    if (shm_msgs::is_equal(ros_pc2->fields[3].name, "rgb")) {
      shm_msgs::PointCloud2ConstIterator4m<uint8_t> ros_pc2_r(*ros_pc2, "r");
      shm_msgs::PointCloud2ConstIterator4m<uint8_t> ros_pc2_g(*ros_pc2, "g");
      shm_msgs::PointCloud2ConstIterator4m<uint8_t> ros_pc2_b(*ros_pc2, "b");

      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x,
        ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(
            (static_cast<int>(*ros_pc2_r)) / 255.0,
            (static_cast<int>(*ros_pc2_g)) / 255.0,
            (static_cast<int>(*ros_pc2_b)) / 255.0));
      }
    } else if (shm_msgs::is_equal(ros_pc2->fields[3].name, "intensity")) {
      shm_msgs::PointCloud2ConstIterator4m<uint8_t> ros_pc2_i(*ros_pc2,
        "intensity");
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
        ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
      }
    }
  }
}
void rosToOpen3d(
  const shm_msgs::msg::PointCloud8m::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc, bool skip_colors)
{
  shm_msgs::PointCloud2ConstIterator8m<float> ros_pc2_x(*ros_pc2, "x");
  shm_msgs::PointCloud2ConstIterator8m<float> ros_pc2_y(*ros_pc2, "y");
  shm_msgs::PointCloud2ConstIterator8m<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
  if (ros_pc2->fields_size == 3 || skip_colors == true) {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
      ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(
        Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  } else {
    o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
    if (shm_msgs::is_equal(ros_pc2->fields[3].name, "rgb")) {
      shm_msgs::PointCloud2ConstIterator8m<uint8_t> ros_pc2_r(*ros_pc2, "r");
      shm_msgs::PointCloud2ConstIterator8m<uint8_t> ros_pc2_g(*ros_pc2, "g");
      shm_msgs::PointCloud2ConstIterator8m<uint8_t> ros_pc2_b(*ros_pc2, "b");

      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x,
        ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(
            (static_cast<int>(*ros_pc2_r)) / 255.0,
            (static_cast<int>(*ros_pc2_g)) / 255.0,
            (static_cast<int>(*ros_pc2_b)) / 255.0));
      }
    } else if (shm_msgs::is_equal(ros_pc2->fields[3].name, "intensity")) {
      shm_msgs::PointCloud2ConstIterator8m<uint8_t> ros_pc2_i(*ros_pc2,
        "intensity");
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
        ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
      }
    }
  }
}

}  // namespace shm_msgs
