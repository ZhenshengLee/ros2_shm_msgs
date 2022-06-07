/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2010-2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Open Source Robotics Foundation, Inc. nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SHM_MSGS__PCL_CONVERSIONS_H__
#define SHM_MSGS__PCL_CONVERSIONS_H__

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/message_event.h>
#include <message_filters/message_traits.h>

#include <shm_msgs/array_helper.hpp>

#include <pcl/PCLHeader.h>
#include <shm_msgs/msg/header.hpp>

// PCLImage is not supported
// #include <pcl/PCLImage.h>
// #include <shm_msgs/msg/image.hpp>

#include <pcl/PCLPointField.h>
#include <shm_msgs/msg/point_field.hpp>

#include <pcl/PCLPointCloud2.h>
#include <shm_msgs/msg/point_cloud2.hpp>

// use the default pcl_conversions

// #include <pcl/PointIndices.h>
// #include <pcl_msgs/msg/point_indices.hpp>

// #include <pcl/ModelCoefficients.h>
// #include <pcl_msgs/msg/model_coefficients.hpp>

// #include <pcl/Vertices.h>
// #include <pcl_msgs/msg/vertices.hpp>

// #include <pcl/PolygonMesh.h>
// #include <pcl_msgs/msg/polygon_mesh.hpp>

#include <pcl/io/pcd_io.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>

namespace shm_msgs {

  /** PCLHeader <=> Header **/

  inline
  void fromPCL(const std::uint64_t &pcl_stamp, rclcpp::Time &stamp)
  {
    stamp = rclcpp::Time(pcl_stamp * 1000ull); // Convert from us to ns
  }

  inline
  void toPCL(const rclcpp::Time &stamp, std::uint64_t &pcl_stamp)
  {
    pcl_stamp = stamp.nanoseconds() / 1000ull;  // Convert from ns to us
  }

  inline
  rclcpp::Time fromPCL(const std::uint64_t &pcl_stamp)
  {
    rclcpp::Time stamp;
    fromPCL(pcl_stamp, stamp);
    return stamp;
  }

  inline
  std::uint64_t toPCL(const rclcpp::Time &stamp)
  {
    std::uint64_t pcl_stamp;
    toPCL(stamp, pcl_stamp);
    return pcl_stamp;
  }

  /** PCLHeader <=> Header **/

  inline
  void fromPCL(const pcl::PCLHeader &pcl_header, shm_msgs::msg::Header &header)
  {
    header.stamp = fromPCL(pcl_header.stamp);
    shm_msgs::set_str(header.frame_id, pcl_header.frame_id);
  }

  inline
  void toPCL(const shm_msgs::msg::Header &header, pcl::PCLHeader &pcl_header)
  {
    toPCL(header.stamp, pcl_header.stamp);
    // TODO(clalancette): Seq doesn't exist in the ROS2 header
    // anymore.  wjwwood suggests that we might be able to get this
    // information from the middleware in the future, but for now we
    // just set it to 0.
    pcl_header.seq = 0;
    pcl_header.frame_id = shm_msgs::get_str(header.frame_id);
  }

  inline
  shm_msgs::msg::Header fromPCL(const pcl::PCLHeader &pcl_header)
  {
    shm_msgs::msg::Header header;
    fromPCL(pcl_header, header);
    return header;
  }

  inline
  pcl::PCLHeader toPCL(const shm_msgs::msg::Header &header)
  {
    pcl::PCLHeader pcl_header;
    toPCL(header, pcl_header);
    return pcl_header;
  }

  /** PCLImage <=> Image **/
  // PCLImage is not supported by shm_msgs

  /** PCLPointField <=> PointField **/

  inline
  void fromPCL(const pcl::PCLPointField &pcl_pf, shm_msgs::msg::PointField &pf)
  {
    shm_msgs::set_str(pf.name, pcl_pf.name);
    pf.offset = pcl_pf.offset;
    pf.datatype = pcl_pf.datatype;
    pf.count = pcl_pf.count;
  }

  // passing std::array
  // https://cplusplus.com/forum/general/184301/

  // template <size_t N>
  inline
  void fromPCL(const std::vector<pcl::PCLPointField> &pcl_pfs, std::array<shm_msgs::msg::PointField, 8> &pfs)
  {
    // pfs.resize(pcl_pfs.size());
    if(pcl_pfs.size() > 8)
    {
      std::stringstream ss;
      ss << "PointField is wrongly formed: actual size which is " << pcl_pfs.size() << " > " << 8
         << " which is the maximum size";
      throw std::runtime_error(ss.str());
    }
    std::vector<pcl::PCLPointField>::const_iterator it = pcl_pfs.begin();
    int i = 0;
    for(; it != pcl_pfs.end(); ++it, ++i) {
      fromPCL(*(it), pfs[i]);
    }
  }

  inline
  void toPCL(const shm_msgs::msg::PointField &pf, pcl::PCLPointField &pcl_pf)
  {
    pcl_pf.name = shm_msgs::get_str(pf.name);
    pcl_pf.offset = pf.offset;
    pcl_pf.datatype = pf.datatype;
    pcl_pf.count = pf.count;
  }

  /**
   * \brief     toPCL
   * \param    pfs input
   * \param    fields_size input
   * \param    pcl_pfs output
   */
  inline
  void toPCL(const std::array<shm_msgs::msg::PointField, 8> &pfs, const int& fields_size, std::vector<pcl::PCLPointField> &pcl_pfs)
  {
    pcl_pfs.resize(fields_size);
    std::array<shm_msgs::msg::PointField, 8>::const_iterator it = pfs.begin();
    int i = 0;
    for(; i != fields_size; ++it, ++i) {
      toPCL(*(it), pcl_pfs[i]);
    }
  }

  /** PCLPointCloud2 <=> PointCloud2 **/

  template<typename Msg>
  inline
  void copyPCLPointCloud2MetaData(const pcl::PCLPointCloud2 &pcl_pc2, Msg &pc2)
  {
    fromPCL(pcl_pc2.header, pc2.header);
    pc2.height = pcl_pc2.height;
    pc2.width = pcl_pc2.width;
    // added
    pc2.fields_size = pcl_pc2.fields.size();
    fromPCL(pcl_pc2.fields, pc2.fields);
    pc2.is_bigendian = pcl_pc2.is_bigendian;
    pc2.point_step = pcl_pc2.point_step;
    pc2.row_step = pcl_pc2.row_step;
    pc2.is_dense = pcl_pc2.is_dense;
  }

  // data between std::vector and std::array
  // https://stackoverflow.com/questions/21276889/copy-stdvector-into-stdarray
  template<typename Msg>
  inline
  void fromPCL(const pcl::PCLPointCloud2 &pcl_pc2, Msg &pc2)
  {
    copyPCLPointCloud2MetaData(pcl_pc2, pc2);
    if(pcl_pc2.data.size() > Msg::DATA_MAX_SIZE)
    {
      std::stringstream ss;
      ss << "PCLPointCloud2 is wrongly formed: actual size which is " << pcl_pc2.data.size() << " > " << Msg::DATA_MAX_SIZE
         << " which is the maximum size";
      throw std::runtime_error(ss.str());
    }
    // pc2.data = pcl_pc2.data;
    std::copy(pcl_pc2.data.begin(), pcl_pc2.data.end(), pc2.data.begin());
  }

  template<typename Msg>
  inline
  void moveFromPCL(pcl::PCLPointCloud2 &pcl_pc2, Msg &pc2)
  {
    copyPCLPointCloud2MetaData(pcl_pc2, pc2);
    if(pcl_pc2.data.size() > Msg::DATA_MAX_SIZE)
    {
      std::stringstream ss;
      ss << "PCLPointCloud2 is wrongly formed: actual size which is " << pcl_pc2.data.size() << " > " << Msg::DATA_MAX_SIZE
         << " which is the maximum size";
      throw std::runtime_error(ss.str());
    }
    // pc2.data.swap(pcl_pc2.data);
    std::move(pcl_pc2.data.begin(), pcl_pc2.data.end(), pc2.data.begin());
  }

  template<typename Msg>
  inline
  void copyPointCloud2MetaData(const Msg &pc2, pcl::PCLPointCloud2 &pcl_pc2)
  {
    toPCL(pc2.header, pcl_pc2.header);
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    toPCL(pc2.fields, pc2.fields_size, pcl_pc2.fields);
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.is_dense = pc2.is_dense;
  }

  template<typename Msg>
  inline
  void toPCL(const Msg &pc2, pcl::PCLPointCloud2 &pcl_pc2)
  {
    copyPointCloud2MetaData(pc2, pcl_pc2);
    // pcl_pc2.data = pc2.data;
    auto data_size = pcl_pc2.point_step * pcl_pc2.width * pcl_pc2.height;
    pcl_pc2.data.resize(data_size);
    std::copy(pc2.data.begin(), pc2.data.begin() + data_size, pcl_pc2.data.begin());
  }

  template<typename Msg>
  inline
  void moveToPCL(Msg &pc2, pcl::PCLPointCloud2 &pcl_pc2)
  {
    copyPointCloud2MetaData(pc2, pcl_pc2);
    // pcl_pc2.data.swap(pc2.data);
    auto data_size = pcl_pc2.point_step * pcl_pc2.width * pcl_pc2.height;
    pcl_pc2.data.resize(data_size);
    std::move(pc2.data.begin(), pc2.data.begin() + data_size, pcl_pc2.data.begin());
  }

  /** pcl::PointIndices <=> pcl_msgs::PointIndices **/
  /** pcl::ModelCoefficients <=> pcl_msgs::ModelCoefficients **/
  /** pcl::Vertices <=> pcl_msgs::Vertices **/

  namespace internal
  {
    template <typename T>
    inline void move(std::vector<T> &a, std::vector<T> &b)
    {
      b.swap(a);
    }

    template <typename T1, typename T2>
    inline void move(std::vector<T1> &a, std::vector<T2> &b)
    {
      b.assign(a.cbegin(), a.cend());
    }
  }

  /** pcl::PolygonMesh <=> pcl_msgs::PolygonMesh **/

} // namespace shm_msgs

namespace shm_msgs {

  /** Overload pcl::getFieldIndex **/

  template<typename Msg>
  inline int getFieldIndex(const Msg &cloud, const std::string &field_name)
  {
    // Get the index we need
    for (size_t d = 0; d < cloud.fields_size; ++d) {
      if (shm_msgs::get_str(cloud.fields[d].name) == field_name) {
        return (static_cast<int>(d));
      }
    }
    return (-1);
  }

  /** Overload pcl::getFieldsList **/

  template<typename Msg>
  inline std::string getFieldsList(const Msg &cloud)
  {
    std::string result;
    for (size_t i = 0; i < cloud.fields_size - 1; ++i) {
      result += shm_msgs::get_str(cloud.fields[i].name) + " ";
    }
    result += shm_msgs::get_str(cloud.fields[cloud.fields_size - 1].name);
    return (result);
  }

  /** Provide pcl::toROSMsg **/

  /** Provide to/fromROSMsg for shm_msgs::msg::PointCloud2 <=> pcl::PointCloud<T> **/

  template<typename T, typename Msg>
  void toROSMsg(const pcl::PointCloud<T> &pcl_cloud, Msg &cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
    shm_msgs::moveFromPCL(pcl_pc2, cloud);
  }

  template<typename T, typename Msg>
  void fromROSMsg(const Msg &cloud, pcl::PointCloud<T> &pcl_cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    shm_msgs::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
  }

  template<typename T, typename Msg>
  void moveFromROSMsg(Msg &cloud, pcl::PointCloud<T> &pcl_cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    shm_msgs::moveToPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
  }

  /** Overload pcl::createMapping **/
  // currently not supported
  // template<typename PointT>
  // void createMapping(const std::vector<shm_msgs::msg::PointField>& msg_fields, MsgFieldMap& field_map)
  // {
  //   std::vector<pcl::PCLPointField> pcl_msg_fields;
  //   shm_msgs::toPCL(msg_fields, pcl_msg_fields);
  //   return createMapping<PointT>(pcl_msg_fields, field_map);
  // }

  namespace io {

    /** Overload pcl::io::savePCDFile **/

    template<typename Msg>
    inline int
    savePCDFile(const std::string &file_name, const Msg &cloud,
                const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
                const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                const bool binary_mode = false)
    {
      pcl::PCLPointCloud2 pcl_cloud;
      shm_msgs::toPCL(cloud, pcl_cloud);
      return pcl::io::savePCDFile(file_name, pcl_cloud, origin, orientation, binary_mode);
    }

    template<typename Msg>
    inline int
    destructiveSavePCDFile(const std::string &file_name, Msg &cloud,
                           const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
                           const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                           const bool binary_mode = false)
    {
      pcl::PCLPointCloud2 pcl_cloud;
      shm_msgs::moveToPCL(cloud, pcl_cloud);
      return pcl::io::savePCDFile(file_name, pcl_cloud, origin, orientation, binary_mode);
    }

    /** Overload pcl::io::loadPCDFile **/

    template<typename Msg>
    inline int loadPCDFile(const std::string &file_name, Msg &cloud)
    {
      pcl::PCLPointCloud2 pcl_cloud;
      int ret = pcl::io::loadPCDFile(file_name, pcl_cloud);
      shm_msgs::moveFromPCL(pcl_cloud, cloud);
      return ret;
    }

  } // namespace io

  // https://stackoverflow.com/questions/57507876/what-are-the-contents-of-pointcloud2
  // header:
  //   seq: 1071
  //   stamp:
  //     secs: 1521699326
  //     nsecs: 676390000
  //   frame_id: "velodyne"
  // height: 1
  // width: 66811
  // fields:
  //   -
  //     name: "x"
  //     offset: 0
  //     datatype: 7
  //     count: 1
  //   -
  //     name: "y"
  //     offset: 4
  //     datatype: 7
  //     count: 1
  //   -
  //     name: "z"
  //     offset: 8
  //     datatype: 7
  //     count: 1
  //   -
  //     name: "intensity"
  //     offset: 16
  //     datatype: 7
  //     count: 1
  //   -
  //     name: "ring"
  //     offset: 20
  //     datatype: 4
  //     count: 1
  // is_bigendian: False
  // point_step: 32
  // row_step: 2137952
  // data: [235, 171, 54, 190, 53, 107, 250, ...

  // the actual point data size is (row_step*height) or (point_step*width*height)
  // meaning row_step is (point_step*width)

  /** Overload asdf **/

  // currently not supported

  // inline
  // bool concatenatePointCloud (const sensor_msgs::msg::PointCloud2 &cloud1,
  //                             const sensor_msgs::msg::PointCloud2 &cloud2,
  //                             sensor_msgs::msg::PointCloud2 &cloud_out)
  // {
  //   //if one input cloud has no points, but the other input does, just return the cloud with points
  //   if (cloud1.width * cloud1.height == 0 && cloud2.width * cloud2.height > 0)
  //   {
  //     cloud_out = cloud2;
  //     return (true);
  //   }
  //   else if (cloud1.width*cloud1.height > 0 && cloud2.width*cloud2.height == 0)
  //   {
  //     cloud_out = cloud1;
  //     return (true);
  //   }

  //   bool strip = false;
  //   for (size_t i = 0; i < cloud1.fields.size (); ++i)
  //     if (cloud1.fields[i].name == "_")
  //       strip = true;

  //   for (size_t i = 0; i < cloud2.fields.size (); ++i)
  //     if (cloud2.fields[i].name == "_")
  //       strip = true;

  //   if (!strip && cloud1.fields.size () != cloud2.fields.size ())
  //   {
  //     PCL_ERROR ("[pcl::concatenatePointCloud] Number of fields in cloud1 (%u) != Number of fields in cloud2 (%u)\n", cloud1.fields.size (), cloud2.fields.size ());
  //     return (false);
  //   }

  //   // Copy cloud1 into cloud_out
  //   cloud_out = cloud1;
  //   size_t nrpts = cloud_out.data.size ();
  //   // Height = 1 => no more organized
  //   cloud_out.width    = cloud1.width * cloud1.height + cloud2.width * cloud2.height;
  //   cloud_out.height   = 1;
  //   if (!cloud1.is_dense || !cloud2.is_dense)
  //     cloud_out.is_dense = false;
  //   else
  //     cloud_out.is_dense = true;

  //   // We need to strip the extra padding fields
  //   if (strip)
  //   {
  //     // Get the field sizes for the second cloud
  //     std::vector<sensor_msgs::msg::PointField> fields2;
  //     std::vector<int> fields2_sizes;
  //     for (size_t j = 0; j < cloud2.fields.size (); ++j)
  //     {
  //       if (cloud2.fields[j].name == "_")
  //         continue;

  //       fields2_sizes.push_back (cloud2.fields[j].count *
  //                                pcl::getFieldSize (cloud2.fields[j].datatype));
  //       fields2.push_back (cloud2.fields[j]);
  //     }

  //     cloud_out.data.resize (nrpts + (cloud2.width * cloud2.height) * cloud_out.point_step);

  //     // Copy the second cloud
  //     for (size_t cp = 0; cp < cloud2.width * cloud2.height; ++cp)
  //     {
  //       int i = 0;
  //       for (size_t j = 0; j < fields2.size (); ++j)
  //       {
  //         if (cloud1.fields[i].name == "_")
  //         {
  //           ++i;
  //           continue;
  //         }

  //         // We're fine with the special RGB vs RGBA use case
  //         if ((cloud1.fields[i].name == "rgb" && fields2[j].name == "rgba") ||
  //             (cloud1.fields[i].name == "rgba" && fields2[j].name == "rgb") ||
  //             (cloud1.fields[i].name == fields2[j].name))
  //         {
  //           memcpy (reinterpret_cast<char*> (&cloud_out.data[nrpts + cp * cloud1.point_step + cloud1.fields[i].offset]),
  //                   reinterpret_cast<const char*> (&cloud2.data[cp * cloud2.point_step + cloud2.fields[j].offset]),
  //                   fields2_sizes[j]);
  //           ++i;  // increment the field size i
  //         }
  //       }
  //     }
  //   }
  //   else
  //   {
  //     for (size_t i = 0; i < cloud1.fields.size (); ++i)
  //     {
  //       // We're fine with the special RGB vs RGBA use case
  //       if ((cloud1.fields[i].name == "rgb" && cloud2.fields[i].name == "rgba") ||
  //           (cloud1.fields[i].name == "rgba" && cloud2.fields[i].name == "rgb"))
  //         continue;
  //       // Otherwise we need to make sure the names are the same
  //       if (cloud1.fields[i].name != cloud2.fields[i].name)
  //       {
  //         PCL_ERROR ("[pcl::concatenatePointCloud] Name of field %d in cloud1, %s, does not match name in cloud2, %s\n", i, cloud1.fields[i].name.c_str (), cloud2.fields[i].name.c_str ());
  //         return (false);
  //       }
  //     }
  //     cloud_out.data.resize (nrpts + cloud2.data.size ());
  //     memcpy (&cloud_out.data[nrpts], &cloud2.data[0], cloud2.data.size ());
  //   }
  //   return (true);
  // }

} // namespace shm_msgs

#endif /* SHM_MSGS__PCL_CONVERSIONS_H__ */
