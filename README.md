# ros2_shm_msgs

fou Chinese reader please refer [简体中文](./README_cn.md)

Zero copy can save times of copy in IPC, thus reduces cpu usage and transport latency, which can be used in real-time-critical systems or resource-constrained computing platforms.

This package provides many ros2 message definitions that have a good support to true zero copy transportation in IPC(in a single machine) context.

Currently tested in ros2 galactic with cyclonedds+iceoryx dds layer.

## todolist

pointcloud and image are currently supported.

|  feature              | Status                             |
|-----------------------|------------------------------------|
| pointcloud8k          | :heavy_check_mark:                 |
| pointcloud256k        | :x: (coming)                       |
| pointcloud1m          | :x: (coming)                       |
| pointcloud2m          | :x: (coming)                       |
| pointcloud4m          | :x: (coming)                       |
| pointcloud8m          | :x: (coming)                       |
| image8k               | :x:                                |
| image256k             | :x:                                |
| image512k             | :x:                                |
| image1m               | :x:                                |
| image2m               | :x:                                |
| image4m               | :x:                                |
| image8m               | :x:                                |
| pointcloud-rviz bridge| :x: (comming)                      |
| image-rviz bridge     | :x: (comming)                      |

## examples

```sh
# t1
iox-roudi

# t2
cd ./install/shm_msgs/lib/shm_msgs/
./pc_talker

# t3
cd ./install/shm_msgs/lib/shm_msgs/
./pc_listener
```

## software components

this package includes ros2 msg definitions and demos that supports the msgs.

- shm_msgs::msg::PointCloud8k
- PointCloud2Modifier8k
- open3d_conversions
- pcl_conversions
- vision_opencv
- rviz-bridge

## true zero copy

Zero_copy is a transport layer to get a better performance especially when payload size exceeds 64k.

Please refer [Using_Zero_Copy_In_ROS2.pdf](./doc/Using_Zero_Copy_In_ROS2.pdf) for more info of concept of z copy.

In short, zero copy needs api support through all transport layers. See [doc from cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/shared_memory.rst) and [doc from rmw_cyclonedds](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md) for details

- fixed-length msg from user-space(enable z copy)
- rclcpp z_copy api(enable z copy)
- rmw z copy api(serialization support)
- dds qos support(iceoryx policy alignment)
- dds z_copy api(iceoryx api)

This package provides fixed-length ros2 msg definitions, helper functions and demos to support true z copy.

## minimum copy

Without fixed-length msg, a serialization is needed before IPC, so copy can not be avoided.

But time of copy can still be reduced comparing to Loopback Network Communication(such as udp multicast), which is the default transport layer in most cases of dds.

Performance can still be improved with minimum copy.

## related work

The [eCAL](https://continental.github.io/ecal/) by Continental provides iceoryx transport layer to get true zero copy, but extra cmake based compilation is needed.

refer [The performance of eCAL](https://continental.github.io/ecal/advanced/performance.html), and [Enable ECAL_LAYER_ICEORYX](https://continental.github.io/ecal/development/ecal_cmake_options.html) for more details.

eCAL also provides default shm based transport layer to get minimum copy performance, see [this issue](https://github.com/continental/ecal/issues/326) for details.

Cyclonedds and eCAL both use iceoryx as the z copy transport layer.

Iceoryx alos provide direct access to rmw, see [rmw_iceoryx](https://github.com/ros2/rmw_iceoryx) for details.

## discussions

Comments have been given from iceoryx and ros2 community.

- [2021 08 05 Eclipse iceoryx developer meetup](https://github.com/eclipse-iceoryx/iceoryx/wiki/2021-08-05-Eclipse-iceoryx-developer-meetup)
- [ros discourse about z copy with cycloedds and iceoryx](https://discourse.ros.org/t/talk-usingzero-copy-data-transfer-in-ros-2/21448/13)
- [autoware.auto issue: Enable zero-copy for pointcloud processing](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1096)

Feel free to create issues in the repo.

## acknowledgement

[Iceoryx](https://iceoryx.io/)
[Apex.AI](http://apex.ai/)
[CycloneDDS](https://www.adlinktech.com/en/CycloneDDS)
[eCAL](http://ecal.io/)
