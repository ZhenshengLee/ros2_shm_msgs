# shm_msgs

Zero copy can save times of copy in IPC, thus reduces cpu usage and transport latency, which can be used in real-time-critical systems or resource-constrained computing platforms.

This package provides many ros2 message definitions that have a good support to true zero copy transportation in IPC(in a single machine) context.

Currently tested in ros2 galactic with cyclonedds+iceoryx dds layer.

## supported msgs

- [x] pointcloud2
- [ ] image

## software components

this package includes ros2 msg definitions and demos that supports the msgs.

- shm_msgs::msg::PointCloud8k
- PointCloud2Modifier8k
- open3d_conversions
- pcl_conversions
- vision_opencv

### modifier

## about (true) zero copy

Zero_copy is a transport layer to get a better performance especially when payload size exceeds 64k.

Please refer [Using_Zero_Copy_In_ROS2.pdf](./doc/Using_Zero_Copy_In_ROS2.pdf) for more info of concept of z copy.

In short, zero copy needs api support through all transport layers. See [doc from cyclonedds] for details

- fixed-length msg from user-space(enable z copy)
- rclcpp z_copy api(enable z copy)
- rmw z copy api(serialization support)
- dds qos support(iceoryx policy alignment)
- dds z_copy api(iceoryx api)

This package provides fixed-length ros2 msg definitions, helper functions and demos to support true z copy.

## about (minimum) zero copy

Without fixed-length msg, a serialization is needed before IPC, so copy can not be avoided.

But time of copy can still be reduced comparing to Loopback Network Communication(such as udp multicast), which is the default transport layer in most cases of dds.

Performance can still be improved with minimum zero copy.

## related work

The [eCAL](https://continental.github.io/ecal/) by Continental provides iceoryx transport layer to get true zero copy, but extra cmake based compilation is needed.

refer [The performance of eCAL](https://continental.github.io/ecal/advanced/performance.html), and [Enable ECAL_LAYER_ICEORYX](https://continental.github.io/ecal/development/ecal_cmake_options.html) for more details.

eCAL also provides default shm based transport layer to get minimum zero_copy performance.

Cyclonedds and eCAL both use iceoryx as the z copy transport layer.

Iceoryx alos provide direct access to rmw, see [rmw_iceoryx](https://github.com/ros2/rmw_iceoryx) for details.

## acknowledgement

[Iceoryx]
[Apex.AI]
[CycloneDDS]
[eCAL]
