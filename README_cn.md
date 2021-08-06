# ros2_shm_msgs

for English please refer[English](./README.md)

零拷贝机制可以节省IPC过程中的拷贝次数，可以降低cpu占用率和传输时延，对于实时关键系统和资源受限的计算平台很有用。

本包提供支持零拷贝机制的ros2消息定义，注意零拷贝机制是在单机IPC上使用的

当前已经在ros2 galactic + cyclonedds + iceoryx中测试通过

关于分布式系统的性能请阅读[理论基础](./doc/design/performance_of_distributed_sys.md)

## 开发计划

本包计划支持pointcloud 和 image两种数据类型。

| feature               | Status                             |
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

## 示例

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

## 软件组件

this package includes ros2 msg definitions and demos that supports the msgs.

- shm_msgs::msg::PointCloud8k
- PointCloud2Modifier8k
- open3d_conversions
- pcl_conversions
- vision_opencv
- rviz-bridge

## 关于真零拷贝

在ros2的使用表明，当传输数据量大于64k时，零拷贝可以提供显著的性能提升。

关于零拷贝的概念请参考APEX的PPT [Using_Zero_Copy_In_ROS2.pdf](./doc/Using_Zero_Copy_In_ROS2.pdf)

零拷贝机制需要自顶向下所有软件层的支持，参考[doc from cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/shared_memory.rst)和[doc from rmw_cyclonedds](https://github.com/ros2/rmw_cyclonedds/blob/master/shared_memory_support.md)

具体而言，下列api均需要支持零拷贝

- 用户使用固定大小的消息数据类型
- rclcpp的零拷贝api支持
- rmw的零拷贝api支持
- dds qos对 iceoryx的支持
- dds 的零拷贝api支持

本包提供的是用户态的固定大小消息定义，以及用户态的demo支持函数。

## 关于最少拷贝

当没有固定大小数据类型时，IPC需要序列化和反序列化操作，所以数据拷贝不可避免

但是与传统dds基于Loopback的网络通信（比如udp 组播）相比，数据拷贝的次数依然有减少。

所以在最少拷贝的传输方式下，性能依然能够得到提升。

## 相关工作

大陆集团的中间件 [eCAL](https://continental.github.io/ecal/) 使用了iceoryx制作了零拷贝数据传输通道，不过需要从源代码编译，增加额外的编译选项。

性能请参考 [The performance of eCAL](https://continental.github.io/ecal/advanced/performance.html), 编译方式和选项请看 [Enable ECAL_LAYER_ICEORYX](https://continental.github.io/ecal/development/ecal_cmake_options.html)

eCAL 也提供了自己的基于共享内存的传输通道，虽然不能得到零拷贝，但是可以实现最少拷贝，其性能可以查看[this issue](https://github.com/continental/ecal/issues/326)

Cyclonedds和eCAL都使用iceoryx作为零拷贝的传输通道

Iceoryx同样可以提供rmw的接口，查看[rmw_iceoryx](https://github.com/ros2/rmw_iceoryx)

## 讨论

iceoryx and ros2 社区讨论了该库的实现方式和成果。

- [2021 08 05 Eclipse iceoryx developer meetup](https://github.com/eclipse-iceoryx/iceoryx/wiki/2021-08-05-Eclipse-iceoryx-developer-meetup)
- [ros discourse about z copy with cycloedds and iceoryx](https://discourse.ros.org/t/talk-usingzero-copy-data-transfer-in-ros-2/21448/13)
- [autoware.auto issue: Enable zero-copy for pointcloud processing](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1096)

欢迎在本库提交issues。

## 鸣谢

[Iceoryx](https://iceoryx.io/)
[Apex.AI](http://apex.ai/)
[CycloneDDS](https://www.adlinktech.com/en/CycloneDDS)
[eCAL](http://ecal.io/)
