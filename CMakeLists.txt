cmake_minimum_required(VERSION 3.5)

project(shm_msgs)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

# add installation directory here
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/open3d/open3d)
list(INSERT CMAKE_PREFIX_PATH 0 /opt/opencv/opencv-4.6.0/)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D)
find_package(Boost QUIET)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_msgs REQUIRED)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED)
endif()

find_package(iceoryx_posh CONFIG REQUIRED)
find_package(fastrtps CONFIG REQUIRED)

set(dependencies
  rclcpp
  rosidl_default_runtime
  cv_bridge
  message_filters
  pcl_msgs
  sensor_msgs
)

set(msg_files
  "msg/String.msg"
  "msg/Header.msg"
  "msg/Image8k.msg"
  "msg/Image512k.msg"
  "msg/Image1m.msg"
  "msg/Image2m.msg"
  "msg/Image4m.msg"
  "msg/Image8m.msg"
  "msg/PointField.msg"
  "msg/PointCloud8k.msg"
  "msg/PointCloud512k.msg"
  "msg/PointCloud1m.msg"
  "msg/PointCloud2m.msg"
  "msg/PointCloud4m.msg"
  "msg/PointCloud8m.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

if(cpp_typesupport_target)
  add_library(${PROJECT_NAME}_library INTERFACE)
  target_include_directories(${PROJECT_NAME}_library INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${PROJECT_NAME}_library INTERFACE
  "${cpp_typesupport_target}")

  install(
    TARGETS ${PROJECT_NAME}_library EXPORT export_${PROJECT_NAME}
  )
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_COMMON_INCLUDE_DIRS}
)

add_subdirectory(lib)

add_subdirectory(src)
# extra image intra-process demos
add_subdirectory(intra)
# extra type-adaption cuda demos
add_subdirectory(cuda)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(tool)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY res
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
