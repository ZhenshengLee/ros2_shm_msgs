cmake_minimum_required(VERSION 3.5)

project(shm_msgs)

list(INSERT CMAKE_PREFIX_PATH 0 /opt/open3d/open3d)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

# only needed to build a separate iceoryx subscriber
# which is used to verify Shared Memory communication
find_package(iceoryx_posh CONFIG REQUIRED)

set(dependencies
  rclcpp
  rosidl_default_runtime
)

set(msg_files
  "msg/String.msg"
  "msg/Header.msg"
  "msg/Image.msg"
  "msg/CompressedImage.msg"
  "msg/PointField.msg"
  "msg/PointCloud1m.msg"
  "msg/PointCloud2m.msg"
  "msg/PointCloud4m.msg"
  "msg/PointCloud512k.msg"
  "msg/PointCloud8k.msg"
  "msg/PointCloud8m.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

# add_executable(talker
#   src/talker.cpp
# )

# ament_target_dependencies(talker
#   "rclcpp"
# )

# # we want to use the messages from the same package
# # would not be needed if we build the messages in a separate package
# rosidl_target_interfaces(talker
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install(TARGETS talker
#   DESTINATION lib/${PROJECT_NAME})

# add_executable(listener
#   src/listener.cpp
# )

# ament_target_dependencies(listener
#   "rclcpp"
# )

# rosidl_target_interfaces(listener
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install(TARGETS listener
#   DESTINATION lib/${PROJECT_NAME})

# # we build an iceoryx subscriber app that also listens to the topic
# add_executable(iox_subscriber
#   src/iox_subscriber.cpp
# )

# target_link_libraries(iox_subscriber
#     iceoryx_posh::iceoryx_posh
# )

# rosidl_target_interfaces(iox_subscriber
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install(TARGETS iox_subscriber
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  include_directories(include)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_image_encodings.cpp
    test/test_array_helper.cpp
    # test/test_image_fill.cpp
    test/test_pointcloud_iterator_8k.cpp
    test/test_open3d_conversions.cpp
    )
  if(TARGET test_${PROJECT_NAME})
    rosidl_target_interfaces(test_${PROJECT_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()
# endif()

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()
