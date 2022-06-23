find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
include_directories(include)

ament_add_gtest(test_${PROJECT_NAME}_basic test_array_helper.cpp)
target_link_libraries(test_${PROJECT_NAME}_basic)
rosidl_target_interfaces(test_${PROJECT_NAME}_basic ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS test_${PROJECT_NAME}_basic DESTINATION test/${PROJECT_NAME})

ament_add_gtest(test_${PROJECT_NAME}_pc
  test_pointcloud_iterator_8k.cpp
  test_pointcloud_iterator_512k.cpp
  test_pointcloud_iterator_1m.cpp
  test_pointcloud_iterator_2m.cpp
  test_pointcloud_iterator_4m.cpp
  test_pointcloud_iterator_8m.cpp
)
rosidl_target_interfaces(test_${PROJECT_NAME}_pc ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS test_${PROJECT_NAME}_pc DESTINATION test/${PROJECT_NAME})

if(Open3D_FOUND)
  ament_add_gtest(test_${PROJECT_NAME}_open3d test_open3d_conversions.cpp)
  target_link_libraries(test_${PROJECT_NAME}_open3d ${PROJECT_NAME}_open3d)
  rosidl_target_interfaces(test_${PROJECT_NAME}_open3d ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS test_${PROJECT_NAME}_open3d DESTINATION test/${PROJECT_NAME})
endif()

ament_add_gtest(test_${PROJECT_NAME}_image
  test_image_encodings.cpp
  test_image_fill.cpp
  test_rgb_colors.cpp
  test_opencv_conversions1.cpp
  test_opencv_conversions2.cpp
  test_endian.cpp
)
target_link_libraries(test_${PROJECT_NAME}_image ${PROJECT_NAME}_image)
rosidl_target_interfaces(test_${PROJECT_NAME}_image ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS test_${PROJECT_NAME}_image DESTINATION test/${PROJECT_NAME})

ament_add_gtest(test_${PROJECT_NAME}_pcl
  test_pcl_conversions.cpp
)
target_link_libraries(test_${PROJECT_NAME}_pcl ${PROJECT_NAME}_pcl)
ament_target_dependencies(test_${PROJECT_NAME}_pcl "rclcpp")
rosidl_target_interfaces(test_${PROJECT_NAME}_pcl ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS test_${PROJECT_NAME}_pcl DESTINATION test/${PROJECT_NAME})
