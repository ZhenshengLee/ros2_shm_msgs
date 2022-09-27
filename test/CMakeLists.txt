find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
include_directories(include)

ament_add_gtest(test_${PROJECT_NAME}_basic test_array_helper.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(test_${PROJECT_NAME}_basic
  ${cpp_typesupport_target}
)
ament_target_dependencies(test_${PROJECT_NAME}_basic "rclcpp" std_msgs)
install(TARGETS test_${PROJECT_NAME}_basic DESTINATION test/${PROJECT_NAME})

ament_add_gtest(test_${PROJECT_NAME}_pc
  test_pointcloud_iterator_8k.cpp
  test_pointcloud_iterator_512k.cpp
  test_pointcloud_iterator_1m.cpp
  test_pointcloud_iterator_2m.cpp
  test_pointcloud_iterator_4m.cpp
  test_pointcloud_iterator_8m.cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(test_${PROJECT_NAME}_pc
  ${cpp_typesupport_target}
)
ament_target_dependencies(test_${PROJECT_NAME}_pc "rclcpp" sensor_msgs)
install(TARGETS test_${PROJECT_NAME}_pc DESTINATION test/${PROJECT_NAME})

if(Open3D_FOUND)
  ament_add_gtest(test_${PROJECT_NAME}_open3d test_open3d_conversions.cpp)
  target_link_libraries(test_${PROJECT_NAME}_open3d ${PROJECT_NAME}_open3d)
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_${PROJECT_NAME}_open3d
    ${cpp_typesupport_target}
  )
  ament_target_dependencies(test_${PROJECT_NAME}_open3d "rclcpp" sensor_msgs)
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
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(test_${PROJECT_NAME}_image
  ${cpp_typesupport_target}
)
ament_target_dependencies(test_${PROJECT_NAME}_image "rclcpp" sensor_msgs)
install(TARGETS test_${PROJECT_NAME}_image DESTINATION test/${PROJECT_NAME})

ament_add_gtest(test_${PROJECT_NAME}_pcl
  test_pcl_conversions.cpp
)
target_link_libraries(test_${PROJECT_NAME}_pcl ${PROJECT_NAME}_pcl)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(test_${PROJECT_NAME}_pcl
  ${cpp_typesupport_target}
)
ament_target_dependencies(test_${PROJECT_NAME}_pcl "rclcpp" sensor_msgs)
install(TARGETS test_${PROJECT_NAME}_pcl DESTINATION test/${PROJECT_NAME})
