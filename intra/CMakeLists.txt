set(INTRA_IMAGE_FILES
  intra_str_node
  intra_int_node
  intra_image_node
)

foreach(INTRA_IMAGE_EXE ${INTRA_IMAGE_FILES})
  add_executable(${INTRA_IMAGE_EXE} ${INTRA_IMAGE_EXE}.cpp)
  ament_target_dependencies(${INTRA_IMAGE_EXE} "rclcpp" "cv_bridge" "sensor_msgs")
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(${INTRA_IMAGE_EXE}  ${PROJECT_NAME}_image
      opencv_highgui
      ${cpp_typesupport_target}
  )
  install(TARGETS ${INTRA_IMAGE_EXE} DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(mat_image_node mat_image_node.cpp mat_image_container.cpp)
ament_target_dependencies(mat_image_node "rclcpp" "cv_bridge" "sensor_msgs")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(mat_image_node  ${PROJECT_NAME}_image
    opencv_highgui
    ${cpp_typesupport_target}
)
install(TARGETS mat_image_node DESTINATION lib/${PROJECT_NAME})
