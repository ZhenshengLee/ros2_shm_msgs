if(Open3D_FOUND)
    add_executable(open3d2m_talker open3d2m_talker.cpp)
    ament_target_dependencies(open3d2m_talker "rclcpp")
    target_link_libraries(open3d2m_talker  ${PROJECT_NAME}_open3d ${Open3D_LIBRARIES})
    rosidl_target_interfaces(open3d2m_talker ${PROJECT_NAME} "rosidl_typesupport_cpp")
    install(TARGETS open3d2m_talker DESTINATION lib/${PROJECT_NAME})

    add_executable(open3d2m_listener open3d2m_listener.cpp)
    ament_target_dependencies(open3d2m_listener "rclcpp")
    target_link_libraries(open3d2m_listener  ${PROJECT_NAME}_open3d ${Open3D_LIBRARIES})
    rosidl_target_interfaces(open3d2m_listener ${PROJECT_NAME} "rosidl_typesupport_cpp")
    install(TARGETS open3d2m_listener DESTINATION lib/${PROJECT_NAME})
endif()

set(SHM_IMAGE_FILES
  image_talker
  image1m_talker
  image2m_talker
  image_listener
  image1m_listener
  image2m_listener
)

foreach(SHM_IMAGE_EXE ${SHM_IMAGE_FILES})
  add_executable(${SHM_IMAGE_EXE} ${SHM_IMAGE_EXE}.cpp)
  ament_target_dependencies(${SHM_IMAGE_EXE} "rclcpp" "cv_bridge" "sensor_msgs")
  target_link_libraries(${SHM_IMAGE_EXE}  ${PROJECT_NAME}_image
      opencv_highgui
  )
  rosidl_target_interfaces(${SHM_IMAGE_EXE} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${SHM_IMAGE_EXE} DESTINATION lib/${PROJECT_NAME})
endforeach()

set(SHM_PCL_FILES
  pcl_talker
  pcl8k_talker
  pcl1m_talker
  pcl2m_talker
  pcl_listener
  pcl8k_listener
  pcl1m_listener
  pcl2m_listener
)

foreach(SHM_PCL_EXE ${SHM_PCL_FILES})
  add_executable(${SHM_PCL_EXE} ${SHM_PCL_EXE}.cpp)
  ament_target_dependencies(${SHM_PCL_EXE} "rclcpp" "sensor_msgs" "pcl_conversions")
  target_link_libraries(${SHM_PCL_EXE}  ${PROJECT_NAME}_pcl
    ${PCL_LIBRARIES}
  )
  rosidl_target_interfaces(${SHM_PCL_EXE} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${SHM_PCL_EXE} DESTINATION lib/${PROJECT_NAME})
endforeach()
