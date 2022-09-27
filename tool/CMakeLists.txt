set(SHM_IMGAE_BRIDGE_FILES
  shm_image8k_bridge
  shm_image512k_bridge
  shm_image1m_bridge
  shm_image2m_bridge
  shm_image4m_bridge
  shm_image8m_bridge
)

foreach(SHM_IMGAE_BRIDGE_EXE ${SHM_IMGAE_BRIDGE_FILES})
  add_executable(${SHM_IMGAE_BRIDGE_EXE} ${SHM_IMGAE_BRIDGE_EXE}.cpp)
  target_include_directories(${SHM_IMGAE_BRIDGE_EXE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  ament_target_dependencies(${SHM_IMGAE_BRIDGE_EXE} "rclcpp" "cv_bridge" "sensor_msgs")
  target_link_libraries(${SHM_IMGAE_BRIDGE_EXE}  ${PROJECT_NAME}_image
      opencv_highgui
      ${cpp_typesupport_target}
  )
  install(TARGETS ${SHM_IMGAE_BRIDGE_EXE} DESTINATION lib/${PROJECT_NAME})
endforeach()

set(SHM_PCL_BRIDGE_FILES
  shm_pcl8k_bridge
  shm_pcl512k_bridge
  shm_pcl1m_bridge
  shm_pcl2m_bridge
  shm_pcl4m_bridge
  shm_pcl8m_bridge
)

foreach(SHM_PCL_BRIDGE_EXE ${SHM_PCL_BRIDGE_FILES})
  add_executable(${SHM_PCL_BRIDGE_EXE} ${SHM_PCL_BRIDGE_EXE}.cpp)
  target_include_directories(${SHM_PCL_BRIDGE_EXE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  ament_target_dependencies(${SHM_PCL_BRIDGE_EXE} "rclcpp" "pcl_conversions" "sensor_msgs")
  target_link_libraries(${SHM_PCL_BRIDGE_EXE}  ${PROJECT_NAME}_image
      ${PCL_LIBRARIES}
      ${cpp_typesupport_target}
  )
  install(TARGETS ${SHM_PCL_BRIDGE_EXE} DESTINATION lib/${PROJECT_NAME})
endforeach()
