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
  ament_target_dependencies(${SHM_IMGAE_BRIDGE_EXE} "rclcpp" "cv_bridge")
  target_link_libraries(${SHM_IMGAE_BRIDGE_EXE}  ${PROJECT_NAME}_image
      opencv_highgui
  )
  rosidl_target_interfaces(${SHM_IMGAE_BRIDGE_EXE} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${SHM_IMGAE_BRIDGE_EXE} DESTINATION lib/${PROJECT_NAME})
endforeach()

