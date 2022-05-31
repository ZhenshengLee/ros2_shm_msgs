if(Open3D_FOUND)
    add_executable(open3d_talker open3d_talker.cpp)
    ament_target_dependencies(open3d_talker "rclcpp")
    target_link_libraries(open3d_talker  ${PROJECT_NAME}_open3d ${Open3D_LIBRARIES})
    rosidl_target_interfaces(open3d_talker ${PROJECT_NAME} "rosidl_typesupport_cpp")
    install(TARGETS open3d_talker DESTINATION lib/${PROJECT_NAME})

    add_executable(open3d_listener open3d_listener.cpp)
    ament_target_dependencies(open3d_listener "rclcpp")
    target_link_libraries(open3d_listener  ${PROJECT_NAME}_open3d ${Open3D_LIBRARIES})
    rosidl_target_interfaces(open3d_listener ${PROJECT_NAME} "rosidl_typesupport_cpp")
    install(TARGETS open3d_listener DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(image_talker image_talker.cpp)
ament_target_dependencies(image_talker "rclcpp")
target_link_libraries(image_talker  ${PROJECT_NAME}_image
    opencv_highgui
)
rosidl_target_interfaces(image_talker ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS image_talker DESTINATION lib/${PROJECT_NAME})

add_executable(image_listener image_listener.cpp)
ament_target_dependencies(image_listener "rclcpp")
target_link_libraries(image_listener  ${PROJECT_NAME}_image
    opencv_highgui
)
rosidl_target_interfaces(image_listener ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS image_listener DESTINATION lib/${PROJECT_NAME})
