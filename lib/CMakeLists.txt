# you should not define a method in a header, unless:
# It is marked inline
# It is a function template
# It is a member function

add_library(${PROJECT_NAME}_open3d
    ./open3d_conversions.cpp
)
target_include_directories(${PROJECT_NAME}_open3d PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Open3D_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_open3d
    ${Open3D_LIBRARIES}
)
rosidl_target_interfaces(${PROJECT_NAME}_open3d
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS ${PROJECT_NAME}_open3d DESTINATION lib/${PROJECT_NAME})

add_library(${PROJECT_NAME}_image
    ./opencv_conversions.cpp
    ./rgb_colors.cpp
)
target_include_directories(${PROJECT_NAME}_image PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )
target_link_libraries(${PROJECT_NAME}_image
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
)
target_link_libraries(${PROJECT_NAME}_image
    Boost::headers
    rcpputils::rcpputils
)
rosidl_target_interfaces(${PROJECT_NAME}_image
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS ${PROJECT_NAME}_image DESTINATION lib/${PROJECT_NAME})