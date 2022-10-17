macro(GA_CHECK_CUDA)
  if ($ENV{GPUAC_COMPILE_WITH_CUDA})
    find_package(CUDA REQUIRED  QUIET)
    find_package(Eigen3 REQUIRED  QUIET)

    # if(NOT ${CUDA_VERSION} VERSION_LESS "10.0"
    #         AND NOT ${CUDA_VERSION} VERSION_EQUAL "10.0" )
    #   message(FATAL_ERROR "GPU support on Melodic requires CUDA<=10.0")
    # endif()
    if(${CUDA_VERSION} VERSION_GREATER "9.1"
          AND ${CMAKE_VERSION} VERSION_LESS "3.12.3")
      unset(CUDA_cublas_device_LIBRARY CACHE)
      set(CUDA_cublas_device_LIBRARY ${CUDA_cublas_LIBRARY})
      set(CUDA_CUBLAS_LIBRARIES ${CUDA_cublas_LIBRARY})
    endif()
    # if ("$ENV{ROS_DISTRO}" STREQUAL "melodic" AND ${EIGEN3_VERSION_STRING} VERSION_LESS "3.3.7")
    #   message(FATAL_ERROR "GPU support on Melodic requires Eigen version>= 3.3.7")
    # endif()
    if(NOT DEFINED CMAKE_CUDA_STANDARD)
        set(CMAKE_CUDA_STANDARD 14)
        set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    endif()
    set(USE_CUDA ON)
  else()
    message(WARNING "CUDA support is disabled. Set the GPUAC_COMPILE_WITH_CUDA environment variable and recompile to enable it")
    set(USE_CUDA OFF)
  endif()
endmacro()

include(CudaComputeTargetFlags)
GA_CHECK_CUDA()
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")
APPEND_TARGET_ARCH_FLAGS()

if(NOT USE_CUDA)
 return()
endif()

# Enable NVTX markers for improved profiling
add_definitions(-DUSE_NVTX)
link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
link_libraries("nvToolsExt")

# nvcc -arch sm_50 ./cuda_hello_world.cu -o docker_hello
cuda_add_executable(cuda_hello_world cuda_hello_world.cu)
target_include_directories(cuda_hello_world PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(cuda_hello_world  ${PROJECT_NAME}_image
    ${CUDA_nvToolsExt_LIBRARY}
    ${CUDA_LIBRARIES}
)
install(TARGETS cuda_hello_world DESTINATION lib/${PROJECT_NAME})

cuda_add_executable(cuda_image_node cuda_image_node.cpp cuda_image_container.cpp)
ament_target_dependencies(cuda_image_node "rclcpp" "cv_bridge" "sensor_msgs")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_include_directories(cuda_image_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(cuda_image_node  ${PROJECT_NAME}_image
    opencv_highgui
    ${cpp_typesupport_target}
    ${CUDA_nvToolsExt_LIBRARY}
    ${CUDA_LIBRARIES}
)
install(TARGETS cuda_image_node DESTINATION lib/${PROJECT_NAME})

add_executable(intra_gmat_node intra_gmat_node.cpp)
ament_target_dependencies(intra_gmat_node "rclcpp" "cv_bridge" "sensor_msgs")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_include_directories(intra_gmat_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(intra_gmat_node  ${PROJECT_NAME}_image
    ${cpp_typesupport_target}
    ${CUDA_nvToolsExt_LIBRARY}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
)
install(TARGETS intra_gmat_node DESTINATION lib/${PROJECT_NAME})

add_executable(gmat_image_node gmat_image_node.cpp gmat_image_container.cpp)
ament_target_dependencies(gmat_image_node "rclcpp" "cv_bridge" "sensor_msgs")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_include_directories(gmat_image_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(gmat_image_node  ${PROJECT_NAME}_image
    ${cpp_typesupport_target}
    ${CUDA_nvToolsExt_LIBRARY}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
)
install(TARGETS gmat_image_node DESTINATION lib/${PROJECT_NAME})
