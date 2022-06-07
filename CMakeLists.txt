cmake_minimum_required(VERSION 3.5)

project(shm_msgs)

list(INSERT CMAKE_PREFIX_PATH 0 /opt/open3d/open3d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D)
find_package(Boost QUIET)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_msgs REQUIRED)
# pcl vtk conflict
list(REMOVE_ITEM PCL_LIBRARIES
  vtkChartsCore
  vtkCommonColor
  vtkCommonCore
  vtksys
  vtkCommonDataModel
  vtkCommonMath
  vtkCommonMisc
  vtkCommonSystem
  vtkCommonTransforms
  vtkCommonExecutionModel
  vtkFiltersGeneral
  vtkCommonComputationalGeometry
  vtkFiltersCore
  vtkInfovisCore
  vtkFiltersExtraction
  vtkFiltersStatistics
  vtkImagingFourier
  vtkImagingCore
  vtkalglib
  vtkRenderingContext2D
  vtkRenderingCore
  vtkFiltersGeometry
  vtkFiltersSources
  vtkRenderingFreeType
  vtkFiltersModeling
  vtkImagingSources
  vtkInteractionStyle
  vtkInteractionWidgets
  vtkFiltersHybrid
  vtkImagingColor
  vtkImagingGeneral
  vtkImagingHybrid
  vtkIOImage
  vtkDICOMParser
  vtkmetaio
  vtkRenderingAnnotation
  vtkRenderingVolume
  vtkIOXML
  vtkIOCore
  vtkIOXMLParser
  vtkIOGeometry
  vtkIOLegacy
  vtkIOPLY
  vtkRenderingLOD
  vtkViewsContext2D
  vtkViewsCore
  vtkRenderingContextOpenGL2
  vtkRenderingOpenGL2
)

find_package(OpenCV 4 QUIET
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED
    COMPONENTS
      opencv_core
      opencv_imgproc
      opencv_imgcodecs
    CONFIG
  )
endif()

find_package(iceoryx_posh CONFIG REQUIRED)
find_package(fastrtps CONFIG REQUIRED)

set(dependencies
  rclcpp
  rosidl_default_runtime
  cv_bridge
  message_filters
  pcl_msgs
)

set(msg_files
  "msg/String.msg"
  "msg/Header.msg"
  "msg/Image8k.msg"
  "msg/Image512k.msg"
  "msg/Image1m.msg"
  "msg/Image2m.msg"
  "msg/Image4m.msg"
  "msg/Image8m.msg"
  "msg/PointField.msg"
  "msg/PointCloud8k.msg"
  "msg/PointCloud512k.msg"
  "msg/PointCloud1m.msg"
  "msg/PointCloud2m.msg"
  "msg/PointCloud4m.msg"
  "msg/PointCloud8m.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_COMMON_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY res
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(lib)

add_subdirectory(src)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(tool)

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()
