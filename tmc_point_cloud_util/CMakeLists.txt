cmake_minimum_required(VERSION 3.8)
project(tmc_point_cloud_util)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Set
set(dependencies
  geometry_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
)

# point_cloud_util
add_library(point_cloud_util SHARED
  src/util.cpp
)
target_include_directories(point_cloud_util
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(point_cloud_util
  ${dependencies}
)


# Install
install(TARGETS point_cloud_util
  EXPORT export_point_cloud_util
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_point_cloud_util HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
