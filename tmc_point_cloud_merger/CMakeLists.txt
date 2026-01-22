cmake_minimum_required(VERSION 3.8)
project(tmc_point_cloud_merger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# Set
set(executable_name point_cloud_merger)
set(dependencies
  geometry_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_ros
)

add_executable(${executable_name}
  src/point_cloud_merger_main.cpp
)

target_include_directories(${executable_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

ament_export_dependencies(
  ${dependencies}
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )
  add_subdirectory(test)
endif()

ament_package()
