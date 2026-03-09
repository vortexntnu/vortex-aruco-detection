cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

set(LIB "${PROJECT_NAME}")

add_library(${LIB} SHARED
  src/lib/aruco_detector.cpp
)

target_link_libraries(${LIB} PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

target_include_directories(${LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${LIB} PUBLIC
  spdlog
  fmt
)

set(ROS_LIB "${PROJECT_NAME}_component")

add_library(${ROS_LIB} SHARED
  src/ros/aruco_detector_ros.cpp
  src/ros/aruco_file_logger.cpp
)

target_link_libraries(${ROS_LIB} PUBLIC
  ${LIB}
)

target_include_directories(${ROS_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${ROS_LIB} PUBLIC
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  vortex_msgs
  tf2
  cv_bridge
  std_srvs
  spdlog
  fmt
)

rclcpp_components_register_node(
    ${ROS_LIB}
    PLUGIN "ArucoDetectorNode"
    EXECUTABLE ${PROJECT_NAME}_node
)

# Export the target for other packages to use
ament_export_targets(export_${LIB} HAS_LIBRARY_TARGET)
ament_export_targets(export_${ROS_LIB} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(OpenCV spdlog fmt)
ament_export_dependencies(rclcpp rclcpp_components cv_bridge sensor_msgs)

install(TARGETS ${LIB}
  EXPORT export_${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${ROS_LIB}
  EXPORT export_${ROS_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
