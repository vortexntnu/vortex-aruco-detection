find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}_test 
    aruco_detector_test.cpp
    ../src/aruco_detector.cpp 
)
target_link_libraries(${PROJECT_NAME}_test
  ${OpenCV_LIBS} # Link OpenCV libraries


  # Eigen3::Eigen # Link Eigen3 if your tests require it
  # Add any other libraries your tests might require
)

target_include_directories(${PROJECT_NAME}_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS} # Ensure OpenCV include dirs are available to the test
)
ament_target_dependencies(${PROJECT_NAME}_test
  cv_bridge
)
