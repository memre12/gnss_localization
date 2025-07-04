cmake_minimum_required(VERSION 3.14)
project(ekf_localizer)

find_package(autoware_cmake REQUIRED)
find_package(kalman_filter REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ekf_localizer.cpp
  src/covariance.cpp
  src/diagnostics.cpp
  src/mahalanobis.cpp
  src/measurement.cpp
  src/state_transition.cpp
  src/warning_message.cpp
  src/ekf_module.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "EKFLocalizer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" ${PROJECT_NAME})
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()


# if(BUILD_TESTING)
#   find_package(ament_cmake_ros REQUIRED)
#   ament_add_ros_isolated_gtest(ekf_localizer-test test/test_ekf_localizer.test
#     test/src/test_ekf_localizer.cpp
#     src/ekf_localizer.cpp
#     src/kalman_filter/kalman_filter.cpp
#     src/kalman_filter/time_delay_kalman_filter.cpp
#   )
#   target_include_directories(ekf_localizer-test
#     PRIVATE
#     include
#   )
#   ament_target_dependencies(ekf_localizer-test geometry_msgs rclcpp tf2 tf2_ros)
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
