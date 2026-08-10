cmake_minimum_required(VERSION 3.8)
project(calibration_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libVescCan REQUIRED)
find_package(rex_interfaces REQUIRED)
find_package(ros_constants REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(${PROJECT_NAME}_component SHARED
  src/calibration_manager.cpp
  src/CalibrateAxis.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME}_component PUBLIC
  ament_index_cpp::ament_index_cpp
  ${cpp_typesupport_target}
  ${rex_interfaces_TARGETS}
  ${rclcpp_components_TARGETS}
  rclcpp::rclcpp
  ros_constants::ros_constants
  libVescCan::libVescCan
)
target_compile_features(${PROJECT_NAME}_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "CalibrateAxis"
  EXECUTABLE calibrate_axis
)

install(TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
