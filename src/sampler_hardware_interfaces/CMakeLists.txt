cmake_minimum_required(VERSION 3.16)
project(sampler_hardware_interfaces)

# --------------------------------------------------
# Compiler options
# --------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------
# Find dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(angles REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
# --------------------------------------------------
# Build shared library (hardware plugin)
# --------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/sampler_hardware.cpp
)

# --------------------------------------------------
# Include directories
# --------------------------------------------------
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)

# --------------------------------------------------
# Dependencies for the library
# --------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  angles
  ros2_control_test_assets
)



# --------------------------------------------------
# Export plugin description file
# --------------------------------------------------
pluginlib_export_plugin_description_file(
  hardware_interface
  sampler_hardware_interfaces.xml
)

# --------------------------------------------------
# Export libraries (optional but recommended)
# --------------------------------------------------
ament_export_libraries(${PROJECT_NAME})

# --------------------------------------------------
# Install targets
# --------------------------------------------------
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES sampler_hardware_interfaces.xml
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Testing section
# --------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gmock(test_my_robot_hardware
    test/test_load_sampler_hardware_interfaces.cpp
  )

  target_link_libraries(test_my_robot_hardware
    ${PROJECT_NAME}
  )

  ament_target_dependencies(test_my_robot_hardware
    hardware_interface
  )

  # optional compile definitions (if needed)
  target_compile_definitions(test_my_robot_hardware PRIVATE "TESTING")
endif()

# --------------------------------------------------
# Final package macro
# --------------------------------------------------
ament_package()