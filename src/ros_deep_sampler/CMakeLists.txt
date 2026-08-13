cmake_minimum_required(VERSION 3.8)
project(ros_deep_sampler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies

find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

find_package(ament_cmake REQUIRED)
find_package(sampler_motion_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rex_interfaces REQUIRED)
find_package(libVescCan REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(backward_ros REQUIRED)
find_package(control_msgs REQUIRED)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()



install(
  DIRECTORY include/
  DESTINATION include
)
# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config files
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes if you have them
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


add_executable(ros_deep_sampler_node 
  src/main.cpp
  src/joint_movement.cpp
  src/autonomy_controller.cpp
  src/autonomy_states.cpp
  src/RosCanConstants.cpp
  src/mission_control.cpp
  
  
 
)

ament_target_dependencies(ros_deep_sampler_node
  rclcpp
  std_msgs
  rex_interfaces   
  sampler_motion_interfaces    
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


target_link_libraries(ros_deep_sampler_node
  ${cpp_typesupport_target}
  libVescCan::libVescCan
  #action_server
 
  #harware_sampler
)


# Export
ament_export_dependencies(
  rosidl_default_runtime
  rex_interfaces 
  sampler_motion_interfaces
)

# Install
install(TARGETS
  ros_deep_sampler_node
  DESTINATION lib/${PROJECT_NAME}
)


# install(TARGETS
#   action_server
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)


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


ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
