cmake_minimum_required(VERSION 3.8)
project(linear_movement_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sampler_motion_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
#find_package(rex_interfaces REQUIRED)
#find_package(libVescCan REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


#


install(DIRECTORY include/
  DESTINATION include
)

add_library(action_server SHARED
  src/linear_movement_server.cpp)
target_include_directories(action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PUBLIC "LINEAR_MOVEMENT_CONTROL_BUILDING_DLL")
ament_target_dependencies(action_server
  "sampler_motion_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  )
rclcpp_components_register_node(action_server PLUGIN "linear_movement_control::MoveLinearActionServer" EXECUTABLE linear_movement_server)


add_library(action_client SHARED
  src/linear_movement_client.cpp)
target_include_directories(action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PUBLIC "LINEAR_MOVEMENT_CONTROL_BUILDING_DLL")
ament_target_dependencies(action_client
  "sampler_motion_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  )
rclcpp_components_register_node(action_client PLUGIN "linear_movement_control::MoveLinearActionClient" EXECUTABLE linear_movement_client)


install(TARGETS
  action_server
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)




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
