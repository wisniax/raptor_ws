cmake_minimum_required(VERSION 3.8)


project(can_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(can_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VescMotorCommand.msg"
  "msg/ManipulatorControl.msg"
  "msg/ProbeStatus.msg"
  "msg/RoverControl.msg"
  "msg/RoverStatus.msg"
  "msg/VescStatus.msg"
  "msg/Wheel.msg"
  "msg/Wheels.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(can_bridge_node src/can_bridge.cpp 
	src/RosCanConstants.cpp
	src/VescInterop.cpp
 	src/VescStatusHandler.cpp
	src/StatusMessage.cpp
 )

add_subdirectory(${CMAKE_SOURCE_DIR}/libVescCan ${CATKIN_DEVEL_PREFIX})

target_include_directories(can_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libVescCan/include>
  )

target_link_libraries(can_bridge_node PUBLIC
  ament_index_cpp::ament_index_cpp
  ${cpp_typesupport_target}
  ${std_msgs_TARGETS}
  ${can_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  libVescCan
)

target_compile_features(can_bridge_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS can_bridge_node
  DESTINATION lib/${PROJECT_NAME})

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
