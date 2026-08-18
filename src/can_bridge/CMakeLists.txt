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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(libVescCan REQUIRED)
find_package(rex_interfaces REQUIRED)
find_package(ros_constants REQUIRED)

#rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_library(can_bridge_common SHARED
        src/VescInterop.cpp
        )
target_compile_features(can_bridge_common PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(can_bridge_common PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
target_link_libraries(can_bridge_common
        libVescCan::libVescCan
        ros_constants::ros_constants
        )
ament_target_dependencies(can_bridge_common
        rclcpp
        can_msgs
        )
ament_export_targets(export_can_bridge_common)
install(TARGETS can_bridge_common
        EXPORT export_can_bridge_common
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

add_library(vesc_status_handler_component SHARED src/VescStatusHandler.cpp)
add_library(status_message_component SHARED src/StatusMessage.cpp)
add_library(motor_control_component SHARED src/MotorControl.cpp)
add_library(manipulator_control_component SHARED src/ManipulatorControl.cpp)
add_library(sampler_status_forwarder_component SHARED src/SamplerStatusForwarder.cpp)
add_library(sampler_control_component SHARED src/SamplerControl.cpp)
add_library(battery_info_forwarder_component SHARED src/BatteryInfoForwarder.cpp)
add_library(config_control_component SHARED src/ConfigControl.cpp)

target_link_libraries(vesc_status_handler_component can_bridge_common)
target_link_libraries(status_message_component can_bridge_common)
target_link_libraries(motor_control_component can_bridge_common)
target_link_libraries(manipulator_control_component can_bridge_common)
target_link_libraries(sampler_status_forwarder_component can_bridge_common)
target_link_libraries(sampler_control_component can_bridge_common)
target_link_libraries(battery_info_forwarder_component can_bridge_common)
target_link_libraries(config_control_component can_bridge_common)

ament_target_dependencies(vesc_status_handler_component
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
        can_msgs
        rex_interfaces
        ros_constants
        )
ament_target_dependencies(status_message_component
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
        can_msgs
        rex_interfaces
        ros_constants
        )
ament_target_dependencies(motor_control_component
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
        can_msgs
        rex_interfaces
        ros_constants
        )
ament_target_dependencies(manipulator_control_component
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
        can_msgs
        rex_interfaces
        ros_constants
        )
ament_target_dependencies(sampler_status_forwarder_component
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
        can_msgs
        rex_interfaces
        ros_constants
        )
ament_target_dependencies(sampler_control_component
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
        can_msgs
        rex_interfaces
        ros_constants
        )
ament_target_dependencies(battery_info_forwarder_component
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
        can_msgs
        rex_interfaces
        ros_constants
        )
ament_target_dependencies(config_control_component
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
        can_msgs
        rex_interfaces
        ros_constants
        )

rclcpp_components_register_node(
        vesc_status_handler_component
        PLUGIN "VescStatusHandler"
        EXECUTABLE vesc_status_handler
)
rclcpp_components_register_node(
        status_message_component
        PLUGIN "StatusMessage"
        EXECUTABLE status_message
)
rclcpp_components_register_node(
        motor_control_component
        PLUGIN "MotorControl"
        EXECUTABLE motor_control
)
rclcpp_components_register_node(
        manipulator_control_component
        PLUGIN "ManipulatorControl"
        EXECUTABLE manipulator_control
)
rclcpp_components_register_node(
        sampler_status_forwarder_component
        PLUGIN "SamplerStatusForwarder"
        EXECUTABLE sampler_status_forwarder
)
rclcpp_components_register_node(
        sampler_control_component
        PLUGIN "SamplerControl"
        EXECUTABLE sampler_control
)
rclcpp_components_register_node(
        battery_info_forwarder_component
        PLUGIN "BatteryInfoForwarder"
        EXECUTABLE battery_info_forwarder
)
rclcpp_components_register_node(
        config_control_component
        PLUGIN "ConfigControl"
        EXECUTABLE config_control
)

ament_export_targets(export_vesc_status_handler_component)
install(TARGETS vesc_status_handler_component
        EXPORT export_vesc_status_handler_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
ament_export_targets(export_status_message_component)
install(TARGETS status_message_component
        EXPORT export_status_message_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
ament_export_targets(export_motor_control_component)
install(TARGETS motor_control_component
        EXPORT export_motor_control_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
ament_export_targets(export_manipulator_control_component)
install(TARGETS manipulator_control_component
        EXPORT export_manipulator_control_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
ament_export_targets(export_sampler_status_forwarder_component)
install(TARGETS sampler_status_forwarder_component
        EXPORT export_sampler_status_forwarder_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
ament_export_targets(export_sampler_control_component)
install(TARGETS sampler_control_component
        EXPORT export_sampler_control_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
ament_export_targets(export_battery_info_forwarder_component)
install(TARGETS battery_info_forwarder_component
        EXPORT export_battery_info_forwarder_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
ament_export_targets(export_config_control_component)
install(TARGETS config_control_component
        EXPORT export_config_control_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
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

ament_export_dependencies(
        rclcpp
        rclcpp_components
        can_msgs
        std_msgs
        rex_interfaces
        ros_constants
        libVescCan
)

ament_package()
