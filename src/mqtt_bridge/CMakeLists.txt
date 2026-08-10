cmake_minimum_required(VERSION 3.14.4)
project(mqtt_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rex_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Paho MQTT library
find_library(PahoMqttC_LIBRARY libpaho-mqtt3as.so.1 REQUIRED)
find_library(PahoMqttCpp_LIBRARY libpaho-mqttpp3.so.1 REQUIRED)

## Declare a C++ library
add_library(${PROJECT_NAME}_component SHARED
  src/mqtt_bridge_node.cpp
  src/MqttBridge.cpp
  src/ROSTopicHandler.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME}_component PUBLIC
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${rex_interfaces_TARGETS}
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
  ${PahoMqttC_LIBRARY}
  ${PahoMqttCpp_LIBRARY}
  "${cpp_typesupport_target}"
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  # Static analysis
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
  # Mosquitto MQTT library
  find_library(MOSQUITTO_LIB mosquitto REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_tests test/mqtt_bridge_tests.cpp)
  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_component ${rcl_interfaces_TARGETS} ${MOSQUITTO_LIB})
endif()

rclcpp_components_register_node(
    ${PROJECT_NAME}_component
    PLUGIN "MqttBridge"
    EXECUTABLE ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_component
        EXPORT export_${PROJECT_NAME}_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/schema")
  install(DIRECTORY schema
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
