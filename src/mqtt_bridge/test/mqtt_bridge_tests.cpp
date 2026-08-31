/*
Copyright (c) 2024-2026 Bartosz Mazurkiewicz.
Licensed under the MIT License.
*/

#include <thread>

#include <gtest/gtest.h>
#include <mosquitto.h>

#include <rclcpp/rclcpp.hpp>
#include "mqtt_bridge/MqttBridge.hpp"
#include <rcl_interfaces/msg/log.hpp>
#include "rex_interfaces/msg/rover_control.hpp"

#include "mqtt_bridge/RapidJsonConfig.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

using namespace std::chrono_literals;

// TODO: Test for ServoStatus

// MQTT parameters
constexpr const char* MQTT_HOST = "localhost";
const int MQTT_PORT = 8883;
constexpr const char* MQTT_CLIENT_ID = "mqtt_bridge_test";
constexpr const char* MQTT_USERNAME = "raptors";
constexpr const char* MQTT_PASSWORD = "changeme";
constexpr const char* MQTT_CA_PATH = "/opt/share/raptor_ws_mqtt_certs/ca.crt";

class MqttBridgeTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    mosquitto_lib_init();
  }

  static void TearDownTestSuite() {
    mosquitto_lib_cleanup();
  }

  static bool compareDoublesWithEpsilon(double a, double b, double epsilon)
  {
    double diff = a - b;
    return (diff < epsilon) && (-diff < epsilon);
  }

  static rclcpp::Time unixMillisecondsToROSTimestamp(u_int64_t msec)
  {
      rclcpp::Time timestamp((int64_t)(msec * 1000000));
      return timestamp;
  }

  static mosquitto* connectToBroker(void* context) {
    mosquitto* mosq = mosquitto_new(MQTT_CLIENT_ID, true, nullptr);
    if    ((mosq == nullptr)
        || (mosquitto_tls_set(mosq, MQTT_CA_PATH, nullptr, nullptr, nullptr, nullptr) != MOSQ_ERR_SUCCESS)
        || (mosquitto_tls_insecure_set(mosq, true) != MOSQ_ERR_SUCCESS)
        || (mosquitto_username_pw_set(mosq, MQTT_USERNAME, MQTT_PASSWORD) != MOSQ_ERR_SUCCESS)
        || (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 20) != MOSQ_ERR_SUCCESS)) {
            mosquitto_destroy(mosq);
            return nullptr;
        }
    mosquitto_user_data_set(mosq, context);
    return mosq;
  }
};

TEST_F(MqttBridgeTest, mqtt_connection_test)
{
  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create a ROS node for the test
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  executor->add_node(test_node);

  bool mqtt_conn_success_received = false;
  // subscribe to ROS logs, check if MQTT connection success message (from mqtt_bridge) gets logged
  auto sub = test_node->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", 10,
    [&](rcl_interfaces::msg::Log::SharedPtr msg) {
      if (msg->msg.find("Successfully connected to MQTT server.") != std::string::npos) {
        mqtt_conn_success_received = true;
      }
    });

  {
    // create mqtt_bridge node
    rclcpp::NodeOptions options;
    MqttBridge mqtt_bridge_node(options);

    // spin until message arrives or timeout
    auto start = std::chrono::steady_clock::now();
    while (!mqtt_conn_success_received) {
      executor->spin_some();
      if (std::chrono::steady_clock::now() - start > 3s) {
        break;
      }
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  rclcpp::shutdown();

  ASSERT_TRUE(mqtt_conn_success_received);
}

TEST_F(MqttBridgeTest, mqtt_to_ros_robotic_arm_control_test)
{
  // create an MQTT client, connect to broker
  mosquitto* mosq = connectToBroker(nullptr);
  ASSERT_TRUE(mosq);

  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create a ROS node for testing
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  // create mqtt_bridge node
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect to broker
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  bool ros_msg_received = false;
  // subscribe to RoboticArmControl topic, check if message is received
  auto sub = test_node->create_subscription<rex_interfaces::msg::RoboticArmControl>(
    "/MQTT/RoboticArmControl", 10,
    [&](rex_interfaces::msg::RoboticArmControl::SharedPtr msg) {
      if (msg->action_type == 1
          && msg->force_cartesian == true
          && msg->force_movement == false
          && msg->gripper == 0.5
          && compareDoublesWithEpsilon(msg->joint_positions[0], 1.1, 0.01)
          && compareDoublesWithEpsilon(msg->joint_positions[1], -2.2, 0.01)
          && compareDoublesWithEpsilon(msg->joint_positions[2], 3.3, 0.01)
          && compareDoublesWithEpsilon(msg->joint_positions[3], -4.4, 0.01)
          && compareDoublesWithEpsilon(msg->joint_positions[4], 5.5, 0.01)
          && compareDoublesWithEpsilon(msg->joint_positions[5], -6.6, 0.01)
          && ((int64_t)msg->header.stamp.sec * 1000) + ((int64_t)msg->header.stamp.nanosec / 1000000) == 1688729381666) {
        ros_msg_received = true;
      }
    });
    // prepare MQTT message to send
  const char* mqtt_payload = "{\"ActionType\": 1, \"ForceCartesian\": true, \"ForceMovement\": false, \"Gripper\": 0.5, \"ForwardKin\": {\"Axis1\": 1.1, \"Axis2\": -2.2, \"Axis3\": 3.3, \"Axis4\": -4.4, \"Axis5\": 5.5, \"Axis6\": -6.6}, \"InvJoystick\": null, \"InvPosition\": null, \"Reference\": null, \"Timestamp\":1688729381666}";
  EXPECT_EQ(MOSQ_ERR_SUCCESS,
   mosquitto_publish(mosq, nullptr, "RappTORS/RoboticArmControl", strlen(mqtt_payload), mqtt_payload, 0, false)); 

   // Spin until ROS message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!ros_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  mosquitto_destroy(mosq);

  ASSERT_TRUE(ros_msg_received);
}

TEST_F(MqttBridgeTest, mqtt_to_ros_rover_control_test)
{
  // create an MQTT client, connect to broker
  mosquitto* mosq = connectToBroker(nullptr);
  ASSERT_TRUE(mosq);

  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create a ROS node for testing
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  // create mqtt_bridge node
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect to broker
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  bool ros_msg_received = false;
  // subscribe to RoverControl topic, check if message is received
  auto sub = test_node->create_subscription<rex_interfaces::msg::RoverControl>(
    "/MQTT/RoverControl", 10,
    [&](rex_interfaces::msg::RoverControl::SharedPtr msg) {
      if (compareDoublesWithEpsilon(msg->x_axis, 1.0, 0.01)
          && compareDoublesWithEpsilon(msg->y_axis, 0.8, 0.01)
          && compareDoublesWithEpsilon(msg->vel, -0.314, 0.01)
          && msg->mode == 3
          && ((int64_t)msg->header.stamp.sec * 1000) + ((int64_t)msg->header.stamp.nanosec / 1000000) == 1688729381666) {
        ros_msg_received = true;
      }
    });

  // prepare MQTT message to send
  const char* mqtt_payload = "{\"XAxis\": 1.0, \"YAxis\": 0.8, \"Vel\": -0.314, \"Mode\": 3, \"Timestamp\":1688729381666}";
  EXPECT_EQ(MOSQ_ERR_SUCCESS,
   mosquitto_publish(mosq, nullptr, "RappTORS/RoverControl", strlen(mqtt_payload), mqtt_payload, 0, false));

  // Spin until ROS message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!ros_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  mosquitto_destroy(mosq);

  ASSERT_TRUE(ros_msg_received);
}

TEST_F(MqttBridgeTest, mqtt_to_ros_sampler_control_test)
{
  // create an MQTT client, connect to broker
  mosquitto* mosq = connectToBroker(nullptr);
  ASSERT_TRUE(mosq);

  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create a ROS node for testing
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  // create mqtt_bridge node
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect to broker
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  bool ros_msg_received = false;
  // subscribe to SamplerControl topic, check if message is received
  auto sub = test_node->create_subscription<rex_interfaces::msg::SamplerControl>(
    "/MQTT/SamplerControl", 10,
    [&](rex_interfaces::msg::SamplerControl::SharedPtr msg) {
      if (compareDoublesWithEpsilon(msg->platform_movement, 0.1, 0.01)
          && compareDoublesWithEpsilon(msg->drill_movement, -0.2, 0.01)
          && compareDoublesWithEpsilon(msg->drill_action, 0.3, 0.01)
          && compareDoublesWithEpsilon(msg->container_degrees_a, 4.4, 0.01)
          && compareDoublesWithEpsilon(msg->container_degrees_b, 5.5, 0.01)
          && compareDoublesWithEpsilon(msg->vacuum_suction, 0.5, 0.01)
          && compareDoublesWithEpsilon(msg->vacuum_a, 0.6, 0.01)
          && compareDoublesWithEpsilon(msg->vacuum_b, 0.7, 0.01)
          && ((int64_t)msg->header.stamp.sec * 1000) + ((int64_t)msg->header.stamp.nanosec / 1000000) == 1688729381666) {
        ros_msg_received = true;
      }
    });

  // prepare MQTT message to send
  const char* mqtt_payload
   = "{\"PlatformMovement\": 0.1, \"DrillMovement\": -0.2, \"DrillAction\": 0.3, \"ContainerDegrees0\": 4.4, \"ContainerDegrees1\": 5.5, \"VacuumSuction\": 0.5, \"VacuumA\": 0.6, \"VacuumB\": 0.7, \"Timestamp\":1688729381666}";
  EXPECT_EQ(MOSQ_ERR_SUCCESS,
   mosquitto_publish(mosq, nullptr, "RappTORS/SamplerControl", strlen(mqtt_payload), mqtt_payload, 0, false));

  // Spin until ROS message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!ros_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  mosquitto_destroy(mosq);

  ASSERT_TRUE(ros_msg_received);
}

TEST_F(MqttBridgeTest, mqtt_to_ros_calibrate_axis_test)
{
  // create an MQTT client, connect to broker
  mosquitto* mosq = connectToBroker(nullptr);
  ASSERT_TRUE(mosq);

  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create a ROS node for testing
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  // create mqtt_bridge node
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect to broker
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  bool ros_msg_received = false;
  // subscribe to CalibrateAxis topic, check if message is received
  auto sub = test_node->create_subscription<rex_interfaces::msg::CalibrateAxis>(
    "/MQTT/CalibrateAxis", 10,
    [&](rex_interfaces::msg::CalibrateAxis::SharedPtr msg) {
      if (msg->vesc_id == 15
          && compareDoublesWithEpsilon(msg->value, -3.14, 0.01)
          && msg->action_type == 2
          && ((int64_t)msg->header.stamp.sec * 1000) + ((int64_t)msg->header.stamp.nanosec / 1000000) == 1688729381666) {
        ros_msg_received = true;
      }
    });

  // prepare MQTT message to send
  const char* mqtt_payload
   = "{\"VescID\": 15, \"Value\": -3.14, \"ActionType\": 2, \"Timestamp\":1688729381666}";
  EXPECT_EQ(MOSQ_ERR_SUCCESS,
   mosquitto_publish(mosq, nullptr, "RappTORS/CalibrateAxis", strlen(mqtt_payload), mqtt_payload, 0, false));

  // Spin until ROS message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!ros_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  mosquitto_destroy(mosq);

  ASSERT_TRUE(ros_msg_received);
}

// TODO: add a negative test for failed RoverStatus validation (incompatible modes)
TEST_F(MqttBridgeTest, mqtt_to_ros_rover_status_test)
{
  // create an MQTT client, connect to broker
  mosquitto* mosq = connectToBroker(nullptr);
  ASSERT_TRUE(mosq);

  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create a ROS node for testing
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  // create mqtt_bridge node
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect to broker
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  bool ros_msg_received = false;
  // subscribe to RoverStatus topic, check if message is received
  auto sub = test_node->create_subscription<rex_interfaces::msg::RoverStatus>(
    "/MQTT/RoverStatus", 10,
    [&](rex_interfaces::msg::RoverStatus::SharedPtr msg) {
      if (msg->communication_state == rex_interfaces::msg::RoverStatus::COMMUNICATION_STATE_OPENED
          && msg->control_mode == (rex_interfaces::msg::RoverStatus::CONTROL_MODE_ROBOTIC_ARM | rex_interfaces::msg::RoverStatus::CONTROL_MODE_DRIVE)
          && ((int64_t)msg->header.stamp.sec * 1000) + ((int64_t)msg->header.stamp.nanosec / 1000000) == 1688729381666) {
        ros_msg_received = true;
      }
    });

  // prepare MQTT message to send
  const char* mqtt_payload
   = "{\"CommunicationState\": 2, \"ControlMode\": 24, \"Timestamp\":1688729381666}";
  EXPECT_EQ(MOSQ_ERR_SUCCESS,
   mosquitto_publish(mosq, nullptr, "RappTORS/RoverStatus", strlen(mqtt_payload), mqtt_payload, 0, false));

  // Spin until ROS message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!ros_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  mosquitto_destroy(mosq);

  ASSERT_TRUE(ros_msg_received);
}

TEST_F(MqttBridgeTest, ros_to_mqtt_battery_info_test)
{
  // ROS init
  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  bool mqtt_msg_received = false;

  // connect to broker, pass bool by reference
  mosquitto* mosq = connectToBroker(&mqtt_msg_received);
  ASSERT_TRUE(mosq);

  // create ROS test node and mqtt_bridge node
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  auto pub = test_node->create_publisher<rex_interfaces::msg::BatteryInfo>("/CAN/RX/battery_info", 10);

  // prepare BatteryInfo message
  rex_interfaces::msg::BatteryInfo msg;
  msg.slot = 2;
  msg.id = 5;
  msg.voltage = 3.14f;
  msg.current = 1.23f;
  msg.temperature = 50.1f;
  msg.charge_percent = 92.5f;
  msg.battery_status = 6;
  msg.hotswap_status = rex_interfaces::msg::BatteryInfo::HOTSWAP_MASTER;
  msg.header.stamp = unixMillisecondsToROSTimestamp(1688729381666);

  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_subscribe(mosq, nullptr, "RappTORS/BatteryInfo", 0));

  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_loop_start(mosq));

  // callback for MQTT message receive
  mosquitto_message_callback_set(
    mosq,
    [](struct mosquitto *mosq __attribute__((unused)),
       void *userdata,
       const struct mosquitto_message *msg) {
        if (strcmp(msg->topic, "RappTORS/BatteryInfo") == 0 && msg->payloadlen > 0) {
          rapidjson::Document d;

          rapidjson::ParseResult ok = d.Parse(reinterpret_cast<char*>(msg->payload));
          if (!ok) return;

          try
          {
            if (d["Slot"].GetInt() == 2
             && d["ID"].GetInt() == 5
             && d["BatteryStatus"].GetUint() == 6
             && d["HotswapStatus"].GetUint() == 1
             && d["Timestamp"].GetUint64() == 1688729381666
             && compareDoublesWithEpsilon(d["ChargePercent"].GetDouble(), 92.5, 0.01)
             && compareDoublesWithEpsilon(d["Current"].GetDouble(), 1.23, 0.01)
             && compareDoublesWithEpsilon(d["Temperature"].GetDouble(), 50.1, 0.01)
             && compareDoublesWithEpsilon(d["Voltage"].GetDouble(), 3.14, 0.01))
            {
              // set mqtt_msg_received to true if correct message received
              *reinterpret_cast<bool*>(userdata) = true;
            }
          }
          catch (const JsonAssertException& e)
          {
              return;
          }
        }
    }
  );

  // sleep to allow mosquitto client to subscribe
  std::this_thread::sleep_for(500ms);

  pub->publish(msg);

  // Spin until MQTT message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!mqtt_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_loop_stop(mosq, true));
  mosquitto_destroy(mosq);

  ASSERT_TRUE(mqtt_msg_received);
}

TEST_F(MqttBridgeTest, ros_to_mqtt_sampler_feedback_test)
{
  // ROS init
  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  bool mqtt_msg_received = false;

  // connect to broker, pass bool by reference
  mosquitto* mosq = connectToBroker(&mqtt_msg_received);
  ASSERT_TRUE(mosq);

  // create ROS test node and mqtt_bridge node
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  auto pub = test_node->create_publisher<rex_interfaces::msg::SamplerFeedback>("/CAN/RX/sampler_status", 10);

  // prepare SamplerFeedback message
  rex_interfaces::msg::SamplerFeedback msg;
  msg.weight_a = 3.14f;
  msg.weight_b = 1.23f;
  msg.weight_c = 5.1f;
  msg.ph = 8.1f;
  msg.distance = 2.5f;
  msg.header.stamp = unixMillisecondsToROSTimestamp(1688729381666);

  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_subscribe(mosq, nullptr, "RappTORS/SamplerFeedback", 0));

  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_loop_start(mosq));

  // callback for MQTT message receive
  mosquitto_message_callback_set(
    mosq,
    [](struct mosquitto *mosq __attribute__((unused)),
       void *userdata,
       const struct mosquitto_message *msg) {
        if (strcmp(msg->topic, "RappTORS/SamplerFeedback") == 0 && msg->payloadlen > 0) {
          rapidjson::Document d;

          rapidjson::ParseResult ok = d.Parse(reinterpret_cast<char*>(msg->payload));
          if (!ok) return;

          try
          {
            if (
             d["Timestamp"].GetUint64() == 1688729381666
             && compareDoublesWithEpsilon(d["WeightA"].GetDouble(), 3.14, 0.01)
             && compareDoublesWithEpsilon(d["WeightB"].GetDouble(), 1.23, 0.01)
             && compareDoublesWithEpsilon(d["WeightC"].GetDouble(), 5.1, 0.01)
             && compareDoublesWithEpsilon(d["Ph"].GetDouble(), 8.1, 0.01)
             && compareDoublesWithEpsilon(d["Distance"].GetDouble(), 2.5, 0.01))
            {
              // set mqtt_msg_received to true if correct message received
              *reinterpret_cast<bool*>(userdata) = true;
            }
          }
          catch (const JsonAssertException& e)
          {
              return;
          }
        }
    }
  );

  // sleep to allow mosquitto client to subscribe
  std::this_thread::sleep_for(500ms);

  pub->publish(msg);

  // Spin until MQTT message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!mqtt_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_loop_stop(mosq, true));
  mosquitto_destroy(mosq);

  ASSERT_TRUE(mqtt_msg_received);
}

TEST_F(MqttBridgeTest, ros_to_mqtt_vesc_status_test)
{
  // ROS init
  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  struct MqttUserData {
    bool mqtt_vesc1_msg_received = false;
    bool mqtt_vesc5_msg_received = false;
  };

  MqttUserData mqtt_user_data;

  // connect to broker, pass bool by reference
  mosquitto* mosq = connectToBroker(&mqtt_user_data);
  ASSERT_TRUE(mosq);

  // create ROS test node and mqtt_bridge node
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  auto pub = test_node->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", 10);

  // prepare VescStatus VESC 1 #1 message
  rex_interfaces::msg::VescStatus msg;
  msg.vesc_id = 1;
  msg.erpm = 80;
  msg.current = 1.1f;
  msg.duty_cycle = -0.2f;
  msg.ah_used = 3.3f;
  msg.ah_charged = 4.4f;
  msg.wh_used = 5.5f;
  msg.wh_charged = 6.6f;
  msg.temp_fet = 17.7f;
  msg.temp_motor = 18.8f;
  msg.current_in = 5.5f;
  msg.pid_pos = 11.1f;
  msg.tachometer = 12.2f;
  msg.volts_in = 13.3f;
  msg.adc1 = 14.4f;
  msg.adc2 = 15.5f;
  msg.adc3 = 16.6f;
  msg.ppm = 17.7f;
  msg.precise_pos = 18.8;
  msg.header.stamp = unixMillisecondsToROSTimestamp(1688729381666);

  // prepare VescStatus VESC 1 #2 message
  rex_interfaces::msg::VescStatus msg2;
  msg2.vesc_id = 1;
  msg2.erpm = 40;
  msg2.current = 3.3f;
  msg2.duty_cycle = -0.6f;
  msg2.ah_used = 5.5f;
  msg2.ah_charged = 6.6f;
  msg2.wh_used = 7.7f;
  msg2.wh_charged = 8.8f;
  msg2.temp_fet = 19.9f;
  msg2.temp_motor = 16.6f;
  msg2.current_in = 3.3f;
  msg2.pid_pos = 13.3f;
  msg2.tachometer = 14.4f;
  msg2.volts_in = 15.5f;
  msg2.adc1 = 16.6f;
  msg2.adc2 = 17.7f;
  msg2.adc3 = 18.8f;
  msg2.ppm = 19.9f;
  msg2.precise_pos = 16.6;
  msg2.header.stamp = unixMillisecondsToROSTimestamp(1688729381801);

  // prepare VescStatus VESC 5 message
  rex_interfaces::msg::VescStatus msg3;
  msg3.vesc_id = 5;
  msg3.erpm = 50;
  msg3.current = 9.9f;
  msg3.duty_cycle = 0.8f;
  msg3.ah_used = 7.7f;
  msg3.ah_charged = 6.6f;
  msg3.wh_used = 5.5f;
  msg3.wh_charged = 4.4f;
  msg3.temp_fet = 13.3f;
  msg3.temp_motor = 12.2f;
  msg3.current_in = 1.1f;
  msg3.pid_pos = 9.9f;
  msg3.tachometer = 8.8f;
  msg3.volts_in = 7.7f;
  msg3.adc1 = 6.6f;
  msg3.adc2 = 5.5f;
  msg3.adc3 = 4.4f;
  msg3.ppm = 3.3f;
  msg3.precise_pos = 2.2;
  msg3.header.stamp = unixMillisecondsToROSTimestamp(1688729381901);

  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_subscribe(mosq, nullptr, "RappTORS/VescStatus", 0));

  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_loop_start(mosq));

  // callback for MQTT message receive
  mosquitto_message_callback_set(
    mosq,
    [](struct mosquitto *mosq __attribute__((unused)),
       void *userdata,
       const struct mosquitto_message *msg) {
        if (strcmp(msg->topic, "RappTORS/VescStatus") == 0 && msg->payloadlen > 0) {
          rapidjson::Document d;

          rapidjson::ParseResult ok = d.Parse(reinterpret_cast<char*>(msg->payload));
          if (!ok) return;

          try
          {
            if (
             d["Timestamp"].GetUint64() == 1688729381801
             && d["VescId"].GetInt() == 1
             && d["ERPM"].GetInt() == 60
             && compareDoublesWithEpsilon(d["Current"].GetDouble(), 2.2, 0.01)
             && compareDoublesWithEpsilon(d["DutyCycle"].GetDouble(), -0.4, 0.01)
             && compareDoublesWithEpsilon(d["AhUsed"].GetDouble(), 4.4, 0.01)
             && compareDoublesWithEpsilon(d["AhCharged"].GetDouble(), 5.5, 0.01)
             && compareDoublesWithEpsilon(d["WhUsed"].GetDouble(), 6.6, 0.01)
             && compareDoublesWithEpsilon(d["WhCharged"].GetDouble(), 7.7, 0.01)
             && compareDoublesWithEpsilon(d["TempFet"].GetDouble(), 18.8, 0.01)
             && compareDoublesWithEpsilon(d["TempMotor"].GetDouble(), 17.7, 0.01)
             && compareDoublesWithEpsilon(d["CurrentIn"].GetDouble(), 4.4, 0.01)
             && compareDoublesWithEpsilon(d["PidPos"].GetDouble(), 12.2, 0.01)
             && compareDoublesWithEpsilon(d["Tachometer"].GetDouble(), 13.3, 0.01)
             && compareDoublesWithEpsilon(d["VoltsIn"].GetDouble(), 14.4, 0.01)
             && compareDoublesWithEpsilon(d["ADC1"].GetDouble(), 15.5, 0.01)
             && compareDoublesWithEpsilon(d["ADC2"].GetDouble(), 16.6, 0.01)
             && compareDoublesWithEpsilon(d["ADC3"].GetDouble(), 17.7, 0.01)
             && compareDoublesWithEpsilon(d["PPM"].GetDouble(), 18.8, 0.01)
             && compareDoublesWithEpsilon(d["PrecisePos"].GetDouble(), 17.7, 0.01))
            {
              // set mqtt_vesc1_msg_received to true if correct message received
              static_cast<MqttUserData*>(userdata)->mqtt_vesc1_msg_received = true;
            } else if (
             d["Timestamp"].GetUint64() == 1688729381901
             && d["VescId"].GetInt() == 5
             && d["ERPM"].GetInt() == 50
             && compareDoublesWithEpsilon(d["Current"].GetDouble(), 9.9, 0.01)
             && compareDoublesWithEpsilon(d["DutyCycle"].GetDouble(), 0.8, 0.01)
             && compareDoublesWithEpsilon(d["AhUsed"].GetDouble(), 7.7, 0.01)
             && compareDoublesWithEpsilon(d["AhCharged"].GetDouble(), 6.6, 0.01)
             && compareDoublesWithEpsilon(d["WhUsed"].GetDouble(), 5.5, 0.01)
             && compareDoublesWithEpsilon(d["WhCharged"].GetDouble(), 4.4, 0.01)
             && compareDoublesWithEpsilon(d["TempFet"].GetDouble(), 13.3, 0.01)
             && compareDoublesWithEpsilon(d["TempMotor"].GetDouble(), 12.2, 0.01)
             && compareDoublesWithEpsilon(d["CurrentIn"].GetDouble(), 1.1, 0.01)
             && compareDoublesWithEpsilon(d["PidPos"].GetDouble(), 9.9, 0.01)
             && compareDoublesWithEpsilon(d["Tachometer"].GetDouble(), 8.8, 0.01)
             && compareDoublesWithEpsilon(d["VoltsIn"].GetDouble(), 7.7, 0.01)
             && compareDoublesWithEpsilon(d["ADC1"].GetDouble(), 6.6, 0.01)
             && compareDoublesWithEpsilon(d["ADC2"].GetDouble(), 5.5, 0.01)
             && compareDoublesWithEpsilon(d["ADC3"].GetDouble(), 4.4, 0.01)
             && compareDoublesWithEpsilon(d["PPM"].GetDouble(), 3.3, 0.01)
             && compareDoublesWithEpsilon(d["PrecisePos"].GetDouble(), 2.2, 0.01))
            {
              // set mqtt_vesc5_msg_received to true if correct message received
              static_cast<MqttUserData*>(userdata)->mqtt_vesc5_msg_received = true;
            }
          }
          catch (const JsonAssertException& e)
          {
              return;
          }
        }
    }
  );

  // sleep to allow mosquitto client to subscribe
  std::this_thread::sleep_for(500ms);

  pub->publish(msg);
  pub->publish(msg2);
  pub->publish(msg3);

  // Spin until MQTT message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!mqtt_user_data.mqtt_vesc1_msg_received || !mqtt_user_data.mqtt_vesc5_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_loop_stop(mosq, true));
  mosquitto_destroy(mosq);

  ASSERT_TRUE(mqtt_user_data.mqtt_vesc1_msg_received);
  ASSERT_TRUE(mqtt_user_data.mqtt_vesc5_msg_received);
}

TEST_F(MqttBridgeTest, mqtt_to_ros_sampler_control_malformed_json_test)
{
  // create an MQTT client, connect to broker
  mosquitto* mosq = connectToBroker(nullptr);
  ASSERT_TRUE(mosq);

  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create a ROS node for testing
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  // create mqtt_bridge node
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect to broker
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  bool ros_warning_received = false;
  // subscribe to ROS logs, check if JSON warning message (from mqtt_bridge) gets logged
  auto sub = test_node->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", 10,
    [&](rcl_interfaces::msg::Log::SharedPtr msg) {
      if (msg->level == msg->WARN && msg->msg.find("JSON parse error") != std::string::npos) {
        ros_warning_received = true;
      }
    });

  // prepare MQTT message to send
  const char* mqtt_payload = "This is not JSON";
  EXPECT_EQ(MOSQ_ERR_SUCCESS,
   mosquitto_publish(mosq, nullptr, "RappTORS/SamplerControl", strlen(mqtt_payload), mqtt_payload, 0, false));

  // Spin until ROS log message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!ros_warning_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  mosquitto_destroy(mosq);

  ASSERT_TRUE(ros_warning_received);
}

TEST_F(MqttBridgeTest, mqtt_to_ros_sampler_control_invalid_json_test)
{
  GTEST_SKIP();
  // create an MQTT client, connect to broker
  mosquitto* mosq = connectToBroker(nullptr);
  ASSERT_TRUE(mosq);

  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create a ROS node for testing
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  // create mqtt_bridge node
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect to broker
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  bool ros_warning_received = false;
  // subscribe to ROS logs, check if JSON warning message (from mqtt_bridge) gets logged
  auto sub = test_node->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", 10,
    [&](rcl_interfaces::msg::Log::SharedPtr msg) {
      if (msg->level == msg->WARN
          && msg->msg.find("JSON schema validation failed (key: SamplerControl)") != std::string::npos) {
        ros_warning_received = true;
      }
    });

  // prepare MQTT message to send
  const char* mqtt_payload = "{\"Axis1\": 1.1, \"Axis2\": 2.2, \"Axis3\": 3.3, \"Axis4\": 4.4, \"Axis5\": 5.5, \"Axis6\": 6.6, \"Gripper\": 0.5, \"Timestamp\":1688729381666}";
  EXPECT_EQ(MOSQ_ERR_SUCCESS,
   mosquitto_publish(mosq, nullptr, "RappTORS/SamplerControl", strlen(mqtt_payload), mqtt_payload, 0, false));

  // Spin until ROS log message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!ros_warning_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  mosquitto_destroy(mosq);

  ASSERT_TRUE(ros_warning_received);
}

TEST_F(MqttBridgeTest, ros_to_mqtt_sampler_feedback_invalid_msg_test)
{
  GTEST_SKIP();
  // ROS init
  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // create ROS test node and mqtt_bridge node
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  auto pub = test_node->create_publisher<rex_interfaces::msg::SamplerFeedback>("/CAN/RX/sampler_status", 10);

  bool ros_warning_received = false;
  // subscribe to ROS logs, check if JSON warning message (from mqtt_bridge) gets logged
  auto sub = test_node->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", 10,
    [&](rcl_interfaces::msg::Log::SharedPtr msg) {
      if (msg->level == msg->WARN
          && msg->msg.find("JSON schema validation failed (key: SamplerFeedback)") != std::string::npos) {
        ros_warning_received = true;
      }
    });

  // prepare invalid SamplerFeedback message
  rex_interfaces::msg::SamplerFeedback msg;
  msg.weight_a = 3.14f;
  msg.weight_b = 1.23f;
  msg.weight_c = 5.1f;
  msg.ph = 48.1f;
  msg.distance = 1002.5f;
  msg.header.stamp = unixMillisecondsToROSTimestamp(1688729381666);

  pub->publish(msg);

  // Spin until warning log message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!ros_warning_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();

  ASSERT_TRUE(ros_warning_received);
}

TEST_F(MqttBridgeTest, ros_to_mqtt_rosout_logs_test) 
{
  // ROS init
  rclcpp::init(0, nullptr);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  bool mqtt_msg_received = false;

  // connect to broker, pass bool by reference
  mosquitto* mosq = connectToBroker(&mqtt_msg_received);
  ASSERT_TRUE(mosq);

  // create ROS test node and mqtt_bridge node
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::NodeOptions options;
  auto mqtt_bridge_node = std::make_shared<MqttBridge>(options);

  // sleep to allow mqtt_bridge to connect
  std::this_thread::sleep_for(1000ms);

  executor->add_node(test_node);
  executor->add_node(mqtt_bridge_node);

  auto pub = test_node->create_publisher<rcl_interfaces::msg::Log>("/rosout", 10);

  // Przygotowanie testowej wiadomości logu
  rcl_interfaces::msg::Log msg;
  msg.level = 20; // 20 to Info w ROS2
  msg.name = "test_source_node";
  msg.msg = "Testowy komunikat bledu";
  msg.file = "test_file.cpp";
  msg.function = "test_function";
  msg.line = 42;
  msg.stamp = unixMillisecondsToROSTimestamp(1688729381666);

  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_subscribe(mosq, nullptr, "RappTORS/Logs/Info", 0));

  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_loop_start(mosq));

  mosquitto_message_callback_set(
    mosq,
    [](struct mosquitto *mosq __attribute__((unused)),
    void *userdata,
       const struct mosquitto_message *msg) {
        if (strcmp(msg->topic, "RappTORS/Logs/Info") == 0 && msg->payloadlen > 0) {
          rapidjson::Document d;

          rapidjson::ParseResult ok = d.Parse(reinterpret_cast<char*>(msg->payload));
          if (!ok) return;

          try
          {
            if (d["level"].GetInt() == 20
             && std::string(d["name"].GetString()) == "test_source_node"
             && std::string(d["msg"].GetString()) == "Testowy komunikat bledu"
             && std::string(d["file"].GetString()) == "test_file.cpp"
             && std::string(d["function"].GetString()) == "test_function"
             && d["line"].GetUint() == 42
             && d["Timestamp"].GetUint64() == 1688729381666 )
            {
              // set mqtt_msg_received to true if correct message received
              *reinterpret_cast<bool*>(userdata) = true;
            }
          }
          catch (const JsonAssertException& e)
          {
              return;
          }
        }
    }
  );

  // sleep to allow mosquitto client to subscribe
  std::this_thread::sleep_for(500ms);

  pub->publish(msg);

  // Spin until MQTT message arrives or timeout
  auto start = std::chrono::steady_clock::now();
  while (!mqtt_msg_received) {
    executor->spin_some();
    if (std::chrono::steady_clock::now() - start > 2s) {
      break;
    }
  }

  // cleanup
  executor->cancel();
  executor->remove_node(test_node);
  executor->remove_node(mqtt_bridge_node);
  rclcpp::shutdown();
  mosquitto_disconnect(mosq);
  EXPECT_EQ(MOSQ_ERR_SUCCESS, mosquitto_loop_stop(mosq, true));
  mosquitto_destroy(mosq);

  ASSERT_TRUE(mqtt_msg_received);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
