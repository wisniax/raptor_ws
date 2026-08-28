#include "mqtt_bridge/MqttBridge.hpp"

/*
Copyright (c) 2024-2026 Bartosz Mazurkiewicz.
Licensed under the MIT License.
*/

MqttBridge::MqttBridge(const rclcpp::NodeOptions& options) : rclcpp::Node("mqtt_bridge", options)
{
    // get_logger().set_level(rclcpp::Logger::Level::Debug);
    setupJsonSchemaValidators();
    configure_client();
    mTimer_Connect = create_wall_timer(std::chrono::milliseconds(5000),
     std::bind(&MqttBridge::tryConnect, this), get_node_base_interface()->get_default_callback_group(), false);
    tryConnect();
}

MqttBridge::~MqttBridge()
{
    disconnect();
}

void MqttBridge::setupJsonSchemaValidators() {
    std::string jsonSchemaDir = ament_index_cpp::get_package_share_directory("mqtt_bridge") + "/schema/";
    // iterate through schema files
    for (const auto& pair : jsonSchemaFiles) {
        std::ifstream ifs(jsonSchemaDir + pair.second);
        if (!ifs.is_open()) {
            // if file cannot be opened, just skip it
            RCLCPP_ERROR_STREAM(get_logger(), "Error opening JSON schema file " << pair.second);
            continue;
        }
        rapidjson::IStreamWrapper isw(ifs);
        rapidjson::Document sd;
        if (sd.ParseStream(isw).HasParseError()) {
            // if schema file cannot be parsed, just skip it
            RCLCPP_ERROR_STREAM(get_logger(), "Error parsing JSON schema file " << pair.second);
            ifs.close();
            continue;
        }
        auto schema = std::make_shared<rapidjson::SchemaDocument>(sd);
        auto validator = std::make_shared<rapidjson::SchemaValidator>(*schema);
        auto atomic_bool = std::make_shared<std::atomic<bool>>(false);
        // add the new (schema, validator, atomic_bool) tuple to map
        // validator keeps the schema by reference, so we need to keep it alive
        jsonSchemaValidators.insert({pair.first, {schema, validator, atomic_bool}});
        ifs.close();
    }
}

bool MqttBridge::validateJSON(rapidjson::Document& doc, const std::string& schemaKey) {
    std::shared_ptr<rapidjson::SchemaValidator> validator;
    std::shared_ptr<std::atomic<bool>> atomic_bool;
    try {
        validator = std::get<1>(jsonSchemaValidators.at(schemaKey));
        atomic_bool = std::get<2>(jsonSchemaValidators.at(schemaKey));
    } catch (const std::out_of_range& e) {
        RCLCPP_WARN_STREAM_ONCE(get_logger(), "No JSON schema found at key " << schemaKey << ", skipping validation!");
        return true;
    }

    // prevent concurrent validation with the same validator
    if (atomic_bool->exchange(true)) {
        RCLCPP_WARN_STREAM(get_logger(), "Concurrent validation attempt for key " << schemaKey << ", discarding MQTT message.");
        return false;
    }

    validator->Reset();

    if (doc.Accept(*validator)) {
        atomic_bool->store(false);
        return true;
    }
    else {
        // Input JSON is invalid according to the schema
        rapidjson::StringBuffer sb;

        validator->GetInvalidSchemaPointer().StringifyUriFragment(sb);
        std::string invalidSchema(sb.GetString());

        sb.Clear();

        validator->GetInvalidDocumentPointer().StringifyUriFragment(sb);
        std::string invalidDocument(sb.GetString());

        
        RCLCPP_WARN_STREAM(get_logger(), "JSON schema validation failed (key: " << schemaKey
         << "), invalid schema: " << invalidSchema << ", keyword: " << validator->GetInvalidSchemaKeyword()
         << ", document: " << invalidDocument);

         atomic_bool->store(false);
         return false;
    }
}

void MqttBridge::on_success(const mqtt::token& asyncActionToken) {
        switch (asyncActionToken.get_type()) {
            case mqtt::token::CONNECT:
                RCLCPP_INFO(get_logger(), "Successfully connected to MQTT server.");
                cli->subscribe(SUBSCRIBED_TOPICS_NAMES, SUBSCRIBED_TOPICS_QOS, nullptr, *this);
                // create ROSTopicHandler instance
                rth = std::make_shared<ROSTopicHandler>(cli, PUBLISHER_QOS, this, this);
                // stop the connection timer
                if (!mTimer_Connect->is_canceled()) mTimer_Connect->cancel();
                break;
            case mqtt::token::SUBSCRIBE:
                RCLCPP_INFO(get_logger(), "Successfully subscribed to MQTT topics.");
                break;
            default:
                break;
        }    
}

void MqttBridge::on_failure(const mqtt::token& asyncActionToken) {
    switch (asyncActionToken.get_type()) {
        case mqtt::token::CONNECT:
            RCLCPP_ERROR_STREAM(get_logger(), "Error connecting to MQTT server: code " << asyncActionToken.get_return_code() << ", retrying soon...");
            mTimer_Connect->reset();
            break;
        case mqtt::token::SUBSCRIBE:
            RCLCPP_ERROR_STREAM(get_logger(), "Error subscribing to MQTT topics: code " << asyncActionToken.get_return_code() << ". Node may not function properly!");
            break;
        default:
            break;
    }
}

void MqttBridge::tryConnect()
{
    if (!mTimer_Connect->is_canceled()) mTimer_Connect->cancel();
    // Start the MQTT connection.
    try
    {
        RCLCPP_INFO(get_logger(), "Trying to connect to MQTT server...");
        cli->connect(connOpts, nullptr, *this);
    }
    catch (const mqtt::exception& exc)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Error connecting to MQTT server: " << exc.what() << ", retrying soon...");
    }
}

void MqttBridge::configure_client() {
    // ROS parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.read_only = true;

    declare_parameter("mqtt_server_address", SERVER_ADDRESS, param_desc);
    declare_parameter("mqtt_client_id", CLIENT_ID, param_desc);
    declare_parameter("mqtt_enable_server_cert_auth", ENABLE_SERVER_CERT_AUTH, param_desc);

    // look for SSL trust store
    bool tstore_found = false;
    if (get_parameter("mqtt_enable_server_cert_auth").as_bool()) {
        std::ifstream tstore(SSL_CA_PATH);
        if (!tstore) {
            RCLCPP_WARN_STREAM(get_logger(),
             "The trust store cannot be opened: " << SSL_CA_PATH << ", passing OS store as connection parameter!");
        } else {
            RCLCPP_INFO_STREAM(get_logger(), "Trust store found at: " << SSL_CA_PATH);
            tstore_found = true;
        }
    } else {
        RCLCPP_WARN_STREAM(get_logger(),
         "Verification of server certificate is disabled, consider adjusting your setup!");
    }

    // MQTT SSL options
    auto sslopts_builder = mqtt::ssl_options_builder()
                        .enable_server_cert_auth(get_parameter("mqtt_enable_server_cert_auth").as_bool())
                        .error_handler(std::bind(&MqttBridge::sslErrorCallback, this, std::placeholders::_1));

    if (tstore_found) sslopts_builder = sslopts_builder.trust_store(SSL_CA_PATH);
    else sslopts_builder = sslopts_builder.trust_store("/etc/ssl/certs/ca-certificates.crt");

    auto sslopts = sslopts_builder.finalize();

    // create mqtt async_client instance
    cli = std::make_shared<mqtt::async_client>(get_parameter("mqtt_server_address").as_string(),
     get_parameter("mqtt_client_id").as_string(), mqtt::create_options(MQTT_VERSION));

    RCLCPP_DEBUG_STREAM(get_logger(),
     "Taking rosparam into account, assuming MQTT broker address: " << get_parameter("mqtt_server_address").as_string()
     << ", client id: " << get_parameter("mqtt_client_id").as_string());

    // mqtt connection options
    connOpts = mqtt::connect_options_builder()
                        .user_name(MQTT_USERNAME)
                        .password(MQTT_PASSWORD)
                        .ssl(std::move(sslopts))
                        .clean_start(CLEAN_START)
                        .keep_alive_interval(std::chrono::seconds(KEEP_ALIVE))
                        .automatic_reconnect(RECONNECT_MIN_RETRY_INTERVAL, RECONNECT_MAX_RETRY_INTERVAL)
                        .finalize();

    // callback for connection lost to MQTT broker
    cli->set_connection_lost_handler([this](const std::string&)
                                    { RCLCPP_ERROR(get_logger(), "Connection to MQTT server lost. Trying to reconnect..."); });

    // callback for incoming MQTT messages
    cli->set_message_callback(std::bind(&MqttBridge::mqttMessageCallback, this, std::placeholders::_1));
}

bool MqttBridge::disconnect()
{
    // Disconnect from MQTT broker
    try
    {
        RCLCPP_INFO(get_logger(), "Disconnecting from the MQTT server...");
        // try to disconnect, timeout after 3s with an exception
        if (!cli->disconnect()->wait_for(3000)) throw mqtt::exception(-1);
        RCLCPP_INFO(get_logger(), "Disconnected from the MQTT server.");
    }
    catch (const mqtt::exception &exc)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Error disconnecting from MQTT server: " << exc.what());
        return false;
    }
    return true;
}

rclcpp::Time MqttBridge::unixMillisecondsToROSTimestamp(u_int64_t msec)
{
    rclcpp::Time timestamp((int64_t)(msec * 1000000));
    return timestamp;
}

int MqttBridge::sslErrorCallback(const std::string& msg)
{
    // this callback may segfault when using paho-cpp < v1.3.2
    // see https://github.com/eclipse-paho/paho.mqtt.cpp/issues/439
    RCLCPP_ERROR_STREAM(get_logger(), "SSL Error: " << msg);
    return 1;  // keep giving more errors, if there are any
}

void MqttBridge::mqttMessageCallback(const mqtt::const_message_ptr& mqtt_msg)
{
    auto messageTopic = mqtt_msg->get_topic();

    RCLCPP_DEBUG(get_logger(),
     "MQTT message received: [%s] on topic: [%s]", mqtt_msg->get_payload_str().c_str(), messageTopic.c_str());

    processMqttMessage(messageTopic, mqtt_msg->get_payload_str().c_str());
}

void MqttBridge::processMqttMessage(const mqtt::string& messageTopic, const char* payloadMsg)
{
    rapidjson::Document d;
    rapidjson::ParseResult ok = d.Parse(payloadMsg);

    if (!ok)
    {
        RCLCPP_WARN_STREAM(get_logger(),
         "JSON parse error: " << rapidjson::GetParseError_En(ok.Code())
         << " (" << ok.Offset() << "), discarding MQTT message.");
    } else {
        try
        {
            if (messageTopic == "RappTORS/RoverControl") {
                if (validateJSON(d, "RoverControl")) rth->publishMessage_RoverControl(createRoverControlMsg(d));
            } else if (messageTopic == "RappTORS/RoboticArmControl") {
                if (validateJSON(d, "RoboticArmControl")) rth->publishMessage_RoboticArmControl(createRoboticArmControlMsg(d));
            } else if (messageTopic == "RappTORS/SamplerControl") {
                if (validateJSON(d, "SamplerControl")) rth->publishMessage_SamplerControl(createSamplerControlMsg(d));
            } else if (messageTopic == "RappTORS/CalibrateAxis") {
                if (validateJSON(d, "CalibrateAxis")) rth->publishMessage_CalibrateAxis(createCalibrateAxisMsg(d));
            } else if (messageTopic == "RappTORS/RoverStatus") {
                if (validateJSON(d, "RoverStatus")) {
                    // custom validation logic for RoverStatus
                    using Status = rex_interfaces::msg::RoverStatus;
                    auto roverStatusMsg = createRoverStatusMsg(d);
                    if (roverStatusMsg.control_mode & Status::CONTROL_MODE_ESTOP || roverStatusMsg.communication_state != Status::COMMUNICATION_STATE_OPENED) {
                        roverStatusMsg.control_mode = Status::CONTROL_MODE_ESTOP;
                        rth->publishMessage_RoverStatus(roverStatusMsg);
                    } else if (validateRoverStatusControlMode(roverStatusMsg.control_mode)) {
                        rth->publishMessage_RoverStatus(roverStatusMsg);
                    } else {
                        RCLCPP_WARN_STREAM(get_logger(),
                            "RoverStatus control_mode validation failed, discarding MQTT message.");
                    }
                }
            } else if (messageTopic == "RappTORS/ArmAutonomy") {
                rth->publishMessage_ArmAutonomy(createArmAutonomyMsg(d));
            } else {
                RCLCPP_WARN_STREAM(get_logger(),
                 "Unknown MQTT topic: " << messageTopic << ", discarding MQTT message.");
            }
        }
        catch (const JsonAssertException& e)
        {
            RCLCPP_WARN_STREAM(get_logger(),
             "JSON assert exception, discarding MQTT message on topic " << messageTopic);
        }
    }
}

rex_interfaces::msg::RoverControl MqttBridge::createRoverControlMsg(const rapidjson::Document& d) {
    rex_interfaces::msg::RoverControl msg;

    msg.vel = d["Vel"].GetDouble();
    msg.x_axis = d["XAxis"].GetDouble();
    msg.y_axis = d["YAxis"].GetDouble();
    msg.mode = d["Mode"].GetUint();

    msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

    return msg;
}

rex_interfaces::msg::SamplerControl MqttBridge::createSamplerControlMsg(const rapidjson::Document& d) {
    rex_interfaces::msg::SamplerControl msg;

    msg.drill_movement = d["DrillMovement"].GetDouble();
    msg.platform_movement = d["PlatformMovement"].GetDouble();
    msg.drill_action = d["DrillAction"].GetDouble();
    msg.container_degrees_a = d["ContainerDegrees0"].GetDouble();
    msg.container_degrees_b = d["ContainerDegrees1"].GetDouble();
    msg.vacuum_suction = d["VacuumSuction"].GetDouble();
    msg.vacuum_a = d["VacuumA"].GetDouble();
    msg.vacuum_b = d["VacuumB"].GetDouble();

    msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

    return msg;
}

// validation based on RoverStatus spreadsheet (compatibility matrix) on RAPTORS DRIVE
bool MqttBridge::validateRoverStatusControlMode(int32_t control_mode)
{
    using Status = rex_interfaces::msg::RoverStatus;

    // Helper:
    // If 'mode' is active, return any illegally co-active modes.
    // Returns 0 if there is no conflict.
    auto hasIllegalCombination = [&](int32_t mode, int32_t illegal_mask) -> int32_t
    {
        return (control_mode & mode) ? (control_mode & illegal_mask) : 0;
    };

    //
    // ESTOP
    //
    if (int32_t offending = hasIllegalCombination(Status::CONTROL_MODE_ESTOP,
            Status::CONTROL_MODE_STOP |
            Status::CONTROL_MODE_CONFIG |
            Status::CONTROL_MODE_DRIVE |
            Status::CONTROL_MODE_ROBOTIC_ARM |
            Status::CONTROL_MODE_DEEP_SAMPLER |
            Status::CONTROL_MODE_SURFACE_SAMPLER |
            Status::CONTROL_MODE_DRIVE_AUTONOMY |
            Status::CONTROL_MODE_ROBOTIC_ARM_AUTONOMY |
            Status::CONTROL_MODE_DEEP_SAMPLER_AUTONOMY |
            Status::CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY))
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "RoverStatus - ESTOP conflict: " << offending);
        return false;
    }

    //
    // STOP
    //
    if (int32_t offending = hasIllegalCombination(Status::CONTROL_MODE_STOP,
            Status::CONTROL_MODE_CONFIG |
            Status::CONTROL_MODE_DRIVE |
            Status::CONTROL_MODE_DEEP_SAMPLER |
            Status::CONTROL_MODE_SURFACE_SAMPLER |
            Status::CONTROL_MODE_DRIVE_AUTONOMY |
            Status::CONTROL_MODE_DEEP_SAMPLER_AUTONOMY |
            Status::CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY))
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "RoverStatus - STOP conflict: " << offending);
        return false;
    }

    //
    // CONFIG
    //
    if (int32_t offending = hasIllegalCombination(Status::CONTROL_MODE_CONFIG,
            Status::CONTROL_MODE_DRIVE |
            Status::CONTROL_MODE_ROBOTIC_ARM |
            Status::CONTROL_MODE_DEEP_SAMPLER |
            Status::CONTROL_MODE_SURFACE_SAMPLER |
            Status::CONTROL_MODE_DRIVE_AUTONOMY |
            Status::CONTROL_MODE_ROBOTIC_ARM_AUTONOMY |
            Status::CONTROL_MODE_DEEP_SAMPLER_AUTONOMY |
            Status::CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY))
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "RoverStatus - CONFIG conflict: " << offending);
        return false;
    }

    //
    // DRIVE
    //
    if (int32_t offending = hasIllegalCombination(Status::CONTROL_MODE_DRIVE,
            Status::CONTROL_MODE_DEEP_SAMPLER |
            Status::CONTROL_MODE_SURFACE_SAMPLER |
            Status::CONTROL_MODE_DRIVE_AUTONOMY |
            Status::CONTROL_MODE_DEEP_SAMPLER_AUTONOMY |
            Status::CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY))
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "RoverStatus - DRIVE conflict: " << offending);
        return false;
    }

    //
    // ROBOTIC_ARM
    //
    if (int32_t offending = hasIllegalCombination(Status::CONTROL_MODE_ROBOTIC_ARM,
            Status::CONTROL_MODE_ROBOTIC_ARM_AUTONOMY))
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "RoverStatus - ROBOTIC_ARM conflict: " << offending);
        return false;
    }

    //
    // DEEP_SAMPLER
    //
    if (int32_t offending = hasIllegalCombination(Status::CONTROL_MODE_DEEP_SAMPLER,
            Status::CONTROL_MODE_DRIVE_AUTONOMY |
            Status::CONTROL_MODE_DEEP_SAMPLER_AUTONOMY |
            Status::CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY))
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "RoverStatus - DEEP_SAMPLER conflict: " << offending);
        return false;
    }

    //
    // SURFACE_SAMPLER
    //
    if (int32_t offending = hasIllegalCombination(Status::CONTROL_MODE_SURFACE_SAMPLER,
            Status::CONTROL_MODE_DRIVE_AUTONOMY |
            Status::CONTROL_MODE_DEEP_SAMPLER_AUTONOMY |
            Status::CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY))
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "RoverStatus - SURFACE_SAMPLER conflict: " << offending);
        return false;
    }

    //
    // DRIVE_AUTONOMY
    //
    if (int32_t offending = hasIllegalCombination(Status::CONTROL_MODE_DRIVE_AUTONOMY,
            Status::CONTROL_MODE_DEEP_SAMPLER_AUTONOMY |
            Status::CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY))
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "RoverStatus - DRIVE_AUTONOMY conflict: " << offending);
        return false;
    }

    //
    // ROBOTIC_ARM_AUTONOMY
    //
    // No remaining checks needed.

    //
    // DEEP_SAMPLER_AUTONOMY
    //
    // No remaining checks needed.

    // All combinations are legal.
    return true;
}

rex_interfaces::msg::RoverStatus MqttBridge::createRoverStatusMsg(const rapidjson::Document& d) {
    rex_interfaces::msg::RoverStatus msg;

    msg.communication_state = d["CommunicationState"].GetInt();
    msg.control_mode = d["ControlMode"].GetInt();

    msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

    return msg;
}

rex_interfaces::msg::CalibrateAxis MqttBridge::createCalibrateAxisMsg(const rapidjson::Document& d) {
    rex_interfaces::msg::CalibrateAxis msg;

    msg.vesc_id = d["VescID"].GetUint();
	msg.value = d["Value"].GetDouble();
	msg.action_type = d["ActionType"].GetUint();

    msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

    return msg;
}

rex_interfaces::msg::RoboticArmControl MqttBridge::createRoboticArmControlMsg(const rapidjson::Document& d) {
    rex_interfaces::msg::RoboticArmControl msg;

    msg.action_type = d["ActionType"].GetInt();
    msg.force_cartesian = d["ForceCartesian"].GetBool();
    msg.force_movement = d["ForceMovement"].GetBool();
    msg.gripper = d["Gripper"].GetDouble();

    if (d["ForwardKin"].IsNull()) {
        msg.joint_positions[0] = std::numeric_limits<double>::quiet_NaN();
        msg.joint_positions[1] = std::numeric_limits<double>::quiet_NaN();
        msg.joint_positions[2] = std::numeric_limits<double>::quiet_NaN();
        msg.joint_positions[3] = std::numeric_limits<double>::quiet_NaN();
        msg.joint_positions[4] = std::numeric_limits<double>::quiet_NaN();
        msg.joint_positions[5] = std::numeric_limits<double>::quiet_NaN();
    } else {
        auto forwardKinArray = d["ForwardKin"].GetObject();
        

        msg.joint_positions[0] = forwardKinArray["Axis1"].GetDouble();
        msg.joint_positions[1] = forwardKinArray["Axis2"].GetDouble();
        msg.joint_positions[2] = forwardKinArray["Axis3"].GetDouble();
        msg.joint_positions[3] = forwardKinArray["Axis4"].GetDouble();
        msg.joint_positions[4] = forwardKinArray["Axis5"].GetDouble();
        msg.joint_positions[5] = forwardKinArray["Axis6"].GetDouble();
    }

    if (d["InvJoystick"].IsNull()) {
        msg.velocity_cmd.linear.x = std::numeric_limits<double>::quiet_NaN();
        msg.velocity_cmd.linear.y = std::numeric_limits<double>::quiet_NaN();
        msg.velocity_cmd.linear.z = std::numeric_limits<double>::quiet_NaN();
        msg.velocity_cmd.angular.x = std::numeric_limits<double>::quiet_NaN();
        msg.velocity_cmd.angular.y = std::numeric_limits<double>::quiet_NaN();
        msg.velocity_cmd.angular.z = std::numeric_limits<double>::quiet_NaN();
    } else {
        auto invJoystickObject = d["InvJoystick"].GetObject();
        auto linearSpeedObject = invJoystickObject["LinearSpeed"].GetObject();
        auto rotationSpeedObject = invJoystickObject["RotationSpeed"].GetObject();

        msg.velocity_cmd.linear.x = linearSpeedObject["X"].GetDouble();
        msg.velocity_cmd.linear.y = linearSpeedObject["Y"].GetDouble();
        msg.velocity_cmd.linear.z = linearSpeedObject["Z"].GetDouble();
        msg.velocity_cmd.angular.x = rotationSpeedObject["X"].GetDouble();
        msg.velocity_cmd.angular.y = rotationSpeedObject["Y"].GetDouble();
        msg.velocity_cmd.angular.z = rotationSpeedObject["Z"].GetDouble();  
    }

    if (d["InvPosition"].IsNull()) {
        msg.pose_cmd.position.x = std::numeric_limits<double>::quiet_NaN();
        msg.pose_cmd.position.y = std::numeric_limits<double>::quiet_NaN();
        msg.pose_cmd.position.z = std::numeric_limits<double>::quiet_NaN();
        msg.pose_cmd.orientation.x = std::numeric_limits<double>::quiet_NaN();
        msg.pose_cmd.orientation.y = std::numeric_limits<double>::quiet_NaN();
        msg.pose_cmd.orientation.z = std::numeric_limits<double>::quiet_NaN();
        msg.pose_cmd.orientation.w = std::numeric_limits<double>::quiet_NaN();
    } else {
        auto invPositionObject = d["InvPosition"].GetObject();
        auto positionObject = invPositionObject["Position"].GetObject();
        auto rotationObject = invPositionObject["Rotation"].GetObject();

        msg.pose_cmd.position.x = positionObject["X"].GetDouble();
        msg.pose_cmd.position.y = positionObject["Y"].GetDouble();
        msg.pose_cmd.position.z = positionObject["Z"].GetDouble();
        msg.pose_cmd.orientation.x = rotationObject["X"].GetDouble();
        msg.pose_cmd.orientation.y = rotationObject["Y"].GetDouble();
        msg.pose_cmd.orientation.z = rotationObject["Z"].GetDouble();
        msg.pose_cmd.orientation.w = rotationObject["w"].GetDouble();
    }

    if (!d["Reference"].IsNull()) msg.header.frame_id = d["Reference"].GetString();

    msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

    return msg;
}

rex_interfaces::msg::RoboticArmAutonomy MqttBridge::createArmAutonomyMsg(const rapidjson::Document &d)
{
    rex_interfaces::msg::RoboticArmAutonomy msg;

    msg.action = d["action"].GetUint();
    msg.mission_id = d["mission_id"].GetString();
    const auto tasks = d["tasks"].GetArray();
    msg.tasks.reserve(tasks.Size());

    for (const auto &taskValue : tasks)
    {
        const auto taskObject = taskValue.GetObject();
        rex_interfaces::msg::RoboticArmTask task;

        task.task_type = taskObject["task_type"].GetUint();
        task.item = taskObject["item"].GetString();
        task.skip_on_failure = taskObject["skip_on_failure"].GetBool();

        msg.tasks.push_back(std::move(task));
    }

    msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

    return msg;
}
