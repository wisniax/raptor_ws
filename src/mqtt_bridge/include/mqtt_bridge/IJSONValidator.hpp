#ifndef MQTT_BRIDGE__IJSONVALIDATOR_HPP_
#define MQTT_BRIDGE__IJSONVALIDATOR_HPP_

/*
Copyright (c) 2024-2026 Bartosz Mazurkiewicz.
Licensed under the MIT License.
*/

#include <string>
#include <rapidjson/document.h>

class IJSONValidator {
public:
    virtual ~IJSONValidator() = default;
    virtual bool validateJSON(rapidjson::Document& doc, const std::string& schemaKey) = 0;
};

#endif  // MQTT_BRIDGE__IJSONVALIDATOR_HPP_