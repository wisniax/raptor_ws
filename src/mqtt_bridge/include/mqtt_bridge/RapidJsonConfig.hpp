#ifndef MQTT_BRIDGE__RAPIDJSONCONFIG_HPP_
#define MQTT_BRIDGE__RAPIDJSONCONFIG_HPP_

/*
Copyright (c) 2024-2026 Bartosz Mazurkiewicz.
Licensed under the MIT License.
*/

#include <exception>

#define RAPIDJSON_HAS_STDSTRING 1
class JsonAssertException : public std::exception
{
public:
    char const* what()
    {
        return "JSON assert exception";
    }
};

// custom assert(x) for rapidjson library so that it doesn't abort the program on errors
#define RAPIDJSON_ASSERT(x) (static_cast<bool>(x) ? void(0) : throw JsonAssertException())

#endif  // MQTT_BRIDGE__RAPIDJSONCONFIG_HPP_
