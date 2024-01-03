#ifndef CAN_WRAPPER_CAN_NODE_SETTINGS_PROVIDER_HPP
#define CAN_WRAPPER_CAN_NODE_SETTINGS_PROVIDER_HPP

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <linux/can.h>
#include "CM/CM.h"

/**
 * @brief Provides access to CAN node settings.
 */
class CanNodeSettingsProvider
{
public:

	CanNodeSettingsProvider();

	/**
	 * @brief Gets a setting value for a specific frame ID and type group.
	 * @param frame_id The frame ID that gets properly masked to device_id.
	 * @param typeGroup The type group family.
	 * @param setting_id The setting ID.
	 * @return The setting value.
	 */
	float getSetting(CM_Address_t node_id, CM_StmInit_TypeId_t stm_family, CM_StmInit_TypeId_t stm_family_target) const;

	/**
	 * @brief Gets a setting value for a specific type ID and setting ID.
	 * @param node_id The CAN node ID.
	 * @param type_id The type_id from documentation.
	 * @return The setting value.
	 */
	float getSetting(CM_Address_t node_id, CM_StmInit_TypeId_t stm_target) const;

	/**
	 * @brief Sets a setting value for a specific type ID and setting ID.
	 * @param node_id The CAN node ID.
	 * @param type_id The type_id from documentation.
	 * @param value The value to set.
	 * @return 0 if successful, -1 otherwise.
	 */
	int8_t setSetting(CM_Address_t node_id, CM_StmInit_TypeId_t stm_target, float value);

	/**
	 * @brief Sets a setting value for all devices for a specific setting ID.
	 * @param setting_id The setting ID.
	 * @param value The value to set.
	 * @return 0 if successful, -1 otherwise.
	 */
	int8_t setSettingForAllDevices(CM_StmInit_TypeId_t stm_target, float value);

private:

	static constexpr int NODE_SETTINGS_ARRSTART = CM_ADDRESS_MAX - CM_ADDRESS_COUNT;

	/**
	 * @brief The node settings array.
	 */
	std::unordered_map<CM_StmInit_TypeId_t,float> mNodeSettings[CM_ADDRESS_COUNT];

};

#endif // CAN_WRAPPER_CAN_NODE_SETTINGS_PROVIDER_HPP
