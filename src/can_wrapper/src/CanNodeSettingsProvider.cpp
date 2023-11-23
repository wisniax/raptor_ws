#include "can_wrapper/CanNodeSettingsProvider.hpp"

CanNodeSettingsProvider::CanNodeSettingsProvider()
{
	setSettingForAllDevices(0x01, 0);
	setSettingForAllDevices(0x10, 1);
	setSettingForAllDevices(0x11, 1);
}

float CanNodeSettingsProvider::getSetting(canid_t frame_id, TypeGroups typeGroup, uint8_t setting_id) const
{
	return getSetting(frame_id & CanMessage::Masks::All_Nodes, typeGroup | setting_id);
}

float CanNodeSettingsProvider::getSetting(uint8_t type_id, uint8_t setting_id) const
{
	if (type_id > 0xF || setting_id > INIT_MAX_TYPE_ID)
		return 0;
	return mNodeSettings[type_id][setting_id];
}

int8_t CanNodeSettingsProvider::setSetting(uint8_t type_id, uint8_t setting_id, float value)
{
	if (type_id > 0xF || setting_id > INIT_MAX_TYPE_ID)
		return -1;
	mNodeSettings[type_id][setting_id] = value;
	return 0;
}

int8_t CanNodeSettingsProvider::setSettingForAllDevices(uint8_t setting_id, float value)
{
	if (setting_id > INIT_MAX_TYPE_ID)
		return -1;
	for (int i = 0; i <= 0xF; i++)
		mNodeSettings[i][setting_id] = value;
	return 0;
}