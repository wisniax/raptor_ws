#include "can_wrapper/CanNodeSettingsProvider.hpp"

CanNodeSettingsProvider::CanNodeSettingsProvider()
{
	setSettingForAllDevices(CM_STMINIT_TYPEID_FAMILY_SETUP | CM_STMINIT_TYPEID_SETUP_CONTROLMODE, 0);
	setSettingForAllDevices(CM_STMINIT_TYPEID_FAMILY_MOTORCONTROL | CM_STMINIT_TYPEID_MOTORCONTROL_COMMAND, 256);
	setSettingForAllDevices(CM_STMINIT_TYPEID_FAMILY_MOTORCONTROL | CM_STMINIT_TYPEID_MOTORCONTROL_FEEDBACK, 128);
}

float CanNodeSettingsProvider::getSetting(CM_Address_t node_id, CM_StmInit_TypeId_t stm_family, CM_StmInit_TypeId_t stm_family_target) const
{
	return getSetting(node_id & CM_ADDRESS_MASK, stm_family | stm_family_target);
}

float CanNodeSettingsProvider::getSetting(CM_Address_t node_id, CM_StmInit_TypeId_t stm_target) const
{
	if (node_id > 0xF || node_id >= CM_STMINIT_TYPEID_FAMILY_MAX)
		return 0;
	return mNodeSettings[node_id][stm_target];
}

int8_t CanNodeSettingsProvider::setSetting(CM_Address_t node_id, CM_StmInit_TypeId_t stm_target, float value)
{
	if (node_id > 0xF || node_id >= CM_STMINIT_TYPEID_FAMILY_MAX)
		return -1;
	mNodeSettings[node_id][stm_target] = value;
	return 0;
}

int8_t CanNodeSettingsProvider::setSettingForAllDevices(CM_StmInit_TypeId_t stm_target, float value)
{
	if (stm_target >= CM_STMINIT_TYPEID_FAMILY_MAX)
		return -1;
	for (int i = 0; i <= 0xF; i++)
		mNodeSettings[i][stm_target] = value;
	return 0;
}