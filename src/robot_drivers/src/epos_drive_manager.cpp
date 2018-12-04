#include "epos_drive_manager.h"

// TODO: remove
#include <iostream>

EposDriveManager::EposDriveManager()
{
	_inverted = false;
}

EposDriveManager::~EposDriveManager()
{
	_inverted = false;
}


bool EposDriveManager::initialize(const std::string device_name, const std::string port_name)
{
	_g_usNodeId = 1;
	_g_pKeyHandle = 0;
	_g_baudrate = 1000000;
	_g_portName = port_name; 
	_g_interfaceName = "USB"; 
	_g_deviceName = device_name; 
	_g_protocolStackName = "MAXON SERIAL V2";

	unsigned int p_pErrorCode;

	if (!_open_device(&p_pErrorCode))
	{
		return false;
	}

	if (!_set_enabled_state(&p_pErrorCode))
	{
		return false;
	}

	if (!_activate_profile_velocity_mode(&p_pErrorCode))
	{
		return false;
	}

	if (!reset_encoders())
	{
		return false;
	}

	reset_inverted();

	return true;
}

bool EposDriveManager::terminate() const
{
	bool status = true;
	unsigned int p_pErrorCode;

	if (!_set_disabled_state(&p_pErrorCode))
	{
		status = false;
	}

	status = status && _close_device(&p_pErrorCode);
	return status;
}

bool EposDriveManager::reset() const
{
	unsigned int p_rlErrorCode;
	if ( VCS_ResetDevice(_g_pKeyHandle, _g_usNodeId, &p_rlErrorCode) == 0)
	{
		return false;
	}

	if (!reset_encoders())
	{
		return false;
	}

	return true;
}

bool EposDriveManager::set_inverted()
{
	_inverted = true;
	return true;
}

bool EposDriveManager::reset_inverted()
{
	_inverted = false;
	return true;
}

bool EposDriveManager::set_rpm(const int motor_rpm) const
{
	unsigned int p_rlErrorCode;
	if ( VCS_MoveWithVelocity(_g_pKeyHandle, _g_usNodeId, (_inverted ? -motor_rpm : motor_rpm), &p_rlErrorCode) == 0)
	{
		return false;
	}

	return true;
}

bool EposDriveManager::set_position(const long position) const
{
	unsigned int p_rlErrorCode;
	if ( VCS_MoveToPosition(_g_pKeyHandle, _g_usNodeId, (_inverted ? -position : position), true, true, &p_rlErrorCode) == 0)
	{
		return false;
	}
	
	return true;
}

bool EposDriveManager::increment_position(const long delta_position) const
{
	unsigned int p_rlErrorCode;
	if ( VCS_MoveToPosition(_g_pKeyHandle, _g_usNodeId, (_inverted ? -delta_position : delta_position), false, true, &p_rlErrorCode) == 0)
	{
		return false;
	}
	
	return true;
}

bool EposDriveManager::get_rpm(int &motor_rpm) const
{
	unsigned int p_rlErrorCode;

	if ( VCS_GetVelocityIs(_g_pKeyHandle, _g_usNodeId, &motor_rpm, &p_rlErrorCode) == 0)
	{
		return false;
	}

	motor_rpm = _inverted ? -motor_rpm : motor_rpm;

	return true;
}

bool EposDriveManager::get_current(short &motor_current) const
{
	unsigned int p_rlErrorCode;
	if (VCS_GetCurrentIs(_g_pKeyHandle, _g_usNodeId, &motor_current, &p_rlErrorCode) == 0)
	{
		return false;
	}

	return true;
}

bool EposDriveManager::get_position(int &motor_position) const
{
	unsigned int p_rlErrorCode;
	if (VCS_GetPositionIs(_g_pKeyHandle, _g_usNodeId, &motor_position, &p_rlErrorCode) == 0)
	{
		return false;
	}

	motor_position = _inverted ? -motor_position : motor_position;

	return true;
}


bool EposDriveManager::reset_encoders() const
{
	bool status = true;
	unsigned int p_rlErrorCode;

	if (!_activate_homing_mode(&p_rlErrorCode))
	{
		return false;
	}

	/* resetting the current position to 0 */
	if (VCS_DefinePosition(_g_pKeyHandle, _g_usNodeId, 0, &p_rlErrorCode) == 0)
	{
		status = false;
	}

	if (!_activate_profile_velocity_mode(&p_rlErrorCode))
	{
		status = false;
	}

	return status;
}

bool EposDriveManager::stop() const
{
	unsigned int p_rlErrorCode;
	if (VCS_HaltVelocityMovement(_g_pKeyHandle, _g_usNodeId, &p_rlErrorCode) == 0)
	{
		return false;
	}

	return true;
}


bool EposDriveManager::_open_device(unsigned int* p_pErrorCode)
{
	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, _g_deviceName.c_str());
	strcpy(pProtocolStackName, _g_protocolStackName.c_str());
	strcpy(pInterfaceName, _g_interfaceName.c_str());
	strcpy(pPortName, _g_portName.c_str());

	bool status = false;
	_g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if (_g_pKeyHandle == 0)
	{
		status = false;
	}
	else if (*p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(_g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(_g_pKeyHandle, _g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(_g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(_g_baudrate==(int)lBaudrate)
					{
						status = true;
					}
				}
			}
		}
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return status;
}

bool EposDriveManager::_close_device(unsigned int* p_pErrorCode) const
{
	*p_pErrorCode = 0;
	if(VCS_CloseDevice(_g_pKeyHandle, p_pErrorCode) !=0 && *p_pErrorCode == 0)
	{
		return true;
	}

	return false;
}


bool EposDriveManager::_set_enabled_state(unsigned int* p_pErrorCode) const
{
	bool status = true;
	int isOutputFault = 0;

	if(VCS_GetFaultState(_g_pKeyHandle, _g_usNodeId, &isOutputFault, p_pErrorCode) == 0)
	{
		status = false;
	}

	if (status)
	{
		if (isOutputFault)
		{
			if (VCS_ClearFault(_g_pKeyHandle, _g_usNodeId, p_pErrorCode) == 0)
			{
				status = false;
			}
		}

		if (status)
		{
			int isOutputEnabled = 0;

			if (VCS_GetEnableState(_g_pKeyHandle, _g_usNodeId, &isOutputEnabled, p_pErrorCode) == 0)
			{
				status = false;
			}

			if (status && !isOutputEnabled)
			{
				if(VCS_SetEnableState(_g_pKeyHandle, _g_usNodeId, p_pErrorCode) == 0)
				{
					status = false;
				}
			}
		}
	}

	return status;
}

bool EposDriveManager::_set_disabled_state(unsigned int* p_pErrorCode) const
{
	if (VCS_SetDisableState(_g_pKeyHandle, _g_usNodeId, p_pErrorCode) == 0)
	{
		return false;
	}

	return true;
}


bool EposDriveManager::_activate_homing_mode(unsigned int* p_pErrorCode) const
{
	if(VCS_ActivateHomingMode(_g_pKeyHandle, _g_usNodeId, p_pErrorCode) == 0)
	{
		return false;
	}

	return true;
}

bool EposDriveManager::_activate_profile_velocity_mode(unsigned int* p_pErrorCode) const
{
	if(VCS_ActivateProfileVelocityMode(_g_pKeyHandle, _g_usNodeId, p_pErrorCode) == 0)
	{
		return false;
	}

	return true;
}

bool EposDriveManager::_activate_profile_position_mode(unsigned int* p_pErrorCode) const
{
	if(VCS_ActivateProfilePositionMode(_g_pKeyHandle, _g_usNodeId, p_pErrorCode) == 0)
	{
		return false;
	}

	return true;
}
