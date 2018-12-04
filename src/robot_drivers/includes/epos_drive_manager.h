#pragma once

#include "Definitions.h"
#include <string.h>
#include <string>
#include "boost/format.hpp"

#define _EPOS_DRIVE_MANAGER_MAX_NUM_OF_TRIALS 5

class EposDriveManager
{
public:
	EposDriveManager();
	~EposDriveManager();

	//example ---> my_device.initialize("EPOS4", "USB0");
	virtual bool initialize(const std::string device_name, const std::string port_name);
	virtual bool terminate() const;
	virtual bool reset() const;

	virtual bool set_rpm(const int motor_rpm) const;
	virtual bool set_position(const long position) const;
	virtual bool increment_position(const long delta_position) const;

	virtual bool get_rpm(int &motor_rpm) const;
	virtual bool get_current(short &motor_current) const;
	virtual bool get_position(int &motor_position) const;

	virtual bool reset_encoders() const;
	virtual bool stop() const; // apply brakes to the wheels

	bool set_inverted();
	bool reset_inverted();

protected:
	virtual bool _open_device(unsigned int* p_pErrorCode);
	virtual bool _close_device(unsigned int* p_pErrorCode) const;

	virtual bool _set_enabled_state(unsigned int* p_pErrorCode) const;
	virtual bool _set_disabled_state(unsigned int* p_pErrorCode) const;

	virtual bool _activate_homing_mode(unsigned int* p_pErrorCode) const;
	virtual bool _activate_profile_velocity_mode(unsigned int* p_pErrorCode) const;
	virtual bool _activate_profile_position_mode(unsigned int* p_pErrorCode) const;

private:
	int _g_baudrate;
	void* _g_pKeyHandle;
	std::string _g_portName;
	std::string _g_deviceName;
	unsigned short _g_usNodeId;
	std::string _g_interfaceName;
	std::string _g_protocolStackName;

	bool _inverted;
};