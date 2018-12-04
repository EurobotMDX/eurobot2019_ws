#pragma once

#include <string>
#include <iomanip>
#include <cassert>
#include <iostream>
#include "boost/format.hpp"
#include "drive_train_manager.h"

class DriveTrainManagerTest
{
public:
	DriveTrainManagerTest();
	~DriveTrainManagerTest();

	bool initialize(const std::string left_device_port_name, const std::string right_device_port_name);
	bool terminate() const;

	// bool test_all() const;

	bool test_rpm(const int max_rpm, const int interval) const;
	bool test_current() const;
	bool test_position(const int max_rpm) const;

	bool test_brakes() const;

	bool _logf(std::string s) const;

private:
    DriveTrainManager *_drive_train_manager;
};