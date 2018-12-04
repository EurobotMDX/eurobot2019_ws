#pragma once

#include <string>
#include <iomanip>
#include <cassert>
#include <iostream>
#include "epos_drive_manager.h"
#include "boost/format.hpp"

template<class X, class Y, class Z>
bool PI_TEST_EQUAL(const X& expected_result, X& result, Y (*f)(Z&));

class EposDriveManagerTest
{
public:
	EposDriveManagerTest();
	~EposDriveManagerTest();

	bool initialize(const std::string device_name, const std::string port_name);
	bool terminate() const;

	// bool test_all() const;

	bool test_rpm(const int max_rpm, const int interval) const;
	bool test_current() const;
	bool test_position(const int max_rpm) const;

	bool test_brakes() const;

	bool _logf(std::string s) const;

private:
	EposDriveManager _epos_device;
};