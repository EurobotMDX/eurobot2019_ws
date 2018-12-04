#include "epos_drive_manager_test.h"

EposDriveManagerTest::EposDriveManagerTest()
{
	_epos_device = EposDriveManager();
}

EposDriveManagerTest::~EposDriveManagerTest()
{
}


bool EposDriveManagerTest::initialize(const std::string device_name, const std::string port_name)
{
	return _epos_device.initialize(device_name, port_name);
}

bool EposDriveManagerTest::terminate() const
{
	return _epos_device.terminate();
}

bool EposDriveManagerTest::test_rpm(const int max_rpm=200, const int interval=10) const
{
	_logf("starting rpm test");

	int motor_rpm = 0;
	bool status = true;

	if (status)
	{
		status = _epos_device.stop();
	}

	if (status)
	{
		status = _epos_device.reset_encoders();
	}

	sleep(0.1);

	if (status)
	{
		status = _epos_device.get_rpm(motor_rpm);
		if (0 != motor_rpm)
		{
			std::string s = str( boost::format("NOT EQUAL ERROR: expected motor_rpm of %1% but received %2%") % 0 % motor_rpm );
			_logf(s);
		}
	}

	// initialize motor to start speed to prevent errors due to acceleration - ask Chibuike
	_epos_device.set_rpm(-max_rpm);
	sleep(1); // wait for motor to get to set rpm

	for (int i=-max_rpm; i<max_rpm; i+=interval)
	{
		status = _epos_device.set_rpm(i);
		if (status)
		{
			sleep(0.01 * interval);
			status = _epos_device.get_rpm(motor_rpm);

			if (status)
			{
				if (std::abs(motor_rpm - i) > 50)
				{
					status = false;
					std::string s = str( boost::format("RPM CPM ERROR: expected motor_rpm of %1% but received %2%") % i % motor_rpm );
					_logf(s);
				}
			}
			else
			{
				std::string s = str( boost::format("RPM GET ERROR: Could not get device rpm"));
				_logf(s);
				break;
			}
		}
		else
		{
			std::string s = str( boost::format("RPM SET ERROR: Could not set device rpm"));
			_logf(s);
			break;
		}
	}

	if (!_epos_device.stop())
	{
		status = false;
	}

	if (!_epos_device.reset_encoders())
	{
		status = false;
	}

	_logf("done");

	return status;
}

bool gg(short &m)
{
	return true;
}

bool EposDriveManagerTest::test_current() const
{
	_logf("starting current test");

	bool status = true;
	short motor_current = 0;

	status = _epos_device.get_current(motor_current);
	if (status)
	{
		const short last_current = motor_current;

		// TODO: continue
		// PI_TEST_EQUAL(last_current, motor_current, EposDriveManager::get_current);
	}
	else
	{
		status = false;
		std::string s = str( boost::format("CURR GET ERROR: Could not get device current"));
		_logf(s);
	}

	_logf("done");

	return status;
}

bool EposDriveManagerTest::test_position(const int max_rpm=200) const
{
	_logf("starting position test");
	
	bool status = true;

	// int start_position = 0;
	// if (!_epos_device.get_position(start_position))
	// {
	// 	_logf("could not get start position");
	// 	status = false;
	// }
	// else
	// {
	// 	std::string s = str( boost::format("starting position test @  %1%") % start_position );
	// 	_logf(s);
	// }

	// if (status)
	// {
	// 	if (!_epos_device.increment_position(max_rpm, 200))
	// 	{
	// 		_logf("could not increment position");
	// 		status = false;
	// 	}
	// 	else
	// 	{
	// 		_logf("incremented position by 200");
	// 	}
	// }
	// else
	// {
	// 	_logf("refused to increment position");
	// }

	// if (status)
	// {
	// 	int end_position = 0;
	// 	if (!_epos_device.get_position(end_position))
	// 	{
	// 		_logf("could not get end position");
	// 		status = false;
	// 	}
	// 	else
	// 	{
	// 		std::string s = str( boost::format("ending position test @  %1%") % end_position );
	// 		_logf(s);
	// 	}
	// }
	// else
	// {
	// 	_logf("refused to get end position");
	// }

	_logf("done");
	
	return status;
}


bool EposDriveManagerTest::test_brakes() const
{
	_logf("starting brakes test");
	
	bool status = true;

	_logf("done");
	
	return status;
}

bool EposDriveManagerTest::_logf(std::string s) const
{
	std::cout << s << std::endl;
}

// template<class X, class Y, class Z>
// bool PI_TEST_EQUAL(const X& expected_result, X& result, Y (*f)(Z&))
// {
// 	bool status = (*f)(result);

// 	if (expected_result != result)
// 	{
// 		std::string s = str( boost::format("NOT EQUAL ERROR: expected %1% but received %2%") % expected_result % result);
// 		status = false;
// 		log(s);
// 	}

// 	return status;
// }