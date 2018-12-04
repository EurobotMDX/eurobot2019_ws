#include "drive_train_manager_test.h"

DriveTrainManagerTest::DriveTrainManagerTest()
{
	_drive_train_manager = new DriveTrainManager();
}

DriveTrainManagerTest::~DriveTrainManagerTest()
{
}

bool DriveTrainManagerTest::initialize(const std::string left_device_port_name, const std::string right_device_port_name)
{
	return _drive_train_manager->initialize(left_device_port_name, right_device_port_name);
}

bool DriveTrainManagerTest::terminate() const
{
	return _drive_train_manager->terminate();
}

bool DriveTrainManagerTest::test_rpm(const int max_rpm=200, const int interval=10) const
{
    _logf("starting rpm test");

    int left_motor_rpm = 0;
    int right_motor_rpm = 0;
	bool status = true;

    if (status)
	{
		status = _drive_train_manager->stop();
	}

    if (status)
	{
		status = _drive_train_manager->reset_encoders();
	}

    sleep(0.1);
    
    if (status)
	{
		status = _drive_train_manager->get_rpm(left_motor_rpm, right_motor_rpm);
		if ((0 != left_motor_rpm) || (0 != right_motor_rpm))
		{
			std::string s;
            
            s = str( boost::format("LEFT: NOT EQUAL ERROR: expected motor_rpm of %1% but received %2%") % 0 % left_motor_rpm );
			_logf(s);

            s = str( boost::format("RIGHT: NOT EQUAL ERROR: expected motor_rpm of %1% but received %2%") % 0 % right_motor_rpm );
			_logf(s);
		}
	}

    // initialize motor to <@max_rpm START SPEED> to prevent errors due to acceleration - ask Chibuike
	_drive_train_manager->set_rpm(-max_rpm, -max_rpm);
    sleep(1); // wait for motors to get to set rpm

    for (int i=-max_rpm; i<max_rpm; i+=interval)
	{
		status = _drive_train_manager->set_rpm(i, i);
		if (status)
		{
			sleep(0.01 * interval);
			status = _drive_train_manager->get_rpm(left_motor_rpm, right_motor_rpm);

			if (status)
			{
				if (std::abs(left_motor_rpm - i) > (0.01 * max_rpm))
				{
					status = false;
					std::string s = str( boost::format("LEFT RPM CPM ERROR: expected motor_rpm of %1% but received %2%") % i % left_motor_rpm );
					_logf(s);
				}

                if (std::abs(right_motor_rpm - i) > (0.01 * max_rpm))
				{
					status = false;
					std::string s = str( boost::format("RIGHT RPM CPM ERROR: expected motor_rpm of %1% but received %2%") % i % right_motor_rpm );
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

    if (!_drive_train_manager->stop())
	{
		status = false;
	}

	if (!_drive_train_manager->reset_encoders())
	{
		status = false;
	}

	_logf("done");

	return status;
}

bool DriveTrainManagerTest::_logf(std::string s) const
{
	std::cout << s << std::endl;
}