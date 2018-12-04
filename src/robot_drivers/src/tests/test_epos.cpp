#include "epos_drive_manager_test.h"
#include <iostream>


int main(int argc, char *argv[])
{
	EposDriveManagerTest epos_device;
	bool status = true;


	if ( !epos_device.initialize("EPOS4", "USB0") )
	{
		std::cout << "Setup failed" << std::endl;
		status = false;
	}

	if (status)
	{
		std::cout << "Starting Tests" << std::endl;

		std::cout << std::endl << "*******************************************************************" << std::endl;
		std::cout << "-- Testing RPM" << std::endl;
		if ( !epos_device.test_rpm(200 * (4554.0 / 130.0), 10) )
		{
			std::cout << "	Failed rpm test" << std::endl;
		}
		else
		{
			std::cout << "	Passed rpm test" << std::endl;
		}
		std::cout << std::endl << "*******************************************************************" << std::endl;

		std::cout << std::endl << "*******************************************************************" << std::endl;
		std::cout << "-- Testing Current" << std::endl;
		if ( !epos_device.test_current() )
		{
			std::cout << "	Failed current test" << std::endl;
		}
		else
		{
			std::cout << "	Passed current test" << std::endl;
		}
		std::cout << std::endl << "*******************************************************************" << std::endl;

		std::cout << std::endl << "*******************************************************************" << std::endl;
		std::cout << "-- Testing Position" << std::endl;
		if ( !epos_device.test_position(200) )
		{
			std::cout << "	Failed position test" << std::endl;
		}
		else
		{
			std::cout << "	Passed position test" << std::endl;
		}
		std::cout << std::endl << "*******************************************************************" << std::endl;

		std::cout << std::endl << "*******************************************************************" << std::endl;
		std::cout << "-- Testing Brakes" << std::endl;
		if ( !epos_device.test_brakes() )
		{
			std::cout << "	Failed brakes test" << std::endl;
		}
		else
		{
			std::cout << "	Passed brakes test" << std::endl;
		}
		std::cout << std::endl << "*******************************************************************" << std::endl;
	}
	else
	{
		std::cout << "Could not start tests" << std::endl;
	}

	if ( !epos_device.terminate() )
	{
		std::cout << "Shutdown failed" << std::endl;
	}

	return 1;
}