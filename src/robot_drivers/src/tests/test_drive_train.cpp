#include "drive_train_manager_test.h"
#include <iostream>


int main(int argc, char *argv[])
{
    DriveTrainManagerTest drive_train_test_manager;
    bool status = true;

    if ( !drive_train_test_manager.initialize("USB0", "USB1") )
    {
        std::cout << "Setup failed" << std::endl;
		status = false;
    }

    if (status)
    {
        std::cout << "Starting Tests" << std::endl;


        std::cout << std::endl << "*******************************************************************" << std::endl;
		std::cout << "-- Testing RPM" << std::endl;
		if ( !drive_train_test_manager.test_rpm(200, 1) )
		{
			std::cout << "	Failed rpm test" << std::endl;
		}
		else
		{
			std::cout << "	Passed rpm test" << std::endl;
		}
		std::cout << std::endl << "*******************************************************************" << std::endl;
    }
    else
	{
		std::cout << "Could not start tests" << std::endl;
	}

	if ( !drive_train_test_manager.terminate() )
	{
		std::cout << "Shutdown failed" << std::endl;
	}

    return 1;
}