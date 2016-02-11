#include "drive_train_manager.h"
#include "boost/format.hpp"
#include <iostream>

// use sleep(0.1 * interval); to delay in seconds

using namespace std;
int main(int argc, char *argv[])
{
	DriveTrainManager my_drive_train;
	bool status = true;

	if (!my_drive_train.initialize("USB0", "USB1"))
	{
		status = false;
		cout << "Initialization failed" << endl;
	}
	
	const double move_speed = 0.4;
	const double turn_speed = 0.15;

	cout << "Starting move test" << endl;

	if (status)
	{
		// pass
	}

	if (!my_drive_train.terminate())
	{
		cout << "Termination failed" << endl;
	}

	return 0;
}