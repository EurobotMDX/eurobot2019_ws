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

	MotionFeedback feedback;
	double move_distance = 0.5;
	const double move_speed = 0.4;
	const double turn_speed = 0.15;

	cout << "Starting move test" << endl;

	if (status)
	{
		// move_distance = 0.5;
		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);

		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(-move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);


		// move_distance = 1.0; printf("\n");
		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);

		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(-move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);

		// move_distance = 1.5; printf("\n");
		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);

		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(-move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);

		// move_distance = 2.0; printf("\n");
		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);

		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(-move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);

		for (int i=0; i<5; i++)
		{
			move_distance = 1.5; printf("\n");
			feedback = {{0,0,0},{0,0,0},(0,0,0),0};
			my_drive_train.translate_by(move_distance, move_speed, feedback); sleep(2);
			print_motion_feedback(feedback);

			feedback = {{0,0,0},{0,0,0},(0,0,0),0};
			my_drive_train.translate_by(-move_distance, move_speed, feedback); sleep(2);
			print_motion_feedback(feedback);
		}

		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.rotate_by(M_PI_2, turn_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);
	}

	if (!my_drive_train.terminate())
	{
		cout << "Termination failed" << endl;
	}

	return 0;
}