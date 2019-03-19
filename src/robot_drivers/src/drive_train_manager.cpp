#include <iostream>
#include "drive_train_manager.h"

DriveTrainManager::DriveTrainManager()
{
	current_x = 0;
	current_y = 0;
	current_theta = 0;

	current_vx = 0;
	current_vy = 0;
	current_vtheta = 0;

	should_run = false;
}

DriveTrainManager::~DriveTrainManager()
{
	current_x = 0;
	current_y = 0;
	current_theta = 0;

	current_vx = 0;
	current_vy = 0;
	current_vtheta = 0;

	should_run = false;
}

bool DriveTrainManager::initialize(const std::string left_device_port_name="USB0", const std::string right_device_port_name="USB1")
{
	// TODO: remove
	left_wheel  = EposDriveManager();
	right_wheel = EposDriveManager();

	bool l_status = false;
	bool r_status = false;

	for (int i=0; i<5; i++)
	{
		l_status = left_wheel.initialize("EPOS4", left_device_port_name);
		if (l_status) break;

		sleep(1);
	}

	for (int i=0; i<5; i++)
	{
		r_status = right_wheel.initialize("EPOS4", right_device_port_name);
		if (r_status) break;

		sleep(1);
	}

	if (!(l_status && r_status))
	{
		return false;
	}

	if (_RB_INVERT_RIGHT)
	{
		left_wheel.reset_inverted();
		right_wheel.set_inverted();
	}
	else
	{
		left_wheel.set_inverted();
		right_wheel.reset_inverted();
	}

	should_run = true;
	return true;
}

bool DriveTrainManager::terminate()
{
	bool status = true;

	if (!left_wheel.terminate())
	{
		status = false;
	}

	if (!right_wheel.terminate())
	{
		status = false;
	}

	should_run = false;
	return status;
}

bool DriveTrainManager::reset() const
{
	bool status = true;

	if (!left_wheel.reset())
	{
		status = false;
	}

	if (!right_wheel.reset())
	{
		status = false;
	}

	return status;
}

bool DriveTrainManager::set_rpm(const double left_rpm, const double right_rpm) const
{
	// scale the rpm by the gear ratio of the gear train attached to the robot or else you will have to
	// to send RPMs like 11000... etc. {ask chibuike}

	bool status = true;

	int new_left_rpm  = _RB_ABS_GEAR_RATIO * left_rpm;
	int new_right_rpm = _RB_ABS_GEAR_RATIO * right_rpm;

	if (!left_wheel.set_rpm(new_left_rpm))
	{
		status = false;
	}

	if (!right_wheel.set_rpm(new_right_rpm))
	{
		status = false;
	}

	return status;
}

bool DriveTrainManager::get_rpm(double &left_rpm, double &right_rpm) const
{
	bool status = true;

	int current_left_rpm;
	int current_right_rpm;

	if (!left_wheel.get_rpm(current_left_rpm))
	{
		status = false;
	}

	if (!right_wheel.get_rpm(current_right_rpm))
	{
		status = false;
	}

	left_rpm  = current_left_rpm / _RB_ABS_GEAR_RATIO;
	right_rpm = current_right_rpm / _RB_ABS_GEAR_RATIO;

	return status;
}

bool DriveTrainManager::get_current(short &left_current, short &right_current) const
{
	bool status = true;

	if (!left_wheel.get_current(left_current))
	{
		status = false;
	}

	if (!right_wheel.get_current(right_current))
	{
		status = false;
	}

	return status;
}

bool DriveTrainManager::get_position(int &left_position, int &right_position) const
{
	bool status = true;

	if (!left_wheel.get_position(left_position))
	{
		status = false;
	}

	if (!right_wheel.get_position(right_position))
	{
		status = false;
	}

	return status;
}

bool DriveTrainManager::set_position(int left_position, int right_position) const
{
	bool status = true;

	if (!left_wheel.set_position(left_position))
	{
		status = false;
	}

	if (!right_wheel.set_position(right_position))
	{
		status = false;
	}

	return status;
}

bool DriveTrainManager::increment_position(int delta_left_position, int delta_right_position) const
{
	bool status = true;

	if (!left_wheel.increment_position(delta_left_position))
	{
		status = false;
	}

	if (!right_wheel.increment_position(delta_right_position))
	{
		status = false;
	}

	return status;
}

bool DriveTrainManager::set_motion(const double linear_velocity, const double angular_velocity) const
{
	double scalar = angular_velocity * (_RB_BASE_WIDTH / 2.0);
	return set_velocities(linear_velocity - scalar, linear_velocity + scalar);
}

bool DriveTrainManager::reset_encoders() const
{
	bool status = true;

	if (!left_wheel.reset_encoders())
	{
		status = false;
	}

	if (!right_wheel.reset_encoders())
	{
		status = false;
	}

	return status;
}

bool DriveTrainManager::stop() const
{
	bool status = true;

	if (!left_wheel.stop())
	{
		status = false;
	}

	if (!right_wheel.stop())
	{
		status = false;
	}

	return status;
}

bool DriveTrainManager::update_odometry()
{
	static int last_left_position = 0;
	static int last_right_position = 0;
	static auto last_update_time = std::chrono::high_resolution_clock::now();

	int current_left_position;
	int current_right_position;
	if ( !get_position(current_left_position, current_right_position) )
	{
		return false;
	}

	double last_left_disp = robot_params::convert_encoder_counts_to_displacement(last_left_position);
	double last_right_disp = robot_params::convert_encoder_counts_to_displacement(last_right_position);

	double current_left_disp = robot_params::convert_encoder_counts_to_displacement(current_left_position);
	double current_right_disp = robot_params::convert_encoder_counts_to_displacement(current_right_position);

	auto current_time = std::chrono::high_resolution_clock::now();
	double d_left = current_left_disp - last_left_disp;
	double d_right = current_right_disp - last_right_disp;
	double theta = (current_left_disp - current_right_disp) / _RB_BASE_WIDTH;
	
	double r = (d_left + d_right) / 2.0;
	double dx = r * std::sin(theta);
	double dy = r * std::cos(theta);
	double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update_time).count() / 1000.0;

	current_vx = dx / dt;
	current_vy = dy / dt;
	current_vtheta = (theta - current_theta) / dt;

	current_x += dx;
	current_y += dy;
	current_theta = theta;
	
	last_update_time = current_time;
	last_left_position = current_left_position;
	last_right_position = current_right_position;

	return true;
}

bool DriveTrainManager::set_velocity(const double vel) const
{
	double rpm = robot_params::convert_velocity_to_rpm(vel);
	return set_rpm(rpm, rpm);
}

bool DriveTrainManager::set_velocities(const double left_vel, const double right_vel) const
{
	set_rpm(robot_params::convert_velocity_to_rpm(left_vel), robot_params::convert_velocity_to_rpm(right_vel));
}

bool DriveTrainManager::get_velocities(double &left_vel, double &right_vel) const
{
	double left_rpm, right_rpm;
	if ( get_rpm(left_rpm, right_rpm) )
	{
		left_vel = robot_params::convert_rpm_to_velocity(left_rpm);
		right_vel = robot_params::convert_rpm_to_velocity(right_rpm);

		return true;
	}

	return false;
}

bool DriveTrainManager::get_displacements(double &left_disp, double &right_disp) const
{
	int left_position, right_position;
	if ( get_position(left_position, right_position) )
	{
		left_disp = robot_params::convert_encoder_counts_to_displacement(left_position);
		right_disp = robot_params::convert_encoder_counts_to_displacement(right_position);

		return true;
	}

	return false;
}