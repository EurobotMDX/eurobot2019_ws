#include <iostream>
#include "drive_train_manager.h"

#define _DT_M_COMPLETION_TIME_SCALAR 1.5

#define _DT_M_X_AXIS_P_CONSTANT 10.0
#define _DT_M_X_AXIS_I_CONSTANT 0.5
#define _DT_M_X_AXIS_D_CONSTANT 0.2

#define _DT_M_Y_AXIS_P_CONSTANT 16.0
#define _DT_M_Y_AXIS_I_CONSTANT 0.003
#define _DT_M_Y_AXIS_D_CONSTANT 2.0

#define _DT_M_THETA_AXIS_P_CONSTANT 10.0
#define _DT_M_THETA_AXIS_I_CONSTANT 0.0
#define _DT_M_THETA_AXIS_D_CONSTANT 0.0

//f = lambda a, b : a - (int(a / b) * b)
#define _DT_M_MODULUS(A, B) (A - (((int)(A / B)) * B))
#define _DT_M_MINF(A, B) ((A) < (B) ? (A) : (B))
#define _DT_M_MAXF(A, B) ((A) > (B) ? (A) : (B))
#define _DT_M_CONSTRAINF(X, MNA, MXB) (_DT_M_MINF(MXB, _DT_M_MAXF(X, MNA)))
#define _DT_M_SIGN(A) ((A/std::abs(A)))

void print_motion_feedback(MotionFeedback &feedback)
{
    std::cout << "-- motion feedback --" << std::endl;

    printf("X "); print_pid_feedback(feedback.x_direction);
    printf("Y "); print_pid_feedback(feedback.y_direction);
    printf("T "); print_pid_feedback(feedback.theta_direction);
    printf("completed = %7.3f\n", feedback.percentage_completed);
}

DriveTrainManager::DriveTrainManager():_base_width(0.2), _wheel_diameter(0.07), _num_of_encoder_counts(4096), _absolute_gear_ratio(4554.0 / 130.0), _invert_right(true)
{
	initialize_pid();
}

DriveTrainManager::DriveTrainManager(const double base_width, const double wheel_diameter, const unsigned int encoder_counts, const double gear_ratio):_base_width(0.2), _wheel_diameter(0.07), _num_of_encoder_counts(4096), _absolute_gear_ratio(4554.0 / 130.0), _invert_right(true)
{
	initialize_pid();
}

bool DriveTrainManager::initialize_pid()
{
	_x_axis_pid_controller  = new PID(1.0, -1.0, _DT_M_X_AXIS_P_CONSTANT, _DT_M_X_AXIS_I_CONSTANT, _DT_M_X_AXIS_D_CONSTANT);
	_y_axis_pid_controller  = new PID(1.0, -1.0, _DT_M_Y_AXIS_P_CONSTANT, _DT_M_Y_AXIS_I_CONSTANT, _DT_M_Y_AXIS_D_CONSTANT);
	_theta_axis_pid_controller = new PID(1.0, -1.0, _DT_M_THETA_AXIS_P_CONSTANT, _DT_M_THETA_AXIS_I_CONSTANT, _DT_M_THETA_AXIS_D_CONSTANT);

	return true;
}

bool DriveTrainManager::initialize_pid(PID_settings &x_axis, PID_settings &y_axis, PID_settings &theta_axis)
{
	_x_axis_pid_controller     = new PID(x_axis.max, x_axis.min, x_axis.Kp, x_axis.Ki, x_axis.Kd);
	_y_axis_pid_controller     = new PID(y_axis.max, y_axis.min, y_axis.Kp, y_axis.Ki, y_axis.Kd);
	_theta_axis_pid_controller = new PID(theta_axis.max, theta_axis.min, theta_axis.Kp, theta_axis.Ki, theta_axis.Kd);

	return true;
}

bool DriveTrainManager::get_pid_settings(PID_settings &x_axis, PID_settings &y_axis, PID_settings &theta_axis)
{
	x_axis     = _x_axis_pid_controller->get_settings();
	y_axis     = _y_axis_pid_controller->get_settings();
	theta_axis = _theta_axis_pid_controller->get_settings();

	return true;
}

DriveTrainManager::~DriveTrainManager()
{
	delete _x_axis_pid_controller;
	delete _y_axis_pid_controller;
	delete _theta_axis_pid_controller;
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

	if (_invert_right)
	{
		left_wheel.reset_inverted();
		right_wheel.set_inverted();
	}
	else
	{
		left_wheel.set_inverted();
		right_wheel.reset_inverted();
	}

	return true;
}

bool DriveTrainManager::terminate() const
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

	return status;
}

double DriveTrainManager::convert_to_position(const double displacement) const
{
	static double scalar = (M_PI * _wheel_diameter) / (_num_of_encoder_counts * _absolute_gear_ratio * 4);
	return displacement / scalar;
}

double DriveTrainManager::convert_to_displacement(const double position) const
{
	static double scalar = (M_PI * _wheel_diameter) / (_num_of_encoder_counts * _absolute_gear_ratio * 4);
	return position * scalar;
}

double DriveTrainManager::convert_to_rpm(const double velocity) const
{
	static double scalar = M_PI * _wheel_diameter / ((double)60); // 60 represents one second
	return velocity / scalar;
}

double DriveTrainManager::convert_to_velocity(const double rpm) const
{
	static double scalar = M_PI * _wheel_diameter / ((double)60); // 60 represents one second
	return rpm * scalar;
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

bool DriveTrainManager::set_rpm(const int left_rpm, const int right_rpm) const
{
	bool status = true;

	if (!right_wheel.set_rpm(right_rpm * _absolute_gear_ratio))
	{
		status = false;
	}

	if (!left_wheel.set_rpm(left_rpm * _absolute_gear_ratio))
	{
		status = false;
	}
	return status;
}

bool DriveTrainManager::get_rpm(int &left_rpm, int &right_rpm) const
{
	bool status = true;

	if (!left_wheel.get_rpm(left_rpm))
	{
		status = false;
	}

	if (!right_wheel.get_rpm(right_rpm))
	{
		status = false;
	}

	left_rpm /= _absolute_gear_ratio;
	right_rpm /= _absolute_gear_ratio;

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

bool DriveTrainManager::set_motion(const double linear_velocity, const double angular_velocity) const
{
	double scalar = angular_velocity * _base_width / 2.0;
	double left_vel = linear_velocity - scalar;
	double right_vel = linear_velocity + scalar;

	return set_velocities(left_vel, right_vel);
}

bool DriveTrainManager::set_motion(const double x_velocity, const double y_velocity, const double angular_velocity) const
{
	double scalar = angular_velocity * _base_width / 2.0;
	
	double left_vel = x_velocity - scalar;
	double right_vel = x_velocity + scalar;

	return set_velocities(left_vel, right_vel);
}

bool DriveTrainManager::set_velocity(const double vel) const
{
	// static double scalar = M_PI * _wheel_diameter / ((double)60);

	// int left_rpm = vel / scalar;
	// int right_rpm = vel / scalar;

	// return set_rpm(left_rpm, right_rpm);

	int rpm = convert_to_rpm(vel);
	return set_rpm(rpm, rpm);
}

bool DriveTrainManager::set_velocities(const double left_vel, const double right_vel) const
{
	int left_rpm = convert_to_rpm(left_vel);
	int right_rpm = convert_to_rpm(right_vel);
	set_rpm(left_rpm, right_rpm);
}

bool DriveTrainManager::get_velocities(double &left_vel, double &right_vel) const
{
	int left_rpm, right_rpm;
	if ( get_rpm(left_rpm, right_rpm) )
	{
		left_vel = convert_to_velocity(left_rpm);
		right_vel = convert_to_velocity(right_rpm);

		return true;
	}

	return false;
}

bool DriveTrainManager::get_displacements(double &left_disp, double &right_disp) const
{
	int left_position, right_position;

	if ( get_position(left_position, right_position) )
	{
		left_disp = convert_to_displacement(left_position);
		right_disp = convert_to_displacement(right_position);

		return true;
	}

	return false;
}

bool DriveTrainManager::apply_motion(const double displacement, const double angular_displacement, const double translation_velocity, const double rotation_velocity, MotionFeedback &feedback) const
{
	if ( stop() == false )
	{
		return false;
	}

	if ( reset_encoders() == false )
	{
		return false;
	}

	if ( !set_velocity(0) ) // ensure that you can write to motors
	{
		return false;
	}

	bool error_detected = false;
	bool timed_out = false;

	const double direction             = (displacement > 0) ? 1.0 : -1.0;
	const double final_displacement    = std::fabs(displacement);
	const double final_rotational_displacement = _DT_M_MODULUS(angular_displacement, M_PI);

	double left_velocity        = 0;
	double left_displacement    = 0;

	double right_velocity       = 0;
	double right_displacement   = 0;
	
	double average_displacement = 0;
	
	double x_displacement       = 0;
	double y_displacement       = 0;
	double theta_displacement   = 0;

	unsigned int x_num_oscillations     = 0;
	unsigned int y_num_oscillations     = 0;
	unsigned int theta_num_oscillations = 0;
	
	double current_speed = std::fabs(translation_velocity);
	double drive_velocity = direction * current_speed;
	
	double time_elapsed = 0;
	double max_loop_time = 0;
	auto start_time = std::chrono::high_resolution_clock::now();

	if (std::fabs(final_displacement) > 0 && std::fabs(current_speed) > 0)
	{
		max_loop_time += _DT_M_COMPLETION_TIME_SCALAR * (final_displacement / current_speed);
	}
	
	if (std::fabs(final_rotational_displacement) > 0 && std::fabs(rotation_velocity) > 0)
	{
		max_loop_time += _DT_M_COMPLETION_TIME_SCALAR * ((_base_width * 0.5 * std::fabs(final_rotational_displacement)) / std::fabs(rotation_velocity));
	}

	while (!error_detected)
	{
		auto current_time = std::chrono::high_resolution_clock::now();
		time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.0;

		if (time_elapsed > max_loop_time)
		{
			timed_out = true; break;
		}

		if ( get_displacements(left_displacement, right_displacement) )
		{
			left_displacement = std::fabs(left_displacement);
			right_displacement = std::fabs(right_displacement);

			average_displacement = (left_displacement + right_displacement) / 2.0;
			theta_displacement = (left_displacement - right_displacement) / 2.0;
			x_displacement = average_displacement * std::sin(theta_displacement);
			y_displacement = average_displacement * std::cos(theta_displacement);
		}
		else
		{
			error_detected = true; break;
		}

		if ( std::fabs(y_displacement - final_displacement) <= 0.0001 &&  std::fabs(theta_displacement - final_rotational_displacement) <= 0.001)
		{
			error_detected = false; break;
		}

		{
			// y axis
			static double y_cv = 0;
			
			double new_y_cv = _y_axis_pid_controller->calculate(final_displacement, y_displacement);
			
			drive_velocity = direction * new_y_cv * current_speed;
			left_velocity  = drive_velocity;
			right_velocity = drive_velocity;

			if (_DT_M_SIGN(y_cv) != _DT_M_SIGN(new_y_cv))
			{
				y_num_oscillations++;
			}

			y_cv = new_y_cv;
		}

		{
			// x axis
			static double x_cv = 0;

			double new_x_cv = _x_axis_pid_controller->calculate(0, x_displacement);
			double x_cv_vel = (1.0 - std::fabs(new_x_cv)) * drive_velocity;

			if (new_x_cv > 0)
			{
				right_velocity = x_cv_vel;
			}
			else if (new_x_cv < 0)
			{
				left_velocity  = x_cv_vel;
			}

			if (_DT_M_SIGN(x_cv) != _DT_M_SIGN(new_x_cv))
			{
				x_num_oscillations++;
			}

			x_cv = new_x_cv;
		}

		{
			// theta axis
			static double theta_cv = 0;

			double new_theta_cv = _theta_axis_pid_controller->calculate(final_rotational_displacement, theta_displacement);
			right_velocity += -1.0 * new_theta_cv * drive_velocity;
			left_velocity  += +1.0 * new_theta_cv * drive_velocity;

			if (_DT_M_SIGN(theta_cv) != _DT_M_SIGN(new_theta_cv))
			{
				theta_num_oscillations++;
			}

			theta_cv = new_theta_cv;
		}

		set_velocities(left_velocity, right_velocity);
	}

	stop();

	if (timed_out)
	{
		time_elapsed = -1.0;
	}

	// update feedback
	feedback = {{0,0,0},{0,0,0},(0,0,0),0};

	feedback.x_direction.error = x_displacement;
	feedback.x_direction.total_time = time_elapsed;
	feedback.x_direction.num_of_oscilations = x_num_oscillations;

	feedback.y_direction.error = final_displacement - y_displacement;
	feedback.y_direction.total_time = time_elapsed;
	feedback.y_direction.num_of_oscilations = y_num_oscillations;

	feedback.theta_direction.error = theta_displacement;
	feedback.theta_direction.total_time = time_elapsed;
	feedback.theta_direction.num_of_oscilations = theta_num_oscillations;

	feedback.percentage_completed = _DT_M_CONSTRAINF(y_displacement / final_displacement, 0.0, 1.0);

	return !(error_detected || timed_out);
}

bool DriveTrainManager::translate_by(const double displacement, const double speed, MotionFeedback &feedback) const
{
	return apply_motion(displacement, 0, speed, 0.0, feedback);
}

bool DriveTrainManager::rotate_by(const double angular_displacement, const double speed, MotionFeedback &feedback) const
{
	return apply_motion(0, angular_displacement, 0.2, speed, feedback);
}


// TODO: Continue with the implementation of the modulus function
// bool DriveTrainManager::rotate_by(const double angle, const double speed) const
// {
// 	if ( stop() == false )
// 	{
// 		return false;
// 	}

// 	if ( reset_encoders() == false )
// 	{
// 		return false;
// 	}

// 	if ( !set_velocity(0) ) // ensure that you can write to motors
// 	{
// 		return false;
// 	}

// 	bool error_detected = false;

// 	const double direction       = (angle > 0) ? 1.0 : -1.0;
// 	const double target_angle    = std::fabs(angle);

// 	double left_displacement     = 0;
// 	double right_displacement    = 0;
// 	double average_displacement  = 0;
// 	double x_displacement        = 0;
// 	double y_displacement        = 0;
// 	double anglular_displacement = 0;
	
// 	double current_speed = std::fabs(speed);// % ((double) M_PI);
// 	double drive_velocity = direction * current_speed;

// 	double left_velocity   = 0;
// 	double right_velocity  = 0;

// 	while (!error_detected)
// 	{
// 		if ( get_displacements(left_displacement, right_displacement) )
// 		{
// 			left_displacement = std::fabs(left_displacement);
// 			right_displacement = std::fabs(right_displacement);

// 			average_displacement = (left_displacement + right_displacement) / 2.0;
// 			anglular_displacement = (left_displacement - right_displacement) / 2.0;
// 			x_displacement = average_displacement * std::sin(anglular_displacement);
// 			y_displacement = average_displacement * std::cos(anglular_displacement);
// 		}
// 		else
// 		{
// 			error_detected = true; break;
// 		}

// 		if ( std::fabs(anglular_displacement - target_angle) <= 0.00000308 ) //1 deg
// 		{
// 			error_detected = false; break;
// 		}

// 		{
// 			// theta axis
// 			double theta_cv = _theta_axis_pid_controller->calculate(target_angle, anglular_displacement);
// 			std::cout << "theta_cv: " << theta_cv << std::endl;
// 			drive_velocity = direction * theta_cv * current_speed;
// 		}

// 		{
// 			// control axis
// 			left_velocity = drive_velocity;
// 			right_velocity = -1.0 * drive_velocity;
// 		}

// 		std::cout << "d: " << direction << ", vel: " << drive_velocity << ", lvel: " << left_velocity << ", rvel: " << right_velocity << ", dx: " << x_displacement << ", dy: " << y_displacement << ", d0: " << anglular_displacement << std::endl;

// 		// set_velocities(drive_velocity, drive_velocity);
// 		set_velocities(left_velocity, right_velocity);
// 	}

// 	stop();

// 	std::cout << "dx: " << x_displacement << ", dy: " << y_displacement << ", d0: " << anglular_displacement << std::endl;

// 	return !(error_detected);
// }