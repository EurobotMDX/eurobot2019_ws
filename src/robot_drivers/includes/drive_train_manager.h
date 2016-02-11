#pragma once

#include <stdio.h>
#include <pid.h>
#include <cmath>
#include <chrono>
#include <math.h>
#include <epos_drive_manager.h>
#include "robot_params.h"


class DriveTrainManager
{
public:
	DriveTrainManager();
	~DriveTrainManager();

	virtual bool initialize(const std::string left_device_port_name, const std::string right_device_port_name);
	virtual bool terminate();
	virtual bool reset();

	virtual bool set_rpm(const double left_rpm, const double right_rpm) const;

	virtual bool get_rpm(double &left_rpm, double &right_rpm) const;
	virtual bool get_current(short &left_current, short &right_current) const;
	virtual bool get_position(int &left_position, int &right_position) const;

	virtual bool set_position(int left_position, int right_position) const;
	virtual bool increment_position(int delta_left_position, int delta_right_position) const;

	virtual bool set_motion(const double linear_velocity, const double angular_velocity) const;

	virtual bool reset_encoders() const;
	virtual bool stop() const;

	EposDriveManager left_wheel;
	EposDriveManager right_wheel;

	virtual bool set_velocity(const double vel) const;
	virtual bool set_velocities(const double left_vel, const double right_vel) const;
	virtual bool get_velocities(double &left_vel, double &right_vel) const;
	virtual bool get_displacements(double &left_disp, double &right_disp) const;

	// average_displacement = (left_displacement + right_displacement) / 2.0;
	// 		theta_displacement = (left_displacement - right_displacement) / 2.0;
	// 		x_displacement = average_displacement * std::sin(theta_displacement);
	// 		y_displacement = average_displacement * std::cos(theta_displacement);

	virtual bool update_odometry();
	
	double current_x;
	double current_y;
	double current_theta;

	double current_vx;
	double current_vy;
	double current_vtheta;

	bool should_run;
};