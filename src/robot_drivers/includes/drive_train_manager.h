#pragma once

#include <stdio.h>
#include <pid.h>
#include <cmath>
#include <chrono>
#include <math.h>
#include <epos_drive_manager.h>

struct MotionFeedback
{
	PID_Feedback x_direction;
	PID_Feedback y_direction;
	PID_Feedback theta_direction;

	double percentage_completed;
};

void print_motion_feedback(MotionFeedback &feedback);

class DriveTrainManager
{
private:
	const bool _invert_right;
	const double _base_width;
	const double _wheel_diameter;
	const double _absolute_gear_ratio;
	const unsigned int _num_of_encoder_counts;

	PID *_x_axis_pid_controller;
	PID *_y_axis_pid_controller;
	PID *_theta_axis_pid_controller;

public:
	DriveTrainManager();
	~DriveTrainManager();

	DriveTrainManager(const double base_width, const double wheel_diameter, const unsigned int encoder_counts, const double gear_ratio);

	virtual bool initialize(const std::string left_device_port_name, const std::string right_device_port_name);
	virtual bool terminate() const;
	virtual bool reset() const;

	virtual bool initialize_pid();
	virtual bool initialize_pid(PID_settings &x_axis, PID_settings &y_axis, PID_settings &theta_axis);
	virtual bool get_pid_settings(PID_settings &x_axis, PID_settings &y_axis, PID_settings &theta_axis);

	virtual bool set_rpm(const int left_rpm, const int right_rpm) const;

	virtual bool get_rpm(int &left_rpm, int &right_rpm) const;
	virtual bool get_current(short &left_current, short &right_current) const;
	virtual bool get_position(int &left_position, int &right_position) const;

	virtual bool set_position(int left_position, int right_position) const;
	virtual bool increment_position(int delta_left_position, int delta_right_position) const;

	virtual bool reset_encoders() const;
	virtual bool stop() const;

	EposDriveManager left_wheel;
	EposDriveManager right_wheel;

	virtual bool set_velocity(const double vel) const;
	virtual bool set_velocities(const double left_vel, const double right_vel) const;
	virtual bool get_velocities(double &left_vel, double &right_vel) const;
	virtual bool get_displacements(double &left_disp, double &right_disp) const;

	virtual bool set_motion(const double linear_velocity, const double angular_velocity) const;
	virtual bool set_motion(const double x_velocity, const double y_velocity, const double angular_velocity) const;

	virtual bool apply_motion(const double displacement, const double angular_displacement, const double translation_veloctiy, const double rotation_velocity, MotionFeedback &feedback) const;

	virtual bool translate_by(const double displacement, const double speed, MotionFeedback &feedback) const;
	virtual bool rotate_by(const double angular_displacement, const double speed, MotionFeedback &feedback) const;
	//virtual bool rotate_by(const double angle, const double speed) const;

	virtual double convert_to_position(const double displacement) const;
	virtual double convert_to_displacement(const double position) const;
	virtual double convert_to_rpm(const double velocity) const;
	virtual double convert_to_velocity(const double rpm) const;
};