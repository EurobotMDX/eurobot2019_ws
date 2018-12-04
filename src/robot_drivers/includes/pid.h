#ifndef _PID_H_
#define _PID_H_

#include <chrono>
#include <cmath>
#include <stdio.h>

struct PID_Feedback
{
	double error;
	double total_time; // -1 if function timed out
	unsigned int num_of_oscilations;
};

void print_pid_feedback(PID_Feedback &feedback);

struct PID_settings
{
    double max;
    double min;
    double Kp;
    double Ki;
    double Kd;
};

void print_pid_settings(PID_settings &settings);

class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double max, double min, double Kp, double Ki, double Kd );

        PID_settings get_settings();

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( const double setpoint, const double pv ) const;
        double get_dt() const;
        ~PID();

    private:
        PIDImpl *pimpl;
        PID_settings _settings;
};

#endif