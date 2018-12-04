#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include "pid.h"

void print_pid_feedback(PID_Feedback &feedback)
{
    printf("fdbk = {e:% 7.7f, t:% 7.5f, osc:%7d};\n", feedback.error, feedback.total_time, feedback.num_of_oscilations);
}

void print_pid_settings(PID_settings &settings)
{
    printf("settings = {mx:% 7.2f, mn:% 7.2f, kp:% 7.5f, ki:% 7.5f, kd:% 7.5f};\n", settings.max, settings.min, settings.Kp, settings.Ki, settings.Kd);
}

class PIDImpl
{
    public:
        PIDImpl( double max, double min, double Kp, double Ki, double Kd );
        ~PIDImpl();
        double calculate( double setpoint, double pv );
        double get_dt();

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;

        std::chrono::time_point<std::chrono::system_clock> _last_loop_update_time;
        void update_timestamp();
};

PID::PID( double max, double min, double Kp, double Ki, double Kd )
{
    _settings = {max, min, Kp, Ki, Kd};
    pimpl = new PIDImpl(max,min,Kp,Ki,Kd);
}

PID_settings PID::get_settings()
{
    return _settings;
}

double PID::calculate( const double setpoint, const double pv ) const
{
    return pimpl->calculate(setpoint,pv);
}

double PID::get_dt() const
{
    return pimpl->get_dt();
}

PID::~PID() 
{
    delete pimpl;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double max, double min, double Kp, double Ki, double Kd ) :
    _dt(0),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Ki(Ki),
    _Kd(Kd),
    _pre_error(0),
    _integral(0)
{
    _last_loop_update_time = std::chrono::high_resolution_clock::now();
}

double PIDImpl::calculate( const double setpoint, const double pv )
{
    // estimate _dt
    update_timestamp();

    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

void PIDImpl::update_timestamp()
{
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - _last_loop_update_time;
    _dt = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0;
    _last_loop_update_time = current_time;
}

double PIDImpl::get_dt()
{
    return _dt;
}

PIDImpl::~PIDImpl()
{
}

#endif