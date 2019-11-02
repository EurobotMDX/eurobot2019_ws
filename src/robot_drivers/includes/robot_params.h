
#pragma once

#include <stdio.h>
#include <cmath>
#include <chrono>
#include <math.h>

// using meters, radians, etc. meteric only!
#define _RB_BASE_WIDTH 0.2210 // to contact center of wheels, old value 0.1714 old 0.2280
#define _RB_WHEEL_DIAMETER 0.07
#define _RB_NUM_OF_ENCODER_COUNTS 16384 //encoders 2400 //16384 // 4096 * 4
#define _RB_ABS_GEAR_RATIO (4554.0 / 130.0)
#define _RB_INVERT_RIGHT true


#define _RB_MINF(A, B) ((A) < (B) ? (A) : (B))
#define _RB_MAXF(A, B) ((A) > (B) ? (A) : (B))
#define _RB_CONSTRAINF(X, MNA, MXB) (_DT_M_MINF(MXB, _DT_M_MAXF(X, MNA)))
#define _RB_SIGN(A) ((A/std::abs(A)))

namespace robot_params
{

double convert_velocity_to_rpm(const double velocity, const double wheel_diameter)
{
    double rps = velocity / (M_PI * wheel_diameter); // revs per second
    return rps * 60.0;
}

double convert_velocity_to_rpm(const double velocity)
{
    return convert_velocity_to_rpm(velocity, _RB_WHEEL_DIAMETER);
}

double convert_rpm_to_velocity(const double rpm, const double wheel_diameter)
{
    double rps = rpm / 60.0; // revs per second
    return (rps * M_PI * wheel_diameter);
}

double convert_rpm_to_velocity(const double rpm)
{
    return convert_rpm_to_velocity(rpm, _RB_WHEEL_DIAMETER);
}

double convert_encoder_counts_to_displacement(const int counts, const double counts_per_rev, const double wheel_diameter, const double gear_ratio)
{
    double num_of_revs =  counts / counts_per_rev;
    return (num_of_revs / gear_ratio) * (M_PI * wheel_diameter);
}

double convert_encoder_counts_to_displacement(const int counts)
{
    return convert_encoder_counts_to_displacement(counts, _RB_NUM_OF_ENCODER_COUNTS, _RB_WHEEL_DIAMETER, _RB_ABS_GEAR_RATIO);
}

double convert_displacement_to_encoder_counts(const double displacement, const double counts_per_rev, const double wheel_diameter, const double gear_ratio)
{
    double num_of_revs = displacement / (M_PI * wheel_diameter);
    return num_of_revs * gear_ratio * counts_per_rev;
}

double convert_displacement_to_encoder_counts(const double displacement)
{
    return convert_displacement_to_encoder_counts(displacement, _RB_NUM_OF_ENCODER_COUNTS, _RB_WHEEL_DIAMETER, _RB_ABS_GEAR_RATIO);
}

}