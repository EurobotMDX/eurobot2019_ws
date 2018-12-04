#include "drive_train_manager.h"
#include "boost/format.hpp"
#include <iostream>

#define _T_DT_AVG(A, B) ((A + B) / 2.0)

void auto_tune_drive_train_x(DriveTrainManager &drive_train, const double move_distance, const double move_speed, double &final_kp, double &final_ki, double &final_kd) // for x direction
{
    const int num_of_test_loop = 3;
    const double max_kp = 10;

    double kp_step = 0.001;
    double kd_step = 0.0005;
    double ki_step = 0.01;

    double kp = kp_step;
    double ki = 0;
    double kd = 0;

    double stable_kp = kp_step;
    double stable_ki = 0;
    double stable_kd = 0;
    unsigned int kd_tune_counter = 0;
    const unsigned int max_kd_tune_counter = 10;

    double lowest_error = 1000;
    unsigned int ki_tune_counter = 0;
    const unsigned int max_ki_tune_counter = 20;

    PID_settings x_settings  = {1.0, -1.0, kp, ki, kd};
    PID_settings y_settings  = {1.0, -1.0, 0, 0, 0};
    PID_settings th_settings = {1.0, -1.0, 0, 0, 0};
    
    while (true) // find Kp and Kd
    {
        if (kp >= max_kp)
        {
            printf("Kp exceeds max.\n");
            break;
        }
        
        if (kd_tune_counter >= max_kd_tune_counter)
        {
            printf("kd_tune_counter exceeds max.\n");
            break;
        }

        // update pid
        drive_train.get_pid_settings(x_settings, y_settings, th_settings);
        PID_settings x_settings  = {1.0, -1.0, kp, ki, kd};
        drive_train.initialize_pid(x_settings, y_settings, th_settings);

        double avg_time = 0;
        double avg_error = 0;
        unsigned int avg_osc = 0;

        for (int i=0; i<num_of_test_loop; i++)
        {
            MotionFeedback feedback = {{0,0,0},{0,0,0},(0,0,0),0};
            drive_train.apply_motion(move_distance, 0, move_speed, 0.1, feedback);

            if (feedback.x_direction.total_time < 0 || avg_time < 0)
            {
                avg_time = -1.0;
            }
            else
            {
                avg_time  = _T_DT_AVG(avg_time , feedback.x_direction.total_time);
            }

            avg_error = _T_DT_AVG(avg_error, feedback.x_direction.error);
            avg_osc   = _T_DT_AVG(avg_osc  , feedback.x_direction.num_of_oscilations);
        }

        printf("Xfdbk %4.4f, %4.4f, %4.4f = {e:% 7.3f, t:% 7.3f, osc:%7d};\n", kp, ki, kd, avg_error, avg_time, avg_osc);

        if (std::fabs(avg_error) < lowest_error)
        {
            stable_ki = ki;
            lowest_error = std::fabs(avg_error);
        }
        
        if (avg_osc > 0)
        {
            kd += kd_step;
            kd_tune_counter++;
        }
        else
        {
            stable_kp = kp;
            stable_kd = kd;

            kd_tune_counter = 0;
            kp += kp_step;
        }
    }

    kp = stable_kp;
    kd = stable_kd;
    ki = ki_step > stable_ki ? ki_step : stable_ki;

    while (true) // finds ki
    {
        if (ki_tune_counter >= max_ki_tune_counter)
        {
            printf("ki_tune_counter exceeds max.\n");
            break;
        }

        drive_train.get_pid_settings(x_settings, y_settings, th_settings);
        x_settings  = {1.0, -1.0, stable_kp, ki, stable_kd};
        drive_train.initialize_pid(x_settings, y_settings, th_settings);

        double avg_time = 0;
        double avg_error = 0;
        unsigned int avg_osc = 0;

        for (int i=0; i<num_of_test_loop; i++)
        {
            MotionFeedback feedback = {{0,0,0},{0,0,0},(0,0,0),0};
            drive_train.apply_motion(move_distance, 0, move_speed, 0.1, feedback);

            if (feedback.x_direction.total_time < 0 || avg_time < 0)
            {
                avg_time = -1.0;
            }
            else
            {
                avg_time  = _T_DT_AVG(avg_time , feedback.x_direction.total_time);
            }

            avg_error = _T_DT_AVG(avg_error, feedback.x_direction.error);
            avg_osc   = _T_DT_AVG(avg_osc  , feedback.x_direction.num_of_oscilations);
        }

        printf("Xfdbk %4.4f, %4.4f, %4.4f = {e:% 7.5f, t:% 7.3f, osc:%7d};\n", kp, ki, kd, avg_error, avg_time, avg_osc);

        if (std::fabs(avg_error) < lowest_error)
        {
            stable_ki = ki;
            lowest_error = std::fabs(avg_error);
        }

        ki_tune_counter++;
        ki += ki_step;
    }

    {
        printf("USING BEST PID\n");
        drive_train.get_pid_settings(x_settings, y_settings, th_settings);
        x_settings  = {1.0, -1.0, stable_kp, stable_ki, stable_kd};
        drive_train.initialize_pid(x_settings, y_settings, th_settings);

        double avg_time = 0;
        double avg_error = 0;
        unsigned int avg_osc = 0;

        for (int i=0; i<num_of_test_loop; i++)
        {
            MotionFeedback feedback = {{0,0,0},{0,0,0},(0,0,0),0};
            drive_train.apply_motion(move_distance, 0, move_speed, 0.1, feedback);

            if (feedback.x_direction.total_time < 0 || avg_time < 0)
            {
                avg_time = -1.0;
            }
            else
            {
                avg_time  = _T_DT_AVG(avg_time , feedback.x_direction.total_time);
            }

            avg_error = _T_DT_AVG(avg_error, feedback.x_direction.error);
            avg_osc   = _T_DT_AVG(avg_osc  , feedback.x_direction.num_of_oscilations);
        }

        printf("Xfdbk %4.4f, %4.4f, %4.4f = {e:% 7.5f, t:% 7.3f, osc:%7d};\n", stable_kp, stable_ki, stable_kd, avg_error, avg_time, avg_osc);
    }

    final_kp = stable_kp;
    final_ki = stable_ki;
    final_kd = stable_kd;
}


void auto_tune_drive_train_y(DriveTrainManager &drive_train, const double move_distance, const double move_speed,double &final_kp, double &final_ki, double &final_kd) // for y direction
{
    const int num_of_test_loop = 3;
    const double max_kp = 200;

    double kp_step = 1;
    double kd_step = 1;
    double ki_step = 0.001;

    double kp = kp_step;
    double ki = 0;
    double kd = 0;

    double stable_kp = kp_step;
    double stable_ki = 0;
    double stable_kd = 0;
    unsigned int kd_tune_counter = 0;
    const unsigned int max_kd_tune_counter = 10;

    double lowest_error = 1000;
    unsigned int ki_tune_counter = 0;
    const unsigned int max_ki_tune_counter = 20;

    PID_settings x_settings  = {1.0, -1.0, 0, 0, 0};
    PID_settings y_settings  = {1.0, -1.0, kp, ki, kd};
    PID_settings th_settings = {1.0, -1.0, 0, 0, 0};
    
    while (true) // find Kp and Kd
    {
        if (kp >= max_kp)
        {
            printf("Kp exceeds max.\n");
            break;
        }
        
        if (kd_tune_counter >= max_kd_tune_counter)
        {
            printf("kd_tune_counter exceeds max.\n");
            break;
        }

        // update pid
        // drive_train.get_pid_settings(x_settings, y_settings, th_settings);
        PID_settings y_settings  = {1.0, -1.0, kp, ki, kd};
        drive_train.initialize_pid(x_settings, y_settings, th_settings);

        double avg_time = 0;
        double avg_error = 0;
        unsigned int avg_osc = 0;

        for (int i=0; i<num_of_test_loop; i++)
        {
            MotionFeedback feedback = {{0,0,0},{0,0,0},(0,0,0),0};
            drive_train.apply_motion(move_distance, 0, move_speed, 0.1, feedback);

            if (feedback.y_direction.total_time < 0 || avg_time < 0)
            {
                avg_time = -1.0;
            }
            else
            {
                avg_time  = _T_DT_AVG(avg_time , feedback.y_direction.total_time);
            }

            avg_error = _T_DT_AVG(avg_error, feedback.y_direction.error);
            avg_osc   = _T_DT_AVG(avg_osc  , feedback.y_direction.num_of_oscilations);
        }

        printf("Yfdbk %4.1f, %4.1f, %4.1f = {e:% 7.3f, t:% 7.3f, osc:%7d};\n", kp, ki, kd, avg_error, avg_time, avg_osc);

        if (std::fabs(avg_error) < lowest_error)
        {
            stable_ki = ki;
            lowest_error = std::fabs(avg_error);
        }
        
        if (avg_osc > 0)
        {
            kd += kd_step;
            kd_tune_counter++;
        }
        else
        {
            stable_kp = kp;
            stable_kd = kd;

            kd_tune_counter = 0;
            kp += kp_step;
        }
    }

    kp = stable_kp;
    kd = stable_kd;
    ki = ki_step > stable_ki ? ki_step : stable_ki;

    while (true) // finds ki
    {
        if (ki_tune_counter >= max_ki_tune_counter)
        {
            printf("ki_tune_counter exceeds max.\n");
            break;
        }

        y_settings  = {1.0, -1.0, stable_kp, ki, stable_kd};
        drive_train.initialize_pid(x_settings, y_settings, th_settings);

        double avg_time = 0;
        double avg_error = 0;
        unsigned int avg_osc = 0;

        for (int i=0; i<num_of_test_loop; i++)
        {
            MotionFeedback feedback = {{0,0,0},{0,0,0},(0,0,0),0};
            drive_train.apply_motion(move_distance, 0, move_speed, 0.1, feedback);

            if (feedback.y_direction.total_time < 0 || avg_time < 0)
            {
                avg_time = -1.0;
            }
            else
            {
                avg_time  = _T_DT_AVG(avg_time , feedback.y_direction.total_time);
            }

            avg_error = _T_DT_AVG(avg_error, feedback.y_direction.error);
            avg_osc   = _T_DT_AVG(avg_osc  , feedback.y_direction.num_of_oscilations);
        }

        printf("Yfdbk %4.1f, %4.4f, %4.1f = {e:% 7.5f, t:% 7.3f, osc:%7d};\n", kp, ki, kd, avg_error, avg_time, avg_osc);

        if (std::fabs(avg_error) < lowest_error)
        {
            stable_ki = ki;
            lowest_error = std::fabs(avg_error);
        }

        ki_tune_counter++;
        ki += ki_step;
    }

    {
        printf("USING BEST PID\n");
        y_settings  = {1.0, -1.0, stable_kp, stable_ki, stable_kd};
        drive_train.initialize_pid(x_settings, y_settings, th_settings);

        double avg_time = 0;
        double avg_error = 0;
        unsigned int avg_osc = 0;

        for (int i=0; i<num_of_test_loop; i++)
        {
            MotionFeedback feedback = {{0,0,0},{0,0,0},(0,0,0),0};
            drive_train.apply_motion(move_distance, 0, move_speed, 0.1, feedback);

            if (feedback.y_direction.total_time < 0 || avg_time < 0)
            {
                avg_time = -1.0;
            }
            else
            {
                avg_time  = _T_DT_AVG(avg_time , feedback.y_direction.total_time);
            }

            avg_error = _T_DT_AVG(avg_error, feedback.y_direction.error);
            avg_osc   = _T_DT_AVG(avg_osc  , feedback.y_direction.num_of_oscilations);
        }

        printf("Yfdbk %4.1f, %4.4f, %4.1f = {e:% 7.5f, t:% 7.3f, osc:%7d};\n", stable_kp, stable_ki, stable_kd, avg_error, avg_time, avg_osc);
    }

    final_kp = stable_kp;
    final_ki = stable_ki;
    final_kd = stable_kd;
}

void test_drive_train(DriveTrainManager &drive_train, const double max_distance, const double max_speed, PID_settings x_settings, PID_settings y_settings)
{
    MotionFeedback feedback = {{0,0,0},{0,0,0},(0,0,0),0};
    PID_settings th_settings  = {1.0, -1.0, 0, 0, 0};
    drive_train.initialize_pid(x_settings, y_settings, th_settings);

    double avg_time = 0;
    double avg_error = 0;
    unsigned int avg_osc = 0;

    for (int i=1; i<10; i+=4)
    {
        for (int j=1; j<10; j+=4)
        {
            drive_train.apply_motion(max_distance * (i/10.0), 0, max_speed * (j/10.0), 0.1, feedback);

            if (feedback.y_direction.total_time < 0 || avg_time < 0)
            {
                avg_time = -1.0;
            }
            else
            {
                avg_time  = _T_DT_AVG(avg_time , feedback.y_direction.total_time);
            }

            avg_error = _T_DT_AVG(avg_error, feedback.y_direction.error);
            avg_osc   = _T_DT_AVG(avg_osc  , feedback.y_direction.num_of_oscilations);
        }

        std::cout << "i = " << i << std::endl;
    }

    printf("Yfdbk = {e:% 7.5f, t:% 7.3f, osc:%7d};\n", avg_error, avg_time, avg_osc);
    print_motion_feedback(feedback);
}

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
    PID_settings x_settings  = {1.0, -1.0, 0, 0, 0};
    PID_settings th_settings  = {1.0, -1.0, 0, 0, 0};
    PID_settings y_settings  = {1.0, -1.0, 0, 0, 0};

	const double move_speed = 0.4;
    const double move_distance = 0.5;

	cout << "Starting move test" << endl;

	if (status)
	{
        // double y_kp = 0;
        // double y_ki = 0;
        // double y_kd = 0;

        // double x_kp = 0;
        // double x_ki = 0;
        // double x_kd = 0;

        // auto_tune_drive_train_y(my_drive_train, move_distance, move_speed, y_kp, y_ki, y_kd);
        // y_settings  = {1.0, -1.0, y_kp, y_ki, y_kd};
        // my_drive_train.initialize_pid(x_settings, y_settings, th_settings);

        // auto_tune_drive_train_x(my_drive_train, move_distance, move_speed, x_kp, x_ki, x_kd);
        // x_settings  = {1.0, -1.0, x_kp, x_ki, x_kd};
        // my_drive_train.initialize_pid(x_settings, y_settings, th_settings);

        // printf("--- TESTING VALUES ---\n");

		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);

		// feedback = {{0,0,0},{0,0,0},(0,0,0),0};
		// my_drive_train.translate_by(-move_distance, move_speed, feedback); sleep(2);
		// print_motion_feedback(feedback);


        // printf("--- RECOMMENDED VALUES ---\n");
        // printf("Y: kp:% 7.5f, ki:% 7.5f, kd:% 7.5f\n", y_kp, y_ki, y_kd);
        // printf("X: kp:% 7.5f, ki:% 7.5f, kd:% 7.5f\n", x_kp, x_ki, x_kd);

        y_settings  = {1.0, -1.0, 4.0, 0.02, 1.0};
        x_settings  = {1.0, -1.0, 0.001, 0.04, 0.0};
        my_drive_train.initialize_pid(x_settings, y_settings, th_settings);
        test_drive_train(my_drive_train, 2.0, 0.8, x_settings, y_settings);

        y_settings  = {1.0, -1.0, 1.0, 0.004, 0.0};
        x_settings  = {1.0, -1.0, 0.001, 0.00, 0.0};
        my_drive_train.initialize_pid(x_settings, y_settings, th_settings);
        test_drive_train(my_drive_train, 2.0, 0.8, x_settings, y_settings);

        y_settings  = {1.0, -1.0, 16.0, 0.00, 2.0};
        x_settings  = {1.0, -1.0, 0.001, 0.00, 0.0};
        my_drive_train.initialize_pid(x_settings, y_settings, th_settings);
        test_drive_train(my_drive_train, 2.0, 0.8, x_settings, y_settings);
	}

	if (!my_drive_train.terminate())
	{
		cout << "Termination failed" << endl;
	}

	return 0;
}