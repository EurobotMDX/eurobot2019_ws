#include "pid.h"
#include <thread>
#include <stdio.h>

int main()
{
    PID pid = PID(100, -100, 0.1, 0.5, 0.01);

    double val = 20;
    for (int i=0; i<100; i++)
    {
        double inc = pid.calculate(0, val);
        printf("val:% 7.3f inc:% 7.3f dt:% 7.3f\n", val, inc, pid.get_dt());
        val += inc;

        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }

    return 0;
}