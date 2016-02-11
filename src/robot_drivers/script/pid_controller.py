
import time

class PIDImpl(object):
    def __init__(self, min, max, Kp, Ki, Kd):
        self.min = min
        self.max = max
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.dt = 0.0
        self.pre_error = 0.0
        self.integral = 0.0

        self.last_loop_update_time = time.time()
    
    def calculate(self, setpoint, pv):
        self.update_timestamp()

        error = setpoint - pv
        p_out = self.Kp * error
        
        self.integral += error * self.dt
        i_out = self.Ki * self.integral

        derivative = (error - self.pre_error) / self.dt
        d_out = self.Kd * derivative

        output = p_out + i_out + d_out

        # TODO: Implement Constrain Function
        if (output > self.max):
            output = self.max
        elif (output < self.min):
            output = self.min

        self.pre_error = error

        return output
    
    def get_dt(self):
        return self.dt
    
    def update_timestamp(self):
        current_time = time.time()
        self.dt = current_time - self.last_loop_update_time
        self.last_loop_update_time = current_time
    
    def constrainf(self, val, min_val, max_val):
        return min(max_val, max(val, min_val))

class PID(object):
    def __init__(self, min, max, Kp, Ki, Kd):
        self.pid_impl = PIDImpl(min=min, max=max, Kp=Kp, Ki=Ki, Kd=Kd)

    def calculate(self, setpoint, pv):
        return self.pid_impl.calculate(setpoint, pv)

    def get_dt(self):
        return self.pid_impl.get_dt()