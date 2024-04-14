class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0
        self.integral = 0
        self.last_error = 0
        self.last_time = None

    def update(self, measurement, current_time):
        error = self.setpoint - measurement
        delta_time = (current_time - self.last_time) if self.last_time is not None else 0
        self.integral += error * delta_time
        derivative = (error - self.last_error) / delta_time if delta_time > 0 else 0

        # PID output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Update last error and last time for next calculation
        self.last_error = error
        self.last_time = current_time

        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0
        self.last_error = 0
