#include "pid.h"

void PID_Init(PID *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0;
    pid->integral = 0.0;
    pid->last_error = 0.0;
    pid->last_time = -1.0; // Initialize to an invalid time
}

double PID_Update(PID *pid, double measurement, double current_time) {
    double error = pid->setpoint - measurement;
    double delta_time = (pid->last_time < 0) ? 0 : (current_time - pid->last_time);
    double derivative = 0.0;

    if (delta_time > 0) {
        pid->integral += error * delta_time;
        derivative = (error - pid->last_error) / delta_time;
    }

    double output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // Update last error and last time for next calculation
    pid->last_error = error;
    pid->last_time = current_time;

    return output;
}

void PID_SetSetpoint(PID *pid, double setpoint) {
    pid->setpoint = setpoint;
    pid->integral = 0.0;    // Reset integral
    pid->last_error = 0.0;  // Reset last error
}
