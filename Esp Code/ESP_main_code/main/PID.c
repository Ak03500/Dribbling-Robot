#include "PID.h"
#include <sys/time.h>

void PID_init(PIDController *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0;
    pid->integral = 0;
    gettimeofday(&(pid->prev_time), NULL);
}

float PID_update(PIDController *pid, float error) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    float dt = (current_time.tv_sec - pid->prev_time.tv_sec) + (current_time.tv_usec - pid->prev_time.tv_usec) / 1000000.0;
    pid->prev_time = current_time;

    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;
    return output;
}
