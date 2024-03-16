#ifndef PID_H
#define PID_H

#include <sys/time.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    struct timeval prev_time;
} PIDController;

void PID_init(PIDController *pid, float Kp, float Ki, float Kd);
float PID_update(PIDController *pid, float error);

#endif /* PID_H */
