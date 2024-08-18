#ifndef PID_H
#define PID_H

#include <stdint.h> // For using data types like uint32_t

typedef struct {
    double kp, ki, kd;      // PID coefficients
    double setpoint;        // Desired target value
    double integral;        // Integral sum
    double last_error;      // Last error value
    double last_time;       // Last time in seconds
} PID;

// Functions to manage PID
void PID_Init(PID *pid, double kp, double ki, double kd);
double PID_Update(PID *pid, double measurement, double current_time);
void PID_SetSetpoint(PID *pid, double setpoint);

#endif // PID_H
