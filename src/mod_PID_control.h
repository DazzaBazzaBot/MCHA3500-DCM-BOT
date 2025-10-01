#ifndef MOD_PID_CONTROL_H
#define MOD_PID_CONTROL_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float prev_error;
    float integral;
} PID_t;

float mod_PID_run(float);
float mod_PID_compute_control(float, float);

#endif