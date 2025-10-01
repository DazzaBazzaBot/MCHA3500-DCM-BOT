#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "math.h"

#include "mod_PID_control.h"

#define PERIOD_MS 5
#define LIMIT 100.0f
#define DEADZONE 0.6f

// Control variables
static PID_t pid = {
    .Kp = 29.0f,
    .Ki = 0.0f,
    .Kd = 1.2f,

    .prev_error = 0.0f,
    .integral = 0.0f,
};

// Angle variables
static float target_angle = 0.0f;
static float error = 0.0f;
static float rpm = 0.0f;

float mod_PID_run(float new_pitch)
{
    new_pitch = 180.0f * new_pitch / 3.14159f;
    error = target_angle - new_pitch;

    //printf("Pitch: %.5f, Error: %.5f\n", new_pitch, error);

    if (fabsf(error) < DEADZONE)
    {
        rpm = 0;
    }
    else
    {
        rpm = -mod_PID_compute_control(error, PERIOD_MS);
        rpm = fmin(fmax(rpm, -LIMIT), LIMIT);
    }
    return rpm;
}

float mod_PID_compute_control(float func_error, float func_dt)
{
    pid.integral += func_error * func_dt;
    float derivative = (func_error - pid.prev_error) / func_dt;

    float output = pid.Kp * func_error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.prev_error = func_error;
    
    return output;
}