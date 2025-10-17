#ifndef MOD_MOTOR_DRIVER_H
#define MOD_MOTOR_DRIVER_H

#include "stdlib.h"

void mod_dcm_configure_hardware(void);

// settin stuff bitch face
void mod_dcm_set_PWM(float, float);
void mod_dcm_set_voltage(float, float);
void mod_dcm_set_voltageLeft(float);
void mod_dcm_set_voltageRight(float);

// directionality
void left_forward(void);
void left_reverse(void);
void left_halt(void);
void right_forward(void);
void right_reverse(void);
void right_halt(void);

// for data logging ig
void mod_dcm_set_voltage_log(float);

#endif