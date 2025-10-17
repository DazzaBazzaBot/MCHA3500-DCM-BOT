#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "mod_LQR_control.h"

#define CTRL_N_INPUT 1
#define CTRL_N_STATE 5

// Control gain K
static float ctrl_mK_f32[CTRL_N_INPUT * CTRL_N_STATE] =
    {
        57.4827f,
        162.0246f,
        49.4904f,
        26.5967f,
        29.8511f,
};

// State vector including integrator
static float ctrl_x_f32[CTRL_N_STATE] =
    {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
};

// Control input
static float ctrl_u_f32[CTRL_N_INPUT] =
    {
        0.0,
};

// This this Az
static float ctrl_Az_f32[CTRL_N_STATE] =
    {
        0.0050f,
        0.0f,
        0.0f,
        0.0f,
        1.0f,
};

// Integrator state
static float ctrl_z_f32[CTRL_N_INPUT] =
    {
        0.0,
};

/* Define control matrix variables */
arm_matrix_instance_f32 ctrl_mK = {CTRL_N_INPUT, CTRL_N_STATE, (float32_t *)ctrl_mK_f32};
arm_matrix_instance_f32 ctrl_x = {CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32};
arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};
arm_matrix_instance_f32 ctrl_Az = {1, CTRL_N_STATE, (float32_t *)ctrl_Az_f32};
arm_matrix_instance_f32 ctrl_z = {1, 1, (float32_t *)ctrl_z_f32};

// Control init
void mod_LQR_init(void)
{
    arm_mat_init_f32(&ctrl_mK, CTRL_N_INPUT, CTRL_N_STATE, (float32_t *)ctrl_mK_f32);
    arm_mat_init_f32(&ctrl_x, CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32);
    arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32);
    arm_mat_init_f32(&ctrl_Az, 1, CTRL_N_STATE, (float32_t *)ctrl_Az_f32);
    arm_mat_init_f32(&ctrl_z, 1, 1, (float32_t *)ctrl_z_f32);
}

// Sets all states at once
void mod_LQR_set_states(float new_states[])
{
    ctrl_x_f32[0] = new_states[0];
    ctrl_x_f32[1] = new_states[1];
    ctrl_x_f32[2] = new_states[2];
    ctrl_x_f32[3] = new_states[3];
}

void mod_LQR_set_x1(float x1)
{
    ctrl_x_f32[0] = x1;
}

void mod_LQR_set_x2(float x2)
{
    ctrl_x_f32[1] = x2;
}

void mod_LQR_set_x3(float x3)
{
    ctrl_x_f32[2] = x3;
}

void mod_LQR_set_x4(float x4)
{
    ctrl_x_f32[3] = x4;
}

// Updates a new control gain
void mod_LQR_update(void)
{
    // K*state
    arm_mat_mult_f32(&ctrl_mK, &ctrl_x, &ctrl_u);

    // Update integrator state
    arm_mat_mult_f32(&ctrl_Az, &ctrl_x, &ctrl_z);

    // Copy updated value of integrator state into state vector
    ctrl_x_f32[4] = ctrl_z_f32[0];
}

float mod_LQR_get_control(void)
{
    return ctrl_u_f32[0];
}