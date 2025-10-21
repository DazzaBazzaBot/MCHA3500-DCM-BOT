#include <stddef.h>
#include "stm32f4xx_hal.h"

#include "mod_LQR_control.h"

#define CTRL_N_INPUT 1
#define CTRL_N_STATE 4
#define CTRL_N_STATE_INT 5

static float LQR_y_ref[CTRL_N_INPUT] = {0.0f};

// Vector from init
static float LQR_N_x_f32[CTRL_N_STATE] =
    {
        1.0f,
        0.0f,
        0.0f,
        0.0f,
};

// Updates each loop
static float LQR_x_ss_f32[CTRL_N_STATE] =
    {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
};

// Value from init
static float LQR_N_u_f32[CTRL_N_INPUT] = {0.0f};

// Updates each loop
// Will be 0  cause N_u is zero
static float LQR_u_ss_f32[CTRL_N_INPUT] = {0.0f};

// negative K to save on steps,
// Value from init
static float LQR_K_f32[CTRL_N_STATE_INT] =
    {
        0.4958f,
        38.8163f,
        0.5884f,
        7.5350f,
        0.1815f,
};

// Updates each loop
static float LQR_state_f32[CTRL_N_STATE_INT] =
    {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
};

// Value from init
static float LQR_Az_f32[CTRL_N_STATE_INT] =
    {
        0.005f,
        0.0f,
        0.0f,
        0.0f,
        1.0f,
};

// UUpdates each loop
static float LQR_z_f32[CTRL_N_INPUT] = {0.0f};

// UUpdates each loop
static float LQR_u_f32[CTRL_N_INPUT] = {0.0f};

// Sets all states at once
void mod_LQR_set_states(float new_states[])
{
    LQR_state_f32[0] = new_states[0];
    LQR_state_f32[1] = new_states[1];
    LQR_state_f32[2] = new_states[2];
    LQR_state_f32[3] = new_states[3];
}

void mod_LQR_set_x1(float x1)
{
    LQR_state_f32[0] = x1;
}

void mod_LQR_set_x2(float x2)
{
    LQR_state_f32[1] = x2;
}

void mod_LQR_set_x3(float x3)
{
    LQR_state_f32[2] = x3;
}

void mod_LQR_set_x4(float x4)
{
    LQR_state_f32[3] = x4;
}

void mod_LQR_set_y_ref(float y_ref)
{
    LQR_y_ref[0] = y_ref;
}

// Updates a new control gain
void mod_LQR_update(void)
{
    // compute feed forward, x_ss = N_x * y_ref
    for (int i = 0; i < CTRL_N_STATE; i++)
    {
        LQR_x_ss_f32[i] = LQR_N_x_f32[i] * LQR_y_ref[0];
    }

    // state - [x_ss; 0]
    for (int i = 0; i < CTRL_N_STATE; i++)
    {
        LQR_state_f32[i] = LQR_state_f32[i] - LQR_x_ss_f32[i];
    }
    // print state vector
    // printf("State: [%.4f, %.4f, %.4f, %.4f, %.4f]\n", LQR_state_f32[0], LQR_state_f32[1], LQR_state_f32[2], LQR_state_f32[3], LQR_state_f32[4]);

    // compute control u = K * state
    LQR_u_f32[0] = 0.0f;
    for (int i = 0; i < CTRL_N_STATE_INT; i++)
    {
        LQR_u_f32[0] += LQR_K_f32[i] * LQR_state_f32[i];
    }

    // update integrator z = A_z * state
    LQR_z_f32[0] = LQR_Az_f32[0] * LQR_state_f32[0] + LQR_Az_f32[4] * LQR_state_f32[4];
    // printf("Integrator: %.4f\n", LQR_z_f32[0]);

    // update state vector with new integrator
    LQR_state_f32[4] = LQR_z_f32[0];
}

float mod_LQR_get_control(void)
{
    return LQR_u_f32[0];
}