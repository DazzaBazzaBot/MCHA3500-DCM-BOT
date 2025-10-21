#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "cmsis_os2.h"

#include "mod_manager.h"
#include "mod_data_logger.h"

#include "mod_mpu6050.h"
#include "mod_dcm_driver.h"
#include "mod_adc.h"
#include "mod_enc.h"

#include "mod_PID_control.h"
#include "mod_LQR_control.h"
#include "mod_MPC_control.h"

#define PERIOD_MS 5.0f
#define ENC_COUNT_PER_REV 1320.0f

// Thread ID and attributes
static osTimerId_t _modManagerTimerID;
static osTimerAttr_t _modManagerTimerAttr =
    {
        .name = "manager"};

// Flags
static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

/* Some little constant things */
static float dt = PERIOD_MS / 1000.0f;
static float PIx2 = 6.283185f;

/* Encoder */
static int32_t new_enc_count_right = 0;
static int32_t last_enc_count_right = 0;

static int32_t new_enc_count_left = 0;
static int32_t last_enc_count_left = 0;

static int32_t enc_average = 0;
static float enc_average_diff = 0;

/* Current sensing */
static float current_left = 0;
static float current_right = 0;

/* State vars */
// State vector [phi, theta, d_phi, d_theta]
static float state_vector[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static float phi = 0.0f;
static float rad_pitch = 0.0f;
static float d_phi = 0.0f;
static float rad_rate = 0.0f;

/* PID */
static float PID_output = 0.0f;

/* LQR */
static float LQR_output = 0.0f;

// Pointer function
static void (*manager_function)(void) = NULL;

void mod_manager_task(void *argument)
{
    // SHUT UP
    UNUSED(argument);

    (*manager_function)();
}

void mod_manager_PID(void)
{
    mod_mpu_update(&rad_pitch, &rad_rate);
    // printf("Pitch: %.5f, Rate: %.5f\n", rad_pitch, rad_rate);

    PID_output = mod_PID_run(rad_pitch);
    // printf("PID Output: %.5f\n", PID_output);

    mod_dcm_set_PWM(PID_output, -PID_output);
    // mod_stp_set_rpm(PID_output, PID_output);
}

void mod_manager_LQR(void)
{
    // Update encoder data [phi, d_phi]
    new_enc_count_right = mod_enc_get_countA();
    new_enc_count_left = mod_enc_get_countB();

    // Update phi
    enc_average = (new_enc_count_right + new_enc_count_left) / 2;
    phi = ((float)enc_average / ENC_COUNT_PER_REV) * PIx2;

    // Update d_phi
    enc_average_diff = ((new_enc_count_right - last_enc_count_right) + (new_enc_count_left - last_enc_count_left)) / 2;
    d_phi = PIx2 * ((enc_average_diff / dt) / ENC_COUNT_PER_REV);

    // Update mpu data [theta, d_theta]
    mod_mpu_update(&rad_pitch, &rad_rate);

    // Update state vector
    state_vector[0] = phi;
    state_vector[1] = rad_pitch;
    state_vector[2] = d_phi;
    state_vector[3] = rad_rate;

    // Update encoder count
    last_enc_count_right = new_enc_count_right;
    last_enc_count_left = new_enc_count_left;

    // Update state vector for control
    mod_LQR_set_states(state_vector);

    // Update Control
    mod_LQR_update();

    // Get control
    LQR_output = mod_LQR_get_control();


    // Update current sensor ?

    // Update torque readings?

    // Print state vector
    //printf("State: [%.4f, %.4f, %.4f, %.4f]\n", state_vector[0], state_vector[1], state_vector[2], state_vector[3]);
}

void mod_manager_MPC(void)
{
    printf("MPC\n");
}

void mod_manager_init(void)
{
    // Configure sub modules
    mod_mpu_configure_hardware();
    mod_dcm_configure_hardware();
    mod_adc_configure_hardware();
    mod_enc_configure_hardware();

    mod_log_init();

    // probs none needed
    // mod_PID_control_init();
    // mod_LQR_control_init();

    // Init big module
    if (!_is_init)
    {
        _modManagerTimerID = osTimerNew(mod_manager_task, osTimerPeriodic, NULL, &_modManagerTimerAttr);
        _is_init = 1;
    }
}

// Start the module manager task
void mod_manager_start(int8_t method)
{
    // Set the function pointer based on the selected method
    if (method == MOD_MANAGER_PID)
    {
        manager_function = &mod_manager_PID;
    }
    else if (method == MOD_MANAGER_LQR)
    {
        manager_function = &mod_manager_LQR;
    }
    else if (method == MOD_MANAGER_MPC)
    {
        manager_function = &mod_manager_MPC;
    }
    else
    {
        manager_function = NULL;
    }

    // Get it started
    if (!_is_running)
    {
        osTimerStart(_modManagerTimerID, PERIOD_MS);
        _is_running = 1;
    }
}

// Stop the module manager task
void mod_manager_stop(void)
{
    if (_is_running)
    {
        osTimerStop(_modManagerTimerID);
        _is_running = 0;
    }
}
