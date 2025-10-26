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

#define PERIOD_MS 10.0f
#define ENC_COUNT_PER_REV 1320.0f
#define VMAX 12
#define VMIN -12
#define VEL_COUNT 3
#define PIx2 6.283185f
#define PHI_MAX 2.0f

// Left motor params
#define N_LEFT 33.0f
#define Ki_LEFT 0.004634328631135f
#define Kw_LEFT 0.258184074389226f
#define Ra_LEFT 5.201728777304033

// Right motor params
#define N_RIGHT 33.0f
#define Ki_RIGHT 0.005211169122476f
#define Kw_RIGHT 0.311524176289173f
#define Ra_RIGHT 5.15671139960308f

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

/* Encoder */
static int32_t new_enc_count_right = 0;
static int32_t last_enc_count_right = 0;

static int32_t new_enc_count_left = 0;
static int32_t last_enc_count_left = 0;

static int32_t enc_average = 0;

/* Current sensing */
static float adc_voltage_left = 0;
static float adc_voltage_right = 0;
static float current_left = 0;
static float current_right = 0;

/* Desired Voltage */
static float voltage_left = 0;
static float voltage_right = 0;

// shut up fuck you
static uint8_t vel_index = 0;
static float omega_left = 0.0f;
static float omega_right = 0.0f;
static float left_vel_buff[VEL_COUNT] = {0};
static float right_vel_buff[VEL_COUNT] = {0};
static float left_vel_sum = 0.0f;
static float right_vel_sum = 0.0f;

/* State vars */
// State vector [phi, theta, d_phi, d_theta]
static float state_vector[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static float phi = 0.0f;
static float theta = 0.0f;
static float d_phi = 0.0f;
static float d_theta = 0.0f;

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

void mod_manager_PID(void);
void mod_manager_PID()
{
    theta = mod_mpu_get_pitch();
    // printf("Pitch: %.5f, Rate: %.5f\n", theta, d_theta);

    PID_output = mod_PID_run(theta);

    mod_dcm_set_PWM(PID_output, -0.9 * PID_output);
    printf("theta: %.5f\n", theta);
}

void mod_manager_LQR(void);
void mod_manager_LQR()
{
    // Update encoder data [phi, d_phi]
    new_enc_count_right = -mod_enc_get_countA();
    new_enc_count_left = -mod_enc_get_countB();

    // Update phi, only using left
    phi = ((float)new_enc_count_left / ENC_COUNT_PER_REV) * PIx2;

    // Average of last 3 dphi
    /*left_vel_sum -= left_vel_buff[vel_index];
    right_vel_sum -= right_vel_buff[vel_index];

    left_vel_buff[vel_index] = PIx2 * (new_enc_count_left - last_enc_count_left) / (dt * ENC_COUNT_PER_REV);
    right_vel_buff[vel_index] = PIx2 * (new_enc_count_right - last_enc_count_right) / (dt * ENC_COUNT_PER_REV);

    left_vel_sum += left_vel_buff[vel_index];
    right_vel_sum += right_vel_buff[vel_index];

    omega_left = left_vel_sum / VEL_COUNT;
    omega_right = right_vel_sum / VEL_COUNT;*/

    // Raw vel
    omega_left = PIx2 * (new_enc_count_left - last_enc_count_left) / (dt * ENC_COUNT_PER_REV);
    omega_right = PIx2 * (new_enc_count_right - last_enc_count_right) / (dt * ENC_COUNT_PER_REV);

    // Will only track left dphi for state
    d_phi = omega_left;

    // Update mpu data [theta, d_theta]
    d_theta = mod_mpu_get_rps();
    theta = mod_mpu_get_pitch();

    // Update state vector
    state_vector[0] = phi;
    state_vector[1] = theta;
    state_vector[2] = d_phi;
    state_vector[3] = d_theta;

    // Update encoder count
    last_enc_count_right = new_enc_count_right;
    last_enc_count_left = new_enc_count_left;

    // Update state vector for control
    mod_LQR_set_states(state_vector);

    // Update Control
    mod_LQR_update();

    // Get control, returns torque
    LQR_output = mod_LQR_get_control();
    LQR_output *= 0.5f;

    // Update current sensor ?
    adc_voltage_left = mod_adc_get_voltageB();
    adc_voltage_right = mod_adc_get_voltageA();
    current_left = (adc_voltage_left - 2.5f) / 0.4f;
    current_right = (adc_voltage_right - 2.5f) / 0.4f;

    // Desired voltage
    // V = R*t/(N*Ki) + Kw*N*w
    voltage_left = (Ra_LEFT * LQR_output / (N_LEFT * Ki_LEFT)) + Kw_LEFT * N_LEFT * omega_left;
    voltage_left = fmin(fmax(VMIN, voltage_left), VMAX);

    voltage_right = (Ra_RIGHT * LQR_output / (N_RIGHT * Ki_RIGHT)) + Kw_RIGHT * N_RIGHT * omega_right;
    voltage_right = fmin(fmax(VMIN, voltage_right), VMAX);
    // printf("Left: %.4f,         Right: %.4f\n", voltage_left, voltage_right);

    // voltage_left = 11*LQR_output;
    // voltage_right = 10*LQR_output;

    // set voltages
    mod_dcm_set_voltageLeft(voltage_left);
    mod_dcm_set_voltageRight(voltage_right);

    // Print state vector
    printf("State: [%.4f, %.4f, %.4f, %.4f]\n", state_vector[0], state_vector[1], state_vector[2], state_vector[3]);

    /*
    vel_index++;
    if (vel_index >= VEL_COUNT)
    {
        vel_index = 0;
    }*/
    phi = fmin(fmax(-PHI_MAX, phi), PHI_MAX);
}

void mod_manager_MPC(void);
void mod_manager_MPC()
{
    printf("MPC\n");
}

void mod_manager_ASS(void);
void mod_manager_ASS()
{
    // New position
    new_enc_count_right = -mod_enc_get_countA();
    new_enc_count_left = -mod_enc_get_countB();
    phi = ((float)new_enc_count_left / ENC_COUNT_PER_REV) * PIx2;

    // Update wheel velocity
    omega_left = PIx2 * (float)(new_enc_count_left - last_enc_count_left) / (dt * ENC_COUNT_PER_REV);
    omega_right = PIx2 * (float)(new_enc_count_right - last_enc_count_right) / (dt * ENC_COUNT_PER_REV);
    d_phi = omega_left;

    // Update var from kalman filter
    theta = mod_mpu_get_pitch();
    d_theta = mod_mpu_get_rps();

    // Update old position
    last_enc_count_right = new_enc_count_right;
    last_enc_count_left = new_enc_count_left;
}

// get them states
float mod_manager_get_phi()
{
    return phi;
}
float mod_manager_get_theta()
{
    return theta;
}
float mod_manager_get_dphi()
{
    return d_phi;
}
float mod_manager_get_dtheta()
{
    return d_theta;
}

void mod_manager_init(void)
{
    // Configure sub modules
    mod_dcm_configure_hardware();
    mod_adc_configure_hardware();
    mod_enc_configure_hardware();

    mod_log_init();
    mod_mpu_init();

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
    else if (method == MOD_MANAGER_ASS)
    {
        manager_function = &mod_manager_ASS;
    }
    else
    {
        manager_function = NULL;
    }

    // Get it started
    if (!_is_running)
    {
        mod_mpu_start();
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
