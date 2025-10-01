#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "cmsis_os2.h"

#include "mod_manager.h"
#include "mod_data_logger.h"

#include "mod_mpu6050.h"
#include "mod_dcm_driver.h"
#include "mod_adc.h"

#include "mod_PID_control.h"

#define PERIOD_MS 10

// Thread ID and attributes
static osTimerId_t _modManagerTimerID;
static osTimerAttr_t _modManagerTimerAttr =
    {
        .name = "manager"};

// Flags
static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

// Vars for stuff and stuff
static float rad_pitch = 0.0f;
static float rad_rate = 0.0f;
static float PID_output = 0.0f;



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
    //printf("Pitch: %.5f, Rate: %.5f\n", rad_pitch, rad_rate);

    PID_output = mod_PID_run(rad_pitch);
    //printf("PID Output: %.5f\n", PID_output);

    
    mod_dcm_set_PWM(PID_output, PID_output);
    //mod_stp_set_rpm(PID_output, PID_output);
}

void mod_manager_LQR(void)
{
    printf("LQR\n");
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

    mod_log_init();


    // probs none needed
    //mod_PID_control_init();



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
