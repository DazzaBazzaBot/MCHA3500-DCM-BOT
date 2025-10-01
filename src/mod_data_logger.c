#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "math.h"

// my awesome mods
#include "mod_data_logger.h"
#include "mod_manager.h"
#include "mod_dcm_driver.h"
#include "mod_mpu6050.h"
#include "mod_adc.h"

#define PERIOD_MS 5.0f
#define LOG_DURATION_MS 5000.0f

// Timer ID and attributes
static osTimerId_t _dataLoggingTimerID;
static osTimerAttr_t _dataLoggingTimerAttr = {
    .name = "dataLogger"};

// flags
static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

// for logger
uint16_t logCount = 0;


// ma logger
static void (*log_function)(void) = NULL;

void log_imu(void);
void log_imu(void)
{

}

void log_inertia(void);
void log_inertia(){
    //ill do it
    //eventually
    // please come back co pilot
    // i cant speell without u
}


void log_motor(void);
void log_motor(){
    float time = logCount * (PERIOD_MS / 1000.0f);
    logCount++;

    // generate 1hz sine wave between -6 and 6 volts
    float voltage = 6.0f * sinf(6.283185 * time);
    mod_dcm_set_voltage_log(voltage);

    // read adc left and right
    float voltage_torque = 0;
    float voltage_current = 0;
    mod_adc_update_readings(&voltage_current, &voltage_torque);

    float current = (voltage_current - 1.65f) / .4f;
    float torque = 0.3577f * (voltage_torque - 1.6337f); // 0.1 Nm per volt


    // print [time],[voltage],[torque],[current],[velocity]
    //printf("%.5f,%.5f,%.5f,%.5f\n", time, voltage, torque, current);

    // stop after 5s
    if(time >= 5.0f){
        mod_log_stop();
        mod_dcm_set_voltage_log(0);
    }   
}


void mod_log_task(void *argument)
{
    // SUPPRESS
    UNUSED(argument);

    (*log_function)();
}


void mod_log_init(void)
{
    if (!_is_init)
    {
        _dataLoggingTimerID = osTimerNew(mod_log_task, osTimerPeriodic, NULL, &_dataLoggingTimerAttr);
        _is_init = 1;
    }
}

void mod_log_start(uint8_t log_type)
{
    if (log_type == LOG_IMU)
    {
        log_function = &log_imu;
    }
    else if (log_type == LOG_INERTIA)
    {
        log_function = &log_inertia;
    }
    else if (log_type == LOG_MOTOR)
    {
        log_function = &log_motor;
    }
    else
    {
        log_function = NULL;
        return;
    }

    if (!_is_running)
    {
        logCount = 0;
        osTimerStart(_dataLoggingTimerID, PERIOD_MS);
        _is_running = 1;
    }
}

void mod_log_stop(void)
{
    if (_is_running)
    {
        osTimerStop(_dataLoggingTimerID);
        _is_running = 0;
    }
}