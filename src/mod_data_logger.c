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
#include "mod_enc.h"

#define LOG_PERIOD_MS 5.0f
#define IMU_LOG_PERIOD_MS 3.0f
#define LOG_DURATION_MS 5000.0f
#define COUNTS_PER_OUTPUT_REV 1320
#define INERTIA_COUNT_PER_REV 4096

static float log_period = LOG_PERIOD_MS;

// Timer ID and attributes
static osTimerId_t _dataLoggingTimerID;
static osTimerAttr_t _dataLoggingTimerAttr = {
    .name = "dataLogger"};

// flags
static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

// for logger
static uint16_t logCount = 0;
static int32_t lastEncoderCount = 0;
static int32_t encoder_CountA = 0;
static int32_t encoder_CountB = 0;
static int32_t encoder_CountAvg = 0;

// ma logger
static void (*log_function)(void) = NULL;

// for mpu logger
static int16_t ax = 0;
static int16_t ay = 0;
static int16_t az = 0;
static int16_t gx = 0;
static int16_t gy = 0;
static int16_t gz = 0;

void log_imu(void);
void log_imu(void)
{
    // update time
    float time = logCount * (IMU_LOG_PERIOD_MS / 1000.0f);
    logCount++;

    // get enc angle
    encoder_CountA = mod_enc_get_countA();
    encoder_CountB = mod_enc_get_countA();
    encoder_CountAvg = (encoder_CountA + encoder_CountB)/2;
    float enc_angle = ((float)encoder_CountAvg / COUNTS_PER_OUTPUT_REV) * 6.283185f;

    //// get acc angle
    //mod_mpu_read_raw_accel(&ax, &ay, &az);
    //float acc_angle = mod_mpu_update_pitch(ax, ay, az);

    //// get gyro rate
    //mod_mpu_read_raw_gyro(&gx, &gy, &gz);
    //float rps_gyro = -mod_mpu_update_rps(gx);
    float acc_angle = 0;
    float rps_gyro = 0;
    mod_mpu_update(&acc_angle, &rps_gyro);

    // print [time],[angle],[acc],[rate]
    printf("%.3f,%.4f,%.4f,%.4f\n", time, enc_angle, acc_angle, rps_gyro);

    // print enc count
    //printf("%ld\n", encoderCount);

    // compare time
    if (time >= 15.0f)
    {
        mod_log_stop();
    }
}

void log_inertia(void);
void log_inertia()
{
    // Time
    float time = logCount * (LOG_PERIOD_MS / 1000.0f);
    logCount++;

    // Rotational velocity
    int32_t newEncoderCount = mod_enc_get_countA();

    float angle_rad = ((float)newEncoderCount / INERTIA_COUNT_PER_REV) * 6.283185f;

    float count_diff = (float)(newEncoderCount - lastEncoderCount);
    float time_period = LOG_PERIOD_MS / 1000.0f; // Convert ms to seconds
    float counts_per_sec = count_diff / time_period;
    float rps = counts_per_sec / INERTIA_COUNT_PER_REV;
    float rad_per_sec = rps * 6.283185f;

    // print [time],[ANGLE],[velocity]
    printf("%.3f,%.5f,%.5f\n", time, angle_rad, rad_per_sec);

    // stop after 5s
    if (time >= 30.0f)
    {
        mod_log_stop();
    }

    // Update last count
    lastEncoderCount = newEncoderCount;
}

void log_motor(void);
void log_motor()
{
    // Time
    float time = logCount * (LOG_PERIOD_MS / 1000.0f);
    logCount++;

    // Voltage input
    float voltage = 9.0f * sinf(6.283185 * time * 0.5f);
    mod_dcm_set_voltage_log(voltage);

    // Current and torque readings
    float voltage_torque = 0;
    float voltage_current = 0;
    mod_adc_update_readings(&voltage_torque, &voltage_current);

    float current = (voltage_current - 1.65f) / 0.4f;
    // float current = (voltage_current - 2.5f) / 0.4f;
    float torque = 0.3577f * (voltage_torque - 1.6337f);

    // Rotational velocity
    int32_t newEncoderCount = mod_enc_get_countB();
    float count_diff = (float)(newEncoderCount - lastEncoderCount);
    float time_period = LOG_PERIOD_MS / 1000.0f; // Convert ms to seconds
    float counts_per_sec = count_diff / time_period;
    float rps = counts_per_sec / COUNTS_PER_OUTPUT_REV;
    float rad_per_sec = rps * 6.283185f;

    // print [time],[voltage],[torque],[current],[velocity]
    printf("%.5f,%.5f,%.5f,%.5f,%.5f\n", time, voltage, torque, current, rad_per_sec);

    // stop after 5s
    if (time >= 10.0f)
    {
        mod_log_stop();
        mod_dcm_set_voltage_log(0);
    }

    // Update last count
    lastEncoderCount = newEncoderCount;
}

void log_motor_freewheel(void);
void log_motor_freewheel()
{
    // Time
    float time = logCount * (LOG_PERIOD_MS / 1000.0f);
    logCount++;

    // Voltage input
    float voltage = 9.0f * sinf(6.283185 * time * 1.0f);
    mod_dcm_set_voltage_log(voltage);

    // Current and torque readings
    float voltage_torque = 0;
    float voltage_current = 0;
    mod_adc_update_readings(&voltage_torque, &voltage_current);

    float current = (voltage_current - 1.65f) / 0.4f;
    // float current = (voltage_current - 2.5f) / 0.4f;

    // Rotational velocity
    int32_t newEncoderCount = mod_enc_get_countB();
    float count_diff = (float)(newEncoderCount - lastEncoderCount);
    float time_period = LOG_PERIOD_MS / 1000.0f; // Convert ms to seconds
    float counts_per_sec = count_diff / time_period;
    float rps = counts_per_sec / COUNTS_PER_OUTPUT_REV;
    float rad_per_sec = rps * 6.283185f;

    // print [time],[voltage],[current],[velocity]
    printf("%.5f,%.5f,%.5f,%.5f\n", time, voltage, current, rad_per_sec);

    // stop after 5s
    if (time >= 10.0f)
    {
        mod_log_stop();
        mod_dcm_set_voltage_log(0);
    }

    // Update last count
    lastEncoderCount = newEncoderCount;
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
        log_period = IMU_LOG_PERIOD_MS;
    }
    else if (log_type == LOG_INERTIA)
    {
        log_function = &log_inertia;
        log_period = LOG_PERIOD_MS;
    }
    else if (log_type == LOG_MOTOR)
    {
        log_function = &log_motor;
        log_period = LOG_PERIOD_MS;
    }
    else if (log_type == LOG_FREEWHEEL)
    {
        log_function = &log_motor_freewheel;
        log_period = LOG_PERIOD_MS;
    }
    else
    {
        log_function = NULL;
        return;
    }

    if (!_is_running)
    {
        logCount = 0;
        encoder_CountA = 0;
        encoder_CountB = 0;
        lastEncoderCount = 0;

        osTimerStart(_dataLoggingTimerID, log_period);
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