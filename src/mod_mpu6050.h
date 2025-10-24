#ifndef MOD_MPU6050_H
#define MOD_MPU6050_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define MPU6050_ADDR 0x68 << 1
#define MPU6050_SMPLRT_DIV_REG 0x19
#define MPU6050_CONFIG_REG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

// Kalman struct
typedef struct
{
    float y[2];

    float R[2][2];
    float Q[3][3];
    float Ad[3][3];

    float Kk[3][2];
    float Pp[3][3];
    float Pm[3][3];

    float xp[3];
    float xm[3];
} KALMAN;

/* Update vars */
void mod_mpu_update(float *angle, float *rps);
float mod_mpu_update_pitch(int16_t, int16_t, int16_t);
float mod_mpu_update_rps(int16_t);
void mod_mpu_update_kalman(float, float);

/* Get updated pitch */
void mod_mpu_get_states(float *angle, float *rps);
float mod_mpu_get_pitch(void);
float mod_mpu_get_rps(void);

/* Read the raw data from the MPU6050 */
HAL_StatusTypeDef mod_mpu_read_raw_accel(int16_t *ax, int16_t *ay, int16_t *az);
HAL_StatusTypeDef mod_mpu_read_raw_gyro(int16_t *gx, int16_t *gy, int16_t *gz);

/* Init mpu pins and I2C */
void mod_mpu_configure_hardware(void);
void mod_mpu_init(void);
void mod_mpu_start(void);
void mod_mpu_stop(void);
void mod_mpu_task(void);

#endif