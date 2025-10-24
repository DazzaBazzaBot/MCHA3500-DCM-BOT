#include <math.h>
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "cmsis_os2.h"

#include "mod_mpu6050.h"

#define MPU_PERIOD_MS 3

// Thread ID and attributes
static osTimerId_t _modMPUTimerID;
static osTimerAttr_t _modMPUTimerAttr =
{
        .name = "mpu6050"};

// Flags
static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

// I2C handle
static I2C_HandleTypeDef hi2c1;

// MPU raw vals
static int16_t ax = 0;
static int16_t ay = 0;
static int16_t az = 0;
static int16_t gx = 0;


// Kalman filter struct
static KALMAN kf = {
    // Initial values
    .y = {0.0f, 0.0f},

    // Covariance matrix
    .R = {{0.236169466336982f,     0.0f}, 
          {0.0f,        0.00000722446f}},
    // Process noise covariance
    .Q = {{0.0002890f,  0.0000103f,  -0.0001530f,}, 
          {0.0000103f,  0.0000036f,   0.0000049f,}, 
          {-0.0001530f, 0.0000049f,   0.0001219f,}},

    // State transition matrix
    .Ad = {{1.0000f, 0.0000f, 0.0000f,}, 
           {0.0030f, 1.0000f, 0.0000f,}, 
           {0.0000f, 0.0000f, 1.0000f}},

    // Kalman gain
    .Kk = {{0.0f, 0.0f}, 
           {0.0f, 0.0f}, 
           {0.0f, 0.0f}},
    // Predicted Pp
    .Pp = {{0.0f, 0.0f, 0.0f}, 
           {0.0f, 0.0f, 0.0f}, 
           {0.0f, 0.0f, 0.0f}},
    // Measured Pm
    .Pm = {{0.0f, 0.0f, 0.0f}, 
           {0.0f, 0.0f, 0.0f}, 
           {0.0f, 0.0f, 0.0f}},

    // Predicted state
    .xp = {0.0f, 0.0f, 0.0f},
    // Measured state
    .xm = {0.0f, 0.0f, 0.0f},
};

// Update vars
float pitch = 0.0f;
float rps = 0.0f;

void mod_mpu_update(float *fangle, float *frps){
    int16_t gy, gz;
    mod_mpu_read_raw_accel(&ax, &ay, &az);
    mod_mpu_read_raw_gyro(&gx, &gy, &gz);

    pitch = mod_mpu_update_pitch(ax, ay, az);

    rps = mod_mpu_update_rps(gx);

    mod_mpu_update_kalman(pitch, rps);

    *frps = kf.xp[0];
    *fangle = kf.xp[1];
}


void mod_mpu_task()
{
    int16_t gy, gz;
    mod_mpu_read_raw_accel(&ax, &ay, &az);
    mod_mpu_read_raw_gyro(&gx, &gy, &gz);

    pitch = mod_mpu_update_pitch(ax, ay, az);

    rps = mod_mpu_update_rps(gx);

    mod_mpu_update_kalman(pitch, rps);
}

float mod_mpu_get_pitch(){
    return kf.xp[1];
}

float mod_mpu_get_rps(){
    return kf.xp[0];
}

// Calc pitch in rads from accel x,y,z
float mod_mpu_update_pitch(int16_t fax, int16_t fay, int16_t faz)
{
    float ax_g = fax / 8192.0f;
    float ay_g = fay / 8192.0f;
    float az_g = faz / 8192.0f;

    float temp_pitch = atan2(ay_g, sqrt(ax_g*ax_g + az_g*az_g));
    //printf("Pitch: %.5f\n", temp_pitch);

    return temp_pitch;
}

// Calc rad per second from gyro x
float mod_mpu_update_rps(int16_t fgx)
{
    float temp_rps = -(3.14159f * (float)(fgx)) / (65.0f * 180.0f);
    //printf("RPS: %.5f\n", temp_rps);

    return temp_rps;
}

//  kalman updatey
void mod_mpu_update_kalman(float angle_accel, float velocity_gyro)
{
    float PmC[2][3];
    float S[2][2], invS[2][2];
    float PmCt[3][2];
    float Kc[3][3];
    float M[3][3];
    float AP[3][3];
    float detS;
    float res1, res2;

    // Pack measurement
    kf.y[0] = angle_accel;
    kf.y[1] = velocity_gyro;

    // === Correction step ===
    // Compute C*Pm (2x3), with C = [0 1 0; 1 0 1]
    PmC[0][0] = kf.Pm[1][0];
    PmC[0][1] = kf.Pm[1][1];
    PmC[0][2] = kf.Pm[1][2];

    PmC[1][0] = kf.Pm[0][0] + kf.Pm[2][0];
    PmC[1][1] = kf.Pm[0][1] + kf.Pm[2][1];
    PmC[1][2] = kf.Pm[0][2] + kf.Pm[2][2];

    // S = C*Pm*C' + R
    S[0][0] = PmC[0][1] + kf.R[0][0];
    S[0][1] = PmC[0][0] + PmC[0][2] + kf.R[0][1];
    S[1][0] = PmC[1][1] + kf.R[1][0];
    S[1][1] = PmC[1][0] + PmC[1][2] + kf.R[1][1];

    // Inverse of S
    detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    invS[0][0] =  S[1][1]/detS;
    invS[0][1] = -S[0][1]/detS;
    invS[1][0] = -S[1][0]/detS;
    invS[1][1] =  S[0][0]/detS;

    // Compute Pm*C' (3x2)
    PmCt[0][0] = kf.Pm[0][1];
    PmCt[0][1] = kf.Pm[0][0] + kf.Pm[0][2];

    PmCt[1][0] = kf.Pm[1][1];
    PmCt[1][1] = kf.Pm[1][0] + kf.Pm[1][2];

    PmCt[2][0] = kf.Pm[2][1];
    PmCt[2][1] = kf.Pm[2][0] + kf.Pm[2][2];

    // Compute Kk = Pm*C' * inv(S) (3x2)
    for (int r=0; r<3; r++) {
        kf.Kk[r][0] = PmCt[r][0]*invS[0][0] + PmCt[r][1]*invS[1][0];
        kf.Kk[r][1] = PmCt[r][0]*invS[0][1] + PmCt[r][1]*invS[1][1];
    }

    // Residual
    res1 = kf.y[0] - kf.xm[1];
    res2 = kf.y[1] - (kf.xm[0] + kf.xm[2]);

    // Correction xp = xm + Kk*residual
    kf.xp[0] = kf.xm[0] + kf.Kk[0][0]*res1 + kf.Kk[0][1]*res2;
    kf.xp[1] = kf.xm[1] + kf.Kk[1][0]*res1 + kf.Kk[1][1]*res2;
    kf.xp[2] = kf.xm[2] + kf.Kk[2][0]*res1 + kf.Kk[2][1]*res2;

    // Kk*C (3x3)
    Kc[0][0] = kf.Kk[0][1];   Kc[0][1] = kf.Kk[0][0];   Kc[0][2] = kf.Kk[0][1];
    Kc[1][0] = kf.Kk[1][1];   Kc[1][1] = kf.Kk[1][0];   Kc[1][2] = kf.Kk[1][1];
    Kc[2][0] = kf.Kk[2][1];   Kc[2][1] = kf.Kk[2][0];   Kc[2][2] = kf.Kk[2][1];

    // M = I - Kk*C
    M[0][0] = 1.0f - Kc[0][0];   M[0][1] = -Kc[0][1];        M[0][2] = -Kc[0][2];
    M[1][0] = -Kc[1][0];         M[1][1] = 1.0f - Kc[1][1];  M[1][2] = -Kc[1][2];
    M[2][0] = -Kc[2][0];         M[2][1] = -Kc[2][1];        M[2][2] = 1.0f - Kc[2][2];

    // Pp = (I - Kk*C)*Pm
    for (int r=0; r<3; r++) {
        for (int c=0; c<3; c++) {
            kf.Pp[r][c] = M[r][0]*kf.Pm[0][c] +
                          M[r][1]*kf.Pm[1][c] +
                          M[r][2]*kf.Pm[2][c];
        }
    }

    // === Prediction step ===
    // xm = Ad*xp
    kf.xm[0] = kf.xp[0];
    kf.xm[1] = kf.Ad[1][0]*kf.xp[0] + kf.xp[1]; // T*xp1 + xp2
    kf.xm[2] = kf.xp[2];

    // AP = Ad*Pp
    AP[0][0] = kf.Pp[0][0];  AP[0][1] = kf.Pp[0][1];  AP[0][2] = kf.Pp[0][2];
    AP[1][0] = kf.Ad[1][0]*kf.Pp[0][0] + kf.Pp[1][0];
    AP[1][1] = kf.Ad[1][0]*kf.Pp[0][1] + kf.Pp[1][1];
    AP[1][2] = kf.Ad[1][0]*kf.Pp[0][2] + kf.Pp[1][2];
    AP[2][0] = kf.Pp[2][0];  AP[2][1] = kf.Pp[2][1];  AP[2][2] = kf.Pp[2][2];

    // Pm = (Ad*Pp)*Ad' + Q
    kf.Pm[0][0] = AP[0][0] + kf.Q[0][0];
    kf.Pm[0][1] = AP[0][0]*kf.Ad[1][0] + AP[0][1] + kf.Q[0][1];
    kf.Pm[0][2] = AP[0][2] + kf.Q[0][2];

    kf.Pm[1][0] = AP[1][0] + kf.Q[1][0];
    kf.Pm[1][1] = AP[1][0]*kf.Ad[1][0] + AP[1][1] + kf.Q[1][1];
    kf.Pm[1][2] = AP[1][2] + kf.Q[1][2];

    kf.Pm[2][0] = AP[2][0] + kf.Q[2][0];
    kf.Pm[2][1] = AP[2][0]*kf.Ad[1][0] + AP[2][1] + kf.Q[2][1];
    kf.Pm[2][2] = AP[2][2] + kf.Q[2][2];
}

// Get pitch and rps
void mod_mpu_get_states(float *fangle, float *frps){
    *frps = kf.xp[0];
    *fangle = kf.xp[1];
}

// Reads raw accelerometer data from the MPU6050
HAL_StatusTypeDef mod_mpu_read_raw_accel(int16_t *fax, int16_t *fay, int16_t *faz)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 6, HAL_MAX_DELAY);

    if (res == HAL_OK)
    {
        *fax = (int16_t)(buffer[0] << 8 | buffer[1]);
        *fay = (int16_t)(buffer[2] << 8 | buffer[3]);
        *faz = (int16_t)(buffer[4] << 8 | buffer[5]);
    }

    return res;
}

// Reads raw gyroscope data from the MPU6050
HAL_StatusTypeDef mod_mpu_read_raw_gyro(int16_t *fgx, int16_t *fgy, int16_t *fgz)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buffer, 6, HAL_MAX_DELAY);

    if (res == HAL_OK)
    {
        *fgx = (int16_t)(buffer[0] << 8 | buffer[1]);
        *fgy = (int16_t)(buffer[2] << 8 | buffer[3]);
        *fgz = (int16_t)(buffer[4] << 8 | buffer[5]);
    }

    return res;
}

// config innit
void mod_mpu_configure_hardware()
{
    if (!_is_init)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};

        // PB8 (SCL) and PB9 (SDA)
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        hi2c1.Instance = I2C1;
        hi2c1.Init.ClockSpeed = 400000;
        hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
        hi2c1.Init.OwnAddress1 = 0;
        hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        hi2c1.Init.OwnAddress2 = 0;
        hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
        HAL_I2C_Init(&hi2c1);

        // Initialize the MPU6050
        uint8_t data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);

        // Set sample rate to 1kHz (1MHz / (1 + 0))
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_SMPLRT_DIV_REG, 1, &data, 1, HAL_MAX_DELAY);

        // Disable DLPF for maximum bandwidth
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);

        // Set gyro range to ±500°/s
        data = 0x08;  // 0x08 = ±500°/s
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

        // Set accel range to ±4g
        data = 0x08;  // 0x08 = ±4g
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    }
}


void mod_mpu_init(void)
{
    // Init big module
    if (!_is_init)
    {
        mod_mpu_configure_hardware();

        _modMPUTimerID = osTimerNew(mod_mpu_task, osTimerPeriodic, NULL, &_modMPUTimerAttr);
        _is_init = 1;
    }
}

// Start the module mpu task
void mod_mpu_start()
{
    // Get it started baby
    if (!_is_running)
    {
        osTimerStart(_modMPUTimerID, MPU_PERIOD_MS);
        _is_running = 1;
    }
}

// Stop the module mpu task
void mod_mpu_stop(void)
{
    if (_is_running)
    {
        osTimerStop(_modMPUTimerID);
        _is_running = 0;
    }
}