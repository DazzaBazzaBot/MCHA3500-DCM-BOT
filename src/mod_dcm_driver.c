#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "stdlib.h"
#include "math.h"

#include "mod_dcm_driver.h"

#define MAX_VOLTAGE 12.0f

// Hardware
static GPIO_InitTypeDef GPIO_InitStruct = {0};
static TIM_HandleTypeDef htim2 = {0};
static TIM_HandleTypeDef htim3 = {0};
static TIM_OC_InitTypeDef sConfigPWM = {0};


void mod_dcm_set_PWM(float pwm_percent_left, float pwm_percent_right)
{
    pwm_percent_left = fmin(fmax(pwm_percent_left, -100), 100);
    pwm_percent_right = fmin(fmax(pwm_percent_right, -100), 100);

    if (pwm_percent_left > 0)
    {
        left_forward();
    }
    else if (pwm_percent_left < 0)
    {
        left_reverse();
    }
    else
    {
        left_halt();
    }

    if (pwm_percent_right > 0)
    {
        right_forward();
    }
    else if (pwm_percent_right < 0)
    {
        right_reverse();
    }
    else
    {
        right_halt();
    }
    uint16_t new_pwm_left = abs(pwm_percent_left) * 5000.0f / 100.0f + 5000.0f;
    uint16_t new_pwm_right = abs(pwm_percent_right) * 5000.0f / 100.0f + 5000.0f;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, new_pwm_left);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, new_pwm_right);

}

void mod_dcm_set_voltage(float voltage_left, float voltage_right)
{
    float pwm_percent_left = (int8_t)(voltage_left / MAX_VOLTAGE * 100.0f);
    float pwm_percent_right = (int8_t)(voltage_right / MAX_VOLTAGE * 100.0f);

    mod_dcm_set_PWM(pwm_percent_left, pwm_percent_right);
}

void mod_dcm_set_voltage_log(float voltage)
{
    float pwm_percent_left = (int8_t)(voltage / MAX_VOLTAGE * 100.0f);
    pwm_percent_left = fmin(fmax(pwm_percent_left, -100), 100);

    if (pwm_percent_left > 0)
    {
        left_forward();
    }
    else if (pwm_percent_left < 0)
    {
        left_reverse();
    }
    else
    {
        left_halt();
    }

    uint16_t new_pwm_left = abs(pwm_percent_left) * 5000.0f / 100.0f + 5000.0f;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, new_pwm_left);
}

void left_forward()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void left_reverse()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void left_halt()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void right_forward()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

void right_reverse()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void right_halt()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}


void mod_dcm_set_voltageLeft(float voltage)
{
    float pwm_percent_left = (int8_t)(voltage / MAX_VOLTAGE * 100.0f);
    pwm_percent_left = fmin(fmax(pwm_percent_left, -100), 100);

    if (pwm_percent_left > 0)
    {
        left_forward();
    }
    else if (pwm_percent_left < 0)
    {
        left_reverse();
    }
    else
    {
        left_halt();
    }

    uint16_t new_pwm_left = abs(pwm_percent_left) * 5000.0f / 100.0f + 5000.0f;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, new_pwm_left);
}

void mod_dcm_set_voltageRight(float voltage)
{
    float pwm_percent_right = -(int8_t)(voltage / MAX_VOLTAGE * 100.0f);
    pwm_percent_right = fmin(fmax(pwm_percent_right, -100), 100);

    if (pwm_percent_right > 0)
    {
        right_forward();
    }
    else if (pwm_percent_right < 0)
    {
        right_reverse();
    }
    else
    {
        right_halt();
    }

    uint16_t new_pwm_right = abs(pwm_percent_right) * 5000.0f / 100.0f + 5000.0f;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, new_pwm_right);
}


/**************************************
 * Configure the DC Motor hardware
 * PB10: PWM signal, TIM2, Channel3, motor left
 * PA8: IN1, motor left
 * PC7: IN2, motor left
 * PB6: IN3, motor right
 * PA7: IN4, motor right
 * PA6: PWM signal, TIM3, Channel1, motor right
 **************************************/
void mod_dcm_configure_hardware(void)
{
    // Enable GPIO Clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure TIM2
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 4;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9999;
    htim2.Init.ClockDivision = 0;
    HAL_TIM_PWM_Init(&htim2);

    // Configure TIM3
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 4;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 9999;
    htim3.Init.ClockDivision = 0;
    HAL_TIM_PWM_Init(&htim3);

    // Configure PWM channels
    sConfigPWM.OCMode = TIM_OCMODE_PWM1;
    sConfigPWM.Pulse = 0;
    sConfigPWM.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigPWM.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigPWM.Pulse = 0;

    // Configure and Start TIM2 Channel 1
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigPWM, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Configure and Start TIM3 Channel 1
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigPWM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // Configure GPIO pins for motor control
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_7; // PA8, PA7
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6; // PB6
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7; // PC7
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}