#include "stm32f4xx_hal.h"

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    // PA5: Heartbeat LED, TIM2_CH1
    // PA6: DCM1 PWM, TIM3_CH1
    // PB10: DCM2 PWM, TIM2_CH3
    if (htim->Instance == TIM2)
    {
        // Enable peripheral clocks
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        // PA5
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        GPIO_InitStructure.Pin = GPIO_PIN_5;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStructure.Pull = GPIO_PULLUP;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

        // PB10
        GPIO_InitStructure.Pin = GPIO_PIN_10;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
    else if (htim->Instance == TIM3)
    {
        // Enable peripheral clocks
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // PA6
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        GPIO_InitStructure.Pin = GPIO_PIN_6;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    // PA0: ADC1_IN0
    // PA1: ADC1_IN1
    if (hadc->Instance == ADC1)
    {
        // Enable peripheral clocks
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // Configure PA0 and PA1 in analog mode, no pullup
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
}