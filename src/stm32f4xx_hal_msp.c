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

DMA_HandleTypeDef hdma_adc1;
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    // PA0: ADC1_IN0
    // PA1: ADC1_IN1
    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_DMA2_CLK_ENABLE(); // or DMA1, depending on your STM32

        // Configure DMA stream/channel for ADC1
        // Example for STM32F4:
        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
        hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_adc1);

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

        // Enable DMA interrupt if needed
        HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}