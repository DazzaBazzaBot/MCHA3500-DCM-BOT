#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "stdlib.h"
#include "math.h"

#include "mod_adc.h"

// Hardware
static ADC_HandleTypeDef hadc1 = {0};
static ADC_ChannelConfTypeDef sConfigADC = {0};

// Flags
static uint8_t _is_init = 0;
static uint8_t _is_running = 0;





void mod_adc_update_readings(float *voltage_1, float *voltage_2)
{
    float adcValue = 0.0f;

    // Start ADC conversion
    HAL_ADC_Start(&hadc1);

    // Poll for conversion completion
    HAL_ADC_PollForConversion(&hadc1, 10);
    adcValue = HAL_ADC_GetValue(&hadc1);
    *voltage_1 = (adcValue * 3.3f) / 4096.0f;

    HAL_ADC_PollForConversion(&hadc1, 10);
    adcValue = HAL_ADC_GetValue(&hadc1);
    *voltage_2 = (adcValue * 3.3f) / 4096.0f;

    // Stop ADC
    HAL_ADC_Stop(&hadc1);
}

/**************************************
 * Configure the ADC hardware
 * PA0: ADC1_IN0, Left Motor Current
 * PA1: ADC1_IN1, Right Motor Current
 **************************************/
void mod_adc_configure_hardware(void)
{
    if (!_is_init)
    {
        // Enable ADC1 clock
        __HAL_RCC_ADC1_CLK_ENABLE();

        // COnfigure ADC1
        hadc1.Instance = ADC1;
        hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
        hadc1.Init.Resolution = ADC_RESOLUTION_12B;
        hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc1.Init.ScanConvMode = ENABLE;
        hadc1.Init.ContinuousConvMode = DISABLE;
        hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
        hadc1.Init.NbrOfConversion = 2;
        hadc1.Init.DiscontinuousConvMode = DISABLE;

        // Initialize ADC
        HAL_ADC_Init(&hadc1);

        // Configure common parameters
        sConfigADC.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        sConfigADC.Offset = 0;

        // Configure channel 0, rank 1
        sConfigADC.Channel = ADC_CHANNEL_0;
        sConfigADC.Rank = 1;
        HAL_ADC_ConfigChannel(&hadc1, &sConfigADC);

        // Configure channel 1, rank 2
        sConfigADC.Channel = ADC_CHANNEL_1;
        sConfigADC.Rank = 2;
        HAL_ADC_ConfigChannel(&hadc1, &sConfigADC);
    }
}