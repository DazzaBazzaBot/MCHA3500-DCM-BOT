#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "stdlib.h"
#include "math.h"

#include "mod_adc.h"

#define ADC_NUM_CHANNELS 2

// Hardware
static ADC_HandleTypeDef hadc1 = {0};
static ADC_ChannelConfTypeDef sConfigADC = {0};

// Flags
static uint8_t _is_init = 0;

// buffer for adc readings
static uint16_t adc_dma_buffer[ADC_NUM_CHANNELS];

void mod_adc_update_readings(float *voltage_1, float *voltage_2)
{
    *voltage_1 = (adc_dma_buffer[0] * 3.3f) / 4096.0f;
    *voltage_2 = (adc_dma_buffer[1] * 3.3f) / 4096.0f;
}

float mod_adc_get_voltageA(void)
{
    return (adc_dma_buffer[0] * 3.3f) / 4096.0f;
}

float mod_adc_get_voltageB(void)
{
    return (adc_dma_buffer[1] * 3.3f) / 4096.0f;
}

/**************************************
 * Configure the ADC hardware
 * PA0: ADC1_IN0, ADCA, right motor current
 * PA1: ADC1_IN1, ADCB, left motor current
 **************************************/
void mod_adc_configure_hardware(void)
{
    if (!_is_init)
    {
        // Enable ADC1 clock
        __HAL_RCC_ADC1_CLK_ENABLE();

        hadc1.Instance = ADC1;
        hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
        hadc1.Init.Resolution = ADC_RESOLUTION_12B;
        hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc1.Init.ScanConvMode = ENABLE;
        hadc1.Init.ContinuousConvMode = ENABLE; // Enable continuous mode for DMA
        hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
        hadc1.Init.NbrOfConversion = ADC_NUM_CHANNELS;
        hadc1.Init.DiscontinuousConvMode = DISABLE;
        hadc1.Init.DMAContinuousRequests = ENABLE;

        HAL_ADC_Init(&hadc1);

        sConfigADC.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        sConfigADC.Offset = 0;

        // Channel 0, rank 1
        sConfigADC.Channel = ADC_CHANNEL_0;
        sConfigADC.Rank = 1;
        HAL_ADC_ConfigChannel(&hadc1, &sConfigADC);

        // Channel 1, rank 2
        sConfigADC.Channel = ADC_CHANNEL_1;
        sConfigADC.Rank = 2;
        HAL_ADC_ConfigChannel(&hadc1, &sConfigADC);

        // Start ADC in DMA mode
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_NUM_CHANNELS);

        _is_init = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
}