#include "unity_fixture.h"

#include <stdint.h>
#include "potentiometer.h"
#include "stm32f4xx_hal.h"

TEST_GROUP(Potentiometer);

TEST_SETUP(Potentiometer)
{
    UT_PTR_SET(HAL_ADC_Init, HAL_ADC_Init_spyHandle);
    UT_PTR_SET(HAL_ADC_ConfigChannel, HAL_ADC_ConfigChannel_spy);
    UT_PTR_SET(HAL_GPIO_Init, HAL_GPIO_Init_spyInitTypeDef);
}

TEST_TEAR_DOWN(Potentiometer)
{
    UnityPointer_UndoAllSets();
}

TEST(Potentiometer, ADCconfig)
{
    pot_init();
    TEST_ASSERT_MESSAGE(ADC_RESOLUTION_12B == ADC_Init_Spy.Init.ClockPrescaler, "Expected ADC prescaler of 2");
    TEST_ASSERT_MESSAGE(ADC_RESOLUTION_12B == ADC_Init_Spy.Init.Resolution, "Expected 12 bit ADC resolution");
    TEST_ASSERT_MESSAGE(ADC_DATAALIGN_LEFT == ADC_Init_Spy.Init.DataAlign, "Expected to configure left aligned output");
    TEST_ASSERT_MESSAGE(ENABLE == ADC_Init_Spy.Init.ContinuousConvMode, "Expected to configure for continuous conversion");
}

TEST(Potentiometer, ChannelConfig)
{
    pot_init();
    TEST_ASSERT_MESSAGE(ADC_CHANNEL_0 == ADC_ChannelConf_Spy.Channel, "Expected to configure CH0");
    TEST_ASSERT_MESSAGE(ADC_SAMPLETIME_480CYCLES == ADC_ChannelConf_Spy.SamplingTime, "Expected 480 cycle sample time");
}

TEST(Potentiometer, ClocksInMSPInit)
{
    ADC_HandleTypeDef hadc1;
    hadc1.Instance = ADC1;
    __HAL_RCC_ADC1_CLK_DISABLE();
    __HAL_RCC_GPIOA_CLK_DISABLE();
    HAL_ADC_MspInit(&hadc1);
    TEST_ASSERT_MESSAGE(1 == __HAL_RCC_ADC1_IS_CLK_ENABLED(), "Expected ADC1 clock to be enabled in HAL_ADC_MspInit().");
    TEST_ASSERT_MESSAGE(1 == __HAL_RCC_GPIOA_IS_CLK_ENABLED(), "Expected GPIOA clock to be enabled in HAL_ADC_MspInit().");
}

TEST(Potentiometer, PinConfig)
{
    ADC_HandleTypeDef hadc1;
    hadc1.Instance = ADC1;
    HAL_ADC_MspInit(&hadc1);
    TEST_ASSERT_MESSAGE(GPIO_PIN_0 == (GPIO_Init_Spy.Pin & GPIO_PIN_0), "Expected HAL_ADC_MspInit() to initialise PA0");
    TEST_ASSERT_MESSAGE(GPIO_MODE_ANALOG == GPIO_Init_Spy.Mode, "Expected GPIO mode to be 'Analog Input'");
    TEST_ASSERT_MESSAGE(GPIO_NOPULL == GPIO_Init_Spy.Pull, "Expected ADC pullup resistors to be disabled");
}

TEST(Potentiometer, ADCisStarted)
{
    ADC_HandleTypeDef hadc1;
    hadc1.Instance = ADC1;
    // ADC_IS_ENABLE(&hadc1);

    TEST_ASSERT_MESSAGE(1 == ADC_IS_ENABLE(&hadc1), "Expected HAL_ADC_Start() to be called");
}

TEST(Potentiometer, GetValueFromADC)
{
    uint16_t expected = 123;
    mock_adc1.DR = 123;
    uint16_t actual = pot_get_value();
    TEST_ASSERT_EQUAL_UINT16(expected, actual);
}

TEST_GROUP_RUNNER(Potentiometer)
{
    RUN_TEST_CASE(Potentiometer, ADCconfig);
    RUN_TEST_CASE(Potentiometer, ChannelConfig);
    RUN_TEST_CASE(Potentiometer, PinConfig);
    RUN_TEST_CASE(Potentiometer, ClocksInMSPInit);
    RUN_TEST_CASE(Potentiometer, ADCisStarted);
    RUN_TEST_CASE(Potentiometer, GetValueFromADC);
}