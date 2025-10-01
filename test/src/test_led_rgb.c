#include "unity_fixture.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "led_rgb.h"

TEST_GROUP(RGB_LED);

TEST_SETUP(RGB_LED)
{
    UT_PTR_SET(HAL_TIM_PWM_Init, HAL_TIM_PWM_Init_spyHandle);
    UT_PTR_SET(HAL_TIM_PWM_ConfigChannel, HAL_TIM_PWM_ConfigChannel_spyInitTypeDef);
    UT_PTR_SET(HAL_GPIO_Init, HAL_GPIO_Init_spyInitTypeDef);
    HAL_TIM_resetSpy();
}

TEST_TEAR_DOWN(RGB_LED)
{
    UnityPointer_UndoAllSets();
    led_rgb_deinit();
}

TEST(RGB_LED, PinConfig)
{
    led_rgb_init();
    TEST_ASSERT_MESSAGE(GPIO_PIN_6 == (GPIO_Init_Spy.Pin & GPIO_PIN_6), "Expected GPIO to initialise Pin 6");
    TEST_ASSERT_MESSAGE(GPIO_PIN_8 == (GPIO_Init_Spy.Pin & GPIO_PIN_8), "Expected GPIO to initialise Pin 8");
    TEST_ASSERT_MESSAGE(GPIO_PIN_9 == (GPIO_Init_Spy.Pin & GPIO_PIN_9), "Expected GPIO to initialise Pin 9");
    TEST_ASSERT_MESSAGE(GPIO_MODE_AF_PP == GPIO_Init_Spy.Mode, "Expected GPIO mode to be 'Alternate Function Push Pull Output'");
    TEST_ASSERT_MESSAGE(GPIO_PULLUP == GPIO_Init_Spy.Pull, "Expected LED pullup resistors to be enabled");
    TEST_ASSERT_MESSAGE(GPIO_AF2_TIM4 == GPIO_Init_Spy.Alternate, "Expected GPIO alternate function for Timer 4");
}

TEST(RGB_LED, RemovedGPIOWritePinExamples)
{
    led_rgb_init();
    led_rgb_set_red(0);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(GPIO_PIN_6, (mock_gpioc.BSRR & GPIO_PIN_6), "Remove the template HAL_GPIO_WritePin() from led_rgb_set_red()!");
    led_rgb_set_green(0);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(GPIO_PIN_8, (mock_gpioc.BSRR & GPIO_PIN_8), "Remove the template HAL_GPIO_WritePin() from led_rgb_set_green()!");
    led_rgb_set_blue(0);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(GPIO_PIN_9, (mock_gpioc.BSRR & GPIO_PIN_9), "Remove the template HAL_GPIO_WritePin() from led_rgb_set_blue()!");
    // GPIO_Init_Spy.Instance->BSRR == GPIO_PIN_8;
    // GPIO_Init_Spy.Instance->BSRR == GPIO_PIN_9;
}

TEST(RGB_LED, ClocksInMSPInit)
{
    TIM_HandleTypeDef   htim4;
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = 0;

    __HAL_RCC_TIM4_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    HAL_TIM_PWM_MspInit(&htim4);
    TEST_ASSERT_MESSAGE(1 == __HAL_RCC_GPIOB_IS_CLK_ENABLED(), "Expected GPIOB clock to be enabled in HAL_TIM_PWM_MspInit().");
    TEST_ASSERT_MESSAGE(1 == __HAL_RCC_TIM4_IS_CLK_ENABLED(), "Expected TIM4 clock to be enabled in HAL_TIM_PWM_MspInit().");
}

TEST(RGB_LED, GPIOClockNotInRGBInit)
{
    UnityPointer_UndoAllSets();
    UT_PTR_SET(HAL_TIM_PWM_Init, HAL_TIM_PWM_Init_noMSPInit);
    UT_PTR_SET(HAL_TIM_PWM_ConfigChannel, HAL_TIM_PWM_ConfigChannel_spyInitTypeDef);
    __HAL_RCC_GPIOB_CLK_DISABLE();
    led_rgb_init();
    TEST_ASSERT_MESSAGE(1 == __HAL_RCC_GPIOB_IS_CLK_DISABLED(), "Expected GPIOB clock enable to be removed from led_rgb_init().");
}

TEST(RGB_LED, TimerConfig)
{
    led_rgb_init();
    TEST_ASSERT_MESSAGE(timerSpy.State == HAL_TIM_STATE_READY, "HAL_TIM_PWM_Init() was not called.");
    TEST_ASSERT_MESSAGE(timerSpy.Instance != NULL, "Timer instance needs to be set! Expect weird errors if you don't set this.");
    TEST_ASSERT_MESSAGE(TIM4 == timerSpy.Instance,"Expected timer 4 instance");
    TEST_ASSERT_MESSAGE(1 == timerSpy.Init.Prescaler,"Expected prescaler of 1");
    TEST_ASSERT_MESSAGE(TIM_COUNTERMODE_UP == timerSpy.Init.CounterMode,"Expected timer to count up");
    TEST_ASSERT_MESSAGE(65535 == timerSpy.Init.Period,"Expected timer period of 65535");
    TEST_ASSERT_MESSAGE(0 == timerSpy.Init.ClockDivision, "Expected timer clock divider of 0");

    TEST_ASSERT_MESSAGE(0xFFFF != timerOCInitSpy.OCMode, "HAL_TIM_PWM_ConfigChannel() was not called or OCMode element not set!");
    if(timerOCInitSpy.OCMode == TIM_OCMODE_PWM2)
    {
        TEST_ASSERT_MESSAGE(TIM_OCPOLARITY_HIGH == timerOCInitSpy.OCPolarity,"Expected output compare polarity to be high for PWM mode 2");
    }
    else if(timerOCInitSpy.OCMode == TIM_OCMODE_PWM1)
    {
        TEST_ASSERT_MESSAGE(TIM_OCPOLARITY_LOW == timerOCInitSpy.OCPolarity,"Expected output compare polarity to be low for PWM mode 1");
    }
    else
    {
        TEST_FAIL_MESSAGE("Expected output compare mode to be set to PWM mode 1 or 2");
    }
    TEST_ASSERT_EQUAL_UINT32(TIM_OCFAST_DISABLE, timerOCInitSpy.OCFastMode);
    /* Check channel 1, 3, and 4 were initialised using PWM_ConfigChannel */
    TEST_ASSERT_MESSAGE((mockTIM4.CCMR1 & timerOCInitSpy.OCMode) == timerOCInitSpy.OCMode, "Expected TIM_CHANNEL_1 to be configured using HAL_TIM_PWM_ConfigChannel().");
    TEST_ASSERT_MESSAGE((mockTIM4.CCMR2 & timerOCInitSpy.OCMode) == timerOCInitSpy.OCMode, "Expected TIM_CHANNEL_3 to be configured using HAL_TIM_PWM_ConfigChannel().");
    TEST_ASSERT_MESSAGE((mockTIM4.CCMR2 & (timerOCInitSpy.OCMode << 8U)) == (timerOCInitSpy.OCMode << 8U), "Expected TIM_CHANNEL_4 to be configured using HAL_TIM_PWM_ConfigChannel().");

    /* Check channels 1, 3 and 4 were started using PWM_Start */
    TEST_ASSERT_MESSAGE((mockTIM4.CCER & (uint32_t)(TIM_CCx_ENABLE << TIM_CHANNEL_1)) == (uint32_t)(TIM_CCx_ENABLE << TIM_CHANNEL_1), "Expected TIM_CHANNEL_1 to be started using HAL_TIM_PWM_Start().");
    TEST_ASSERT_MESSAGE((mockTIM4.CCER & (uint32_t)(TIM_CCx_ENABLE << TIM_CHANNEL_3)) == (uint32_t)(TIM_CCx_ENABLE << TIM_CHANNEL_3), "Expected TIM_CHANNEL_3 to be started using HAL_TIM_PWM_Start().");
    TEST_ASSERT_MESSAGE((mockTIM4.CCER & (uint32_t)(TIM_CCx_ENABLE << TIM_CHANNEL_4)) == (uint32_t)(TIM_CCx_ENABLE << TIM_CHANNEL_4), "Expected TIM_CHANNEL_4 to be started using HAL_TIM_PWM_Start().");

}

TEST(RGB_LED, DefaultBrightness)
{
    TEST_ASSERT_EQUAL_UINT16(0, led_rgb_get_red());
}

TEST(RGB_LED, SetBrightness)
{
    led_rgb_set_red(1234);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(1234, led_rgb_get_red(),"Expected red to set/get compare register for TIM4 CH1");
    led_rgb_set_green(4321);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(4321, led_rgb_get_green(),"Expected green to set/get compare register for TIM4 CH3");
    led_rgb_set_blue(1324);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(1324, led_rgb_get_blue(),"Expected blue to set/get compare register for TIM4 CH4");
}

TEST_GROUP_RUNNER(RGB_LED)
{
    RUN_TEST_CASE(RGB_LED, PinConfig);
    RUN_TEST_CASE(RGB_LED, ClocksInMSPInit);
    RUN_TEST_CASE(RGB_LED, GPIOClockNotInRGBInit);
    RUN_TEST_CASE(RGB_LED, TimerConfig);
    RUN_TEST_CASE(RGB_LED, RemovedGPIOWritePinExamples);
    RUN_TEST_CASE(RGB_LED, DefaultBrightness);
    RUN_TEST_CASE(RGB_LED, SetBrightness);
}