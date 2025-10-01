#include "unity_fixture.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "button.h"

static osEventFlagsId_t _spyButtonEventID;
static osThreadId_t _spybutton_task_id;

TEST_GROUP(Button);
TEST_SETUP(Button)
{
    mockThreadTeardownAll(); //Has to act as an 'init' function otherwise the array of mocks is left with undefined values on startup.
    // dimmer_task_init();
    UT_PTR_SET(HAL_GPIO_Init, HAL_GPIO_Init_spyInitTypeDef);
    button_init();
    _spyButtonEventID = spyGetEventId("button");
    _spybutton_task_id = spyGetThreadId("button");
    //Test for correct task id so all tests fail before executing functions with an invalid pointer
    TEST_ASSERT_NOT_NULL_MESSAGE(_spybutton_task_id, "Expected thread to be started using osThreadNew() and thread name attr to be 'button'");
    TEST_ASSERT_NOT_NULL_MESSAGE(_spyButtonEventID, "Expected osEventFlag to be created and given name 'button' in function button_init()");
    mock_gpioc.IDR = 0;
}
static void raise_PC7(void) { mock_gpioc.IDR |= GPIO_PIN_7; }       // Raise PC7
static void lower_PC7(void) { mock_gpioc.IDR &= ~(GPIO_PIN_7); }    // Lower PC7


TEST_TEAR_DOWN(Button)
{
    UnityPointer_UndoAllSets();
    mockThreadTeardownAll();    //Alex 16/05/19: this was commented out previously, couldn't remember why. un-commented it
    mockEventTeardownAll();
    button_deinit();
}

TEST(Button, GPIOConfig)
{
    TEST_ASSERT_MESSAGE(GPIO_PIN_7 == (GPIO_Init_Spy.Pin & GPIO_PIN_7), "Expected GPIO PC7 to be initialised");
    TEST_ASSERT_MESSAGE(GPIO_MODE_IT_RISING_FALLING == GPIO_Init_Spy.Mode, "Expected GPIO mode to be 'Interrupt Rising and Falling Edge' for RTOS debounce");
    TEST_ASSERT_MESSAGE(GPIO_PULLDOWN == GPIO_Init_Spy.Pull, "Expected Button pulldown resistor to be enabled");
}

TEST(Button, GPIOClock)
{
    button_deinit();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    button_init();
    TEST_ASSERT_MESSAGE(1 == __HAL_RCC_GPIOC_IS_CLK_ENABLED(), "Expected GPIOC clock to be enabled in button_init().");
}

TEST(Button, InterruptEnabled)
{
    button_deinit();
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    button_init();
    TEST_ASSERT_MESSAGE(1 == NVIC_CheckEnableIRQspy(EXTI9_5_IRQn), "Expected EXTI9_5 to be enabled in the NVIC");
}

TEST(Button, InterruptCleared)
{
    __HAL_GPIO_EXTI_SET_IT(GPIO_PIN_7);
    EXTI9_5_IRQHandler();
    TEST_ASSERT_MESSAGE(0 == __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7), "HAL_GPIO_EXTI_IRQHandler() must be called and clear the Pin 4 intterupt flag in EXTI9_5_IRQHandler()");
}

TEST(Button, InterruptPressed)
{
	button_isr();
    uint32_t testGet = osEventFlagsGet(_spyButtonEventID);
    TEST_ASSERT_EQUAL_MESSAGE(1, testGet, "Expected event flag to be set in ISR. Ensure button_isr() is called from EXTI9_5_IRQHandler()");
}

TEST(Button, DefaultPressed)
{
    TEST_ASSERT_EQUAL_UINT8(0, button_get_pressed());
}

TEST(Button, PopPressed)
{
    button_set_pressed();
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, button_pop_pressed(), "Expected pop to return flag when it is set");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, button_get_pressed(), "Expected pop to clear flag");
}

TEST(Button, DebounceWithEventFlags)
{
    raise_PC7();        //Raise PC7 to test 'rising edge'
    button_isr();       //Sets BUTTON_SIGNAL event flag
    button_debounce();  //Waits for event flag reads GPIO pin and sets state.
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1,  button_get_pressed(), "Expected PC7 rising edge to set _was_pressed flag");
    uint32_t testGet = osEventFlagsGet(_spyButtonEventID);
    TEST_ASSERT_EQUAL_MESSAGE(0, testGet, "Expected event flags to be cleared at end of switch-case");

    lower_PC7();        //Lower PC7 to test 'falling edge'
    button_isr();       //Sets BUTTON_SIGNAL event flag
    button_debounce();  //Waits for event flag reads GPIO pin and sets state.
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1,  button_get_pressed(), "Expected PC7 falling edge to retain state of _was_pressed flag");
    testGet = osEventFlagsGet(_spyButtonEventID);
    TEST_ASSERT_EQUAL_MESSAGE(0, testGet, "Expected event flags to be cleared at end of switch-case");
}


//TODO Button thread tests

TEST_GROUP_RUNNER(Button)
{
    RUN_TEST_CASE(Button, GPIOConfig);
    RUN_TEST_CASE(Button, GPIOClock);
    RUN_TEST_CASE(Button, InterruptEnabled);
    RUN_TEST_CASE(Button, InterruptCleared);
    RUN_TEST_CASE(Button, DefaultPressed);
    RUN_TEST_CASE(Button, InterruptPressed);
    RUN_TEST_CASE(Button, PopPressed);
    RUN_TEST_CASE(Button, DebounceWithEventFlags);
}