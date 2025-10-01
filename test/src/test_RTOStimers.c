#include "unity_fixture.h"

#include <stdint.h>
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

void _RTOStimer_update(void *argument);
void _RTOStimer_update2(void *argument);

osTimerId_t _test_RTOStimer_id;

const osTimerAttr_t RTOSTimer_attr = 
{
    .name = "RTOStest_timer"
};

TEST_GROUP(RTOStimers);

TEST_SETUP(RTOStimers)
{
    _test_RTOStimer_id = osTimerNew(_RTOStimer_update, osTimerPeriodic, (void*)0, &RTOSTimer_attr);
    TEST_ASSERT_NOT_NULL_MESSAGE(_test_RTOStimer_id, "Expected timer ID to be returned");
}

TEST_TEAR_DOWN(RTOStimers)
{
    mockTimerTeardownAll();
}

TEST(RTOStimers, CallbackFuncSet)
{
    mock_timerHandle_t *hTimer;
    hTimer = mockGetTimerFromHandle(_test_RTOStimer_id);
    TEST_ASSERT_EQUAL_PTR(_RTOStimer_update, hTimer->pxCallbackFunction);
}

TEST(RTOStimers, AttributesSet)
{
    mock_timerHandle_t *hTimer;
    hTimer = mockGetTimerFromHandle(_test_RTOStimer_id);

    TEST_ASSERT_EQUAL_STRING(RTOSTimer_attr.name, hTimer->pcTimerName);
    TEST_ASSERT_EQUAL_STRING(RTOSTimer_attr.name, osTimerGetName(_test_RTOStimer_id));
    TEST_ASSERT_EQUAL_MESSAGE(osTimerPeriodic, hTimer->uxAutoReload, "Expected timer to be set to osPeriodic");
    TEST_ASSERT_EQUAL_MESSAGE(-1, hTimer->xTimerPeriodInTicks, "Expected timer period to be set to -1 when initailised but not started");
}

TEST(RTOStimers, GetStateAfterNewAndStart)
{
    osMockTimerState_t state = osTimerIsRunning(_test_RTOStimer_id);
    TEST_ASSERT_EQUAL_INT_MESSAGE(osTimerReady, state, "Expected timer to be ready but not started");

    osTimerStart(_test_RTOStimer_id, 10);
    state = osTimerIsRunning(_test_RTOStimer_id);
    TEST_ASSERT_EQUAL_INT_MESSAGE(osTimerRunning, state, "Expected timer to be running when started");
}

TEST(RTOStimers, GetStateAfterSuspend)
{
    osTimerStop(_test_RTOStimer_id);
    osMockTimerState_t state = osTimerIsRunning(_test_RTOStimer_id);
    TEST_ASSERT_EQUAL_INT_MESSAGE(osTimerStopped, state, "Expected timer to be stopped");
}

TEST(RTOStimers, SetTimerTicks)
{
    int test_ticks = 10;
    osTimerStart(_test_RTOStimer_id, test_ticks);
    mock_timerHandle_t *hTimer;
    hTimer = mockGetTimerFromHandle(_test_RTOStimer_id);
    TEST_ASSERT_EQUAL_INT_MESSAGE(test_ticks, hTimer->xTimerPeriodInTicks, "Expected timer to have the correct time when started");
}

TEST(RTOStimers, CleanUpTimersforTesting)
{
    mockTimerTeardownAll();
    mock_timerHandle_t *hTimer;
    hTimer = mockGetTimerFromHandle(_test_RTOStimer_id);
    TEST_ASSERT_EQUAL_PTR(NULL, hTimer->pxCallbackFunction);
    TEST_ASSERT_EQUAL_STRING(NULL, osTimerGetName(_test_RTOStimer_id));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(-1UL, osTimerIsRunning(_test_RTOStimer_id), "Expected timer to be paused");
}

TEST(RTOStimers, ResumeOrSuspendWithoutCreate)
{
    osTimerId_t _test_RTOStimer_id2;
    #pragma warning ( push )
    #pragma warning ( ignored "-Wuninitialized" ) //These pragmas might work for clang
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wuninitialized" //These pragmas work for GCC -- I need to ask Chris about this. If you get errors, sorry!
    
    TEST_ASSERT_EQUAL_INT_MESSAGE(NULL, spyCheckInitialisedTimerId(_test_RTOStimer_id2), "Expected non-initialised timer id to return NULL pointer");
    TEST_ASSERT_EQUAL_INT_MESSAGE(_test_RTOStimer_id, spyCheckInitialisedTimerId(_test_RTOStimer_id), "Expected initialised timer id to return correct pointer");
    TEST_ASSERT_EQUAL_INT_MESSAGE(osErrorParameter, osTimerStart(_test_RTOStimer_id2, 10), "Expected non-initialised timer id to send error when attempting to resume");
    TEST_ASSERT_EQUAL_INT_MESSAGE(osErrorParameter, osTimerStop(_test_RTOStimer_id2), "Expected non-initialised timer id to send error when attempting to suspend");
    
    #pragma GCC diagnostic pop
    #pragma warning ( pop )
}

TEST(RTOStimers, MultipleNewTimers)
{
    osTimerId_t _test_RTOStimer_id2;
    _test_RTOStimer_id2 = osTimerNew(_RTOStimer_update2, osTimerPeriodic, (void*)0, &RTOSTimer_attr);
    mock_timerHandle_t *hTimer2;
    hTimer2 = mockGetTimerFromHandle(_test_RTOStimer_id2);
    TEST_ASSERT_EQUAL_PTR(_RTOStimer_update2, hTimer2->pxCallbackFunction);
}

// TEST(RTOStimers, GuardAgainstMultipleofSame)
// {
//     osTimerId_t _test_RTOStimer_id2;
//     _test_RTOStimer_id2 = osTimerNew(_RTOStimer_update2, NULL, &RTOSTimer_attr);
//     mock_timerHandle_t *hTimer2;
//     hTimer2 = mockGetTimerFromHandle(_test_RTOStimer_id2);
//     TEST_ASSERT_EQUAL_PTR(_RTOStimer_update2, hTimer2->CallbackFunc);
// }

TEST(RTOStimers, GetTimerFromListUsingName)
{
    osTimerId_t _test_timerID, _target_timerID;
    osTimerAttr_t test_timer_attr = {.name = "dimmer"};
    _test_timerID = osTimerNew(_RTOStimer_update2, osTimerPeriodic, (void*)0, &test_timer_attr);
    _target_timerID = spyGetTimerId("dimmer");
    TEST_ASSERT_NOT_NULL(_target_timerID);
    TEST_ASSERT_EQUAL_PTR(_test_timerID, _target_timerID);
}

TEST(RTOStimers, GetTimerFromListUsingNameNullIfWrong)
{
    osTimerId_t _target_timerID;
    osTimerAttr_t test_timer_attr = {.name = "dimmer"};
    osTimerNew(_RTOStimer_update2, osTimerPeriodic, (void*)0, &test_timer_attr);
    _target_timerID = spyGetTimerId("dumber");
    TEST_ASSERT_NULL(_target_timerID);
}

TEST_GROUP_RUNNER(RTOStimers)
{
    RUN_TEST_CASE(RTOStimers, ResumeOrSuspendWithoutCreate);
    RUN_TEST_CASE(RTOStimers, CallbackFuncSet);
    RUN_TEST_CASE(RTOStimers, AttributesSet);
    RUN_TEST_CASE(RTOStimers, GetStateAfterNewAndStart);
    RUN_TEST_CASE(RTOStimers, GetStateAfterSuspend);
    RUN_TEST_CASE(RTOStimers, MultipleNewTimers);
    RUN_TEST_CASE(RTOStimers, CleanUpTimersforTesting);
    RUN_TEST_CASE(RTOStimers, GetTimerFromListUsingName);
    RUN_TEST_CASE(RTOStimers, GetTimerFromListUsingNameNullIfWrong);
}

void _RTOStimer_update(void *argument)
{
    (void) argument;

}
void _RTOStimer_update2(void *argument)
{
    (void) argument;
}