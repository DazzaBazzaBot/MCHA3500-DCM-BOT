#include "unity_fixture.h"
#include <stdint.h>
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "heartbeat_task.h"

osTimerId_t _spyHeartbeatTimerID;

TEST_GROUP(RTOSHeartbeat);
TEST_SETUP(RTOSHeartbeat)
{
    heartbeat_task_init();
    _spyHeartbeatTimerID = spyGetTimerId("heartbeat");
    TEST_ASSERT_NOT_NULL_MESSAGE(_spyHeartbeatTimerID, "Expected osTimer to be created and given name 'heartbeat' in function heartbeat_task_init()");
}

TEST_TEAR_DOWN(RTOSHeartbeat)
{
    // heartbeat_task_stop();
    heartbeat_task_deinit();
    mockTimerTeardownAll();
}

TEST(RTOSHeartbeat, HeartbeatStart)
{
    heartbeat_task_deinit();
    osTimerStop(_spyHeartbeatTimerID);
    heartbeat_task_start();
    osMockTimerState_t state = osTimerIsRunning(_spyHeartbeatTimerID);
    TEST_ASSERT_MESSAGE(osTimerRunning == state, "Expected heartbeat_task_start to call osTimerStart() to start timer.");
}

TEST(RTOSHeartbeat, HeartbeatStop)
{
    heartbeat_task_stop();
    osMockTimerState_t state = osTimerIsRunning(_spyHeartbeatTimerID);
    TEST_ASSERT_EQUAL_INT_MESSAGE(osTimerStopped, state, "Expected heartbeat_task_stop to stop timer.");
}

TEST(RTOSHeartbeat, HeartbeatStartonInit)
{
    //heartbeat_task_init calls heartbeat_task_start, starting the timer
    osMockTimerState_t state = osTimerIsRunning(_spyHeartbeatTimerID);
    TEST_ASSERT_EQUAL_INT_MESSAGE(osTimerRunning, state, "Expected heartbeat_task_init to start timer by calling heartbeat_task_start");
}

TEST(RTOSHeartbeat, TimerPeriodAndPeriodic)
{
    mock_timerHandle_t *hTimer;
    hTimer = mockGetTimerFromHandle(_spyHeartbeatTimerID);
    TEST_ASSERT_EQUAL_MESSAGE(osTimerPeriodic, hTimer->uxAutoReload, "Expected timer to be periodic");
    TEST_ASSERT_EQUAL_INT_MESSAGE(10, hTimer->xTimerPeriodInTicks, "Expected timer to have 10mS period");
    // TEST_ASSERT_EQUAL_PTR(_RTOStimer_update, hTimer->pxCallbackFunction); //Callback function is local/private
}

TEST_GROUP_RUNNER(RTOSHeartbeat)
{
    RUN_TEST_CASE(RTOSHeartbeat, HeartbeatStop);
    RUN_TEST_CASE(RTOSHeartbeat, HeartbeatStart);
    RUN_TEST_CASE(RTOSHeartbeat, HeartbeatStartonInit);
    RUN_TEST_CASE(RTOSHeartbeat, TimerPeriodAndPeriodic);
}