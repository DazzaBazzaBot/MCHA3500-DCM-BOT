#include "unity_fixture.h"

#include <string.h>
#include "cmsis_os2.h"
#include "dimmer_task.h"

static osThreadId_t _dimmer_task_id;

TEST_GROUP(RTOSDimmer);

TEST_SETUP(RTOSDimmer)
{
    mockThreadTeardownAll(); //Has to act as an 'init' function otherwise the array of mocks is left with undefined values on startup.
    dimmer_task_init();
    _dimmer_task_id = spyGetThreadId("dimmer");
    //Test for correct task id so all tests fail before executing functions with an invalid pointer
    TEST_ASSERT_NOT_NULL_MESSAGE(_dimmer_task_id, "Expected thread to be started using osThreadNew() and thread name attr to be 'dimmer'");
}

TEST_TEAR_DOWN(RTOSDimmer)
{
    dimmer_task_deinit();
    mockThreadTeardownAll();    //Alex 16/05/19: this was commented out previously, couldn't remember why. un-commented it
}

/*IGNORE_TEST(RTOSDimmer, CorrectCallbackFunc)
{
    // _dimmer_task_update is local :(
}*/

TEST(RTOSDimmer, PriorityNormal)
{
    TEST_ASSERT_MESSAGE(osPriorityLow == osThreadGetPriority(_dimmer_task_id), "Expected thread priority to be set to osPriorityLow");
}

TEST(RTOSDimmer, Resume)
{
    dimmer_task_stop(); // These are to ensure is_running flag is set correctly
    osThreadSuspend(_dimmer_task_id);
    dimmer_task_resume();
    TEST_ASSERT_MESSAGE(osThreadRunning == osThreadGetState(_dimmer_task_id), "Expected thread to be resumed");
}

TEST(RTOSDimmer, Stop)
{
    dimmer_task_resume();
    osThreadResume(_dimmer_task_id);
    dimmer_task_stop();
    TEST_ASSERT_MESSAGE(osThreadInactive == osThreadGetState(_dimmer_task_id), "Expected thread to be suspended");
}

TEST(RTOSDimmer, IsRunningWhenRunning)
{
    TEST_ASSERT_TRUE_MESSAGE(dimmer_task_is_running(), "Expected thread to be running when started");
    dimmer_task_stop();
    osThreadSuspend(_dimmer_task_id);
    dimmer_task_resume();
    TEST_ASSERT_TRUE_MESSAGE(dimmer_task_is_running(), "Expected thread to be running when resumed");
}

TEST(RTOSDimmer, IsStoppedWhenStopped)
{
    dimmer_task_resume();
    osThreadResume(_dimmer_task_id);
    dimmer_task_stop();
    TEST_ASSERT_FALSE_MESSAGE(dimmer_task_is_running(), "Expected thread to be running when resumed");
}

TEST_GROUP_RUNNER(RTOSDimmer)
{
    RUN_TEST_CASE(RTOSDimmer, PriorityNormal);
    RUN_TEST_CASE(RTOSDimmer, Resume);
    RUN_TEST_CASE(RTOSDimmer, Stop);
    RUN_TEST_CASE(RTOSDimmer, IsRunningWhenRunning);
    RUN_TEST_CASE(RTOSDimmer, IsStoppedWhenStopped);
}