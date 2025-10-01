#include "unity_fixture.h"

#include <stdint.h>
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

void _testRTOSthread_update(void *argument);
void _testRTOSthread_update2(void *argument);

osThreadId_t _test_RTOSthread_id;

const osThreadAttr_t RTOSTask_attr = 
{
    .name = "RTOStest_thread",
    .priority = osPriorityHigh
};

TEST_GROUP(RTOSthreads);

TEST_SETUP(RTOSthreads)
{
    _test_RTOSthread_id = osThreadNew(_testRTOSthread_update, NULL, &RTOSTask_attr);
}

TEST_TEAR_DOWN(RTOSthreads)
{
    mockThreadTeardownAll();
}

TEST(RTOSthreads, CallbackFuncSet)
{
    mock_taskHandle_t *hTask;
    hTask = mockGetThreadFromHandle(_test_RTOSthread_id);
    TEST_ASSERT_EQUAL_PTR(_testRTOSthread_update, hTask->CallbackFunc);
}

TEST(RTOSthreads, AttributesSet)
{
    mock_taskHandle_t *hTask;
    hTask = mockGetThreadFromHandle(_test_RTOSthread_id);
    TEST_ASSERT_EQUAL_STRING(RTOSTask_attr.name, hTask->pcTaskName);
    TEST_ASSERT_EQUAL_STRING(RTOSTask_attr.name, osThreadGetName(_test_RTOSthread_id));
    TEST_ASSERT_EQUAL_MESSAGE(RTOSTask_attr.priority, hTask->uxCurrentPriority, "Expected thread priority to be set to osPriorityHigh");
    TEST_ASSERT_EQUAL_MESSAGE(RTOSTask_attr.priority, osThreadGetPriority(_test_RTOSthread_id), "Expected thread priority to be set to osPriorityHigh");
}

TEST(RTOSthreads, GetStateAfterNew)
{
    osThreadState_t state = osThreadGetState(_test_RTOSthread_id);
    TEST_ASSERT_EQUAL_INT_MESSAGE(osThreadRunning, state, "Expected thread to be initialised and running");
}

TEST(RTOSthreads, GetStateAfterSuspend)
{
    osThreadSuspend(_test_RTOSthread_id);
    osThreadState_t state = osThreadGetState(_test_RTOSthread_id);
    TEST_ASSERT_EQUAL_INT_MESSAGE(osThreadInactive, state, "Expected thread to be paused");
}

TEST(RTOSthreads, GetStateAfterResumed)
{
    osThreadResume(_test_RTOSthread_id);
    osThreadState_t state = osThreadGetState(_test_RTOSthread_id);
    TEST_ASSERT_EQUAL_INT_MESSAGE(osThreadRunning, state, "Expected thread to be resumed after its paused");
}

TEST(RTOSthreads, CleanUpThreadsforTesting)
{
    mockThreadTeardownAll();
    mock_taskHandle_t *hTask;
    hTask = mockGetThreadFromHandle(_test_RTOSthread_id);
    TEST_ASSERT_EQUAL_PTR(NULL, hTask->CallbackFunc);
    TEST_ASSERT_EQUAL_STRING(NULL, osThreadGetName(_test_RTOSthread_id));
    TEST_ASSERT_EQUAL_MESSAGE(NULL, osThreadGetPriority(_test_RTOSthread_id), "Expected thread priority to be set to osPriorityHigh");
    TEST_ASSERT_EQUAL_INT_MESSAGE(NULL, osThreadGetState(_test_RTOSthread_id), "Expected thread to be paused");
}

IGNORE_TEST(RTOSthreads, ResumeOrSuspendWithoutCreate)
{
    osThreadId_t _test_RTOSthread_id2; //This will generate compiler warnings about being used uninitialised. That's the intent!
    #pragma warning ( push )
    #pragma warning ( ignored "-Wuninitialized" ) //These pragmas might work for clang
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wuninitialized" //These pragmas work for GCC -- I need to ask Chris about this. If you get errors, sorry!
    TEST_ASSERT_EQUAL_INT_MESSAGE(osErrorParameter, osThreadResume(_test_RTOSthread_id2), "Expected non-initialised thread id to send error when attempting to resume");
    TEST_ASSERT_EQUAL_INT_MESSAGE(osErrorParameter, osThreadSuspend(_test_RTOSthread_id2), "Expected non-initialised thread id to send error when attempting to suspend");
    #pragma GCC diagnostic pop
    #pragma warning ( pop )
}

TEST(RTOSthreads, MultipleNewThreads)
{
    osThreadId_t _test_RTOSthread_id2;
    _test_RTOSthread_id2 = osThreadNew(_testRTOSthread_update2, NULL, &RTOSTask_attr);
    mock_taskHandle_t *hTask2;
    hTask2 = mockGetThreadFromHandle(_test_RTOSthread_id2);
    TEST_ASSERT_EQUAL_PTR(_testRTOSthread_update2, hTask2->CallbackFunc);
}

// TEST(RTOSthreads, GuardAgainstMultipleofSame)
// {
//     osThreadId_t _test_RTOSthread_id2;
//     _test_RTOSthread_id2 = osThreadNew(_testRTOSthread_update2, NULL, &RTOSTask_attr);
//     mock_taskHandle_t *hTask2;
//     hTask2 = mockGetThreadFromHandle(_test_RTOSthread_id2);
//     TEST_ASSERT_EQUAL_PTR(_testRTOSthread_update2, hTask2->CallbackFunc);
// }

TEST(RTOSthreads, GetThreadFromListUsingName)
{
    osThreadId_t _test_dimmerID, _target_dimmerID;
    osThreadAttr_t test_dimmer_attr = {.name = "dimmer"};
    _test_dimmerID = osThreadNew(_testRTOSthread_update2, NULL, &test_dimmer_attr);
    _target_dimmerID = spyGetThreadId("dimmer");
    TEST_ASSERT_NOT_NULL(_target_dimmerID);
    TEST_ASSERT_EQUAL_PTR(_test_dimmerID, _target_dimmerID);
}

TEST(RTOSthreads, GetThreadFromListUsingNameNullIfWrong)
{
    osThreadId_t _test_dimmerID, _target_dimmerID;
    osThreadAttr_t test_dimmer_attr = {.name = "dimmer"};
    _test_dimmerID = osThreadNew(_testRTOSthread_update2, NULL, &test_dimmer_attr);
    UNUSED(_test_dimmerID); //Avoid compiler warning. We don't actually want this thread ID to be used.
    _target_dimmerID = spyGetThreadId("dumber");
    TEST_ASSERT_NULL(_target_dimmerID);
}

TEST_GROUP_RUNNER(RTOSthreads)
{
    RUN_TEST_CASE(RTOSthreads, ResumeOrSuspendWithoutCreate);
    RUN_TEST_CASE(RTOSthreads, CallbackFuncSet);
    RUN_TEST_CASE(RTOSthreads, AttributesSet);
    // RUN_TEST_CASE(RTOSthreads, ErrorIfPriorityLessThanIdle);
    RUN_TEST_CASE(RTOSthreads, GetStateAfterNew);
    RUN_TEST_CASE(RTOSthreads, GetStateAfterSuspend);
    RUN_TEST_CASE(RTOSthreads, GetStateAfterResumed);
    RUN_TEST_CASE(RTOSthreads, MultipleNewThreads);
    RUN_TEST_CASE(RTOSthreads, CleanUpThreadsforTesting);
    RUN_TEST_CASE(RTOSthreads, GetThreadFromListUsingName);
    RUN_TEST_CASE(RTOSthreads, GetThreadFromListUsingNameNullIfWrong);
}

void _testRTOSthread_update(void *argument)
{
    UNUSED(argument); //This function left intentionally blank. Just used as a func pointer for above tests.
}
void _testRTOSthread_update2(void *argument)
{
    UNUSED(argument); //This function left intentionally blank. Just used as a func pointer for above tests.
}