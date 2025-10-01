#include "unity_fixture.h"

#include <stdint.h>
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#define TEST_SIGNAL 0x00FF

osEventFlagsId_t _test_RTOSevent_id;

const osEventFlagsAttr_t RTOSEvent_attr = 
{
    .name = "RTOStest_event"
};

TEST_GROUP(RTOSevents);

TEST_SETUP(RTOSevents)
{
    _test_RTOSevent_id = osEventFlagsNew(&RTOSEvent_attr);
    TEST_ASSERT_NOT_NULL_MESSAGE(_test_RTOSevent_id, "Expected event ID to be returned");
}

TEST_TEAR_DOWN(RTOSevents)
{
    mockEventTeardownAll();
}

TEST(RTOSevents, EventFlagNew)
{
    // _test_RTOSevent_id2 = osEventFlagsNew(&RTOSEvent_attr);
}

TEST(RTOSevents, EventFlagSetGet)
{
    uint32_t testSet = osEventFlagsSet(_test_RTOSevent_id, TEST_SIGNAL);
    TEST_ASSERT_EQUAL_MESSAGE(TEST_SIGNAL, testSet&TEST_SIGNAL, "Expected event to return that flags were set");
    uint32_t testGet = osEventFlagsGet(_test_RTOSevent_id);
    TEST_ASSERT_EQUAL_MESSAGE(testSet, testGet, "Expected event to get the flags which were just set");

}

TEST(RTOSevents, EventFlagClear)
{
    /* First, set the flag and ensure it was set */
    uint32_t testSet = osEventFlagsSet(_test_RTOSevent_id, TEST_SIGNAL);
    TEST_ASSERT_EQUAL_MESSAGE(TEST_SIGNAL, testSet&TEST_SIGNAL, "Expected event to return that flags were set");
    /* Then, clear the flag and ensure it was cleared */
    uint32_t testClear = osEventFlagsClear(_test_RTOSevent_id, 0x000F);
    TEST_ASSERT_EQUAL_MESSAGE(0x00F0, testClear, "Expected event to clear flags specified");
}

TEST(RTOSevents, EventFlagWait)
{
        //if flag isn't set, osErrorTimeout if timeout specified, else osErrorResource
    osEventFlagsWait(_test_RTOSevent_id, TEST_SIGNAL, osFlagsWaitAll, osWaitForever);
        //if flag is set, returns flag values.
        // Tests for Any and All flag options.
}

// TEST TODO: if NULL eventid then functions return osErrorParameter


// TEST(RTOSevents, AttributesSet)
// {
//     mock_eventHandle_t *hEvent;
//     hEvent = mockGetEventFromHandle(_test_RTOSevent_id);

//     TEST_ASSERT_EQUAL_STRING(RTOSEvent_attr.name, hEvent->pcEventName);
//     TEST_ASSERT_EQUAL_STRING(RTOSEvent_attr.name, osEventGetName(_test_RTOSevent_id));
//     TEST_ASSERT_EQUAL_MESSAGE(osEventPeriodic, hEvent->uxAutoReload, "Expected event priority to be set to osPriorityHigh");
//     TEST_ASSERT_EQUAL_MESSAGE(-1, hEvent->xEventPeriodInTicks, "Expected event priority to be set to osPriorityHigh");
// }

// TEST(RTOSevents, CleanUpEventsforTesting)
// {
//     mockEventTeardownAll();
//     mock_eventHandle_t *hEvent;
//     hEvent = mockGetEventFromHandle(_test_RTOSevent_id);
//     TEST_ASSERT_EQUAL_PTR(NULL, hEvent->pxCallbackFunction);
//     TEST_ASSERT_EQUAL_STRING(NULL, osEventGetName(_test_RTOSevent_id));
//     TEST_ASSERT_EQUAL_INT_MESSAGE(osEventInactive, osEventIsRunning(_test_RTOSevent_id), "Expected event to be paused");
// }

TEST(RTOSevents, SetOrClearWithoutCreate)
{
    osEventFlagsId_t _test_RTOSevent_id2;
    #pragma warning ( push )
    #pragma warning ( ignored "-Wuninitialized" ) //These pragmas might work for clang
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wuninitialized" //These pragmas work for GCC -- I need to ask Chris about this. If you get errors, sorry!

    TEST_ASSERT_EQUAL_PTR_MESSAGE(NULL, spyCheckInitialisedEventId(_test_RTOSevent_id2), "Expected non-initialised event id to return NULL pointer");
    TEST_ASSERT_EQUAL_PTR_MESSAGE(_test_RTOSevent_id, spyCheckInitialisedEventId(_test_RTOSevent_id), "Expected initialised event id to return correct pointer");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(osErrorParameter, osEventFlagsSet(_test_RTOSevent_id2, 0x00FF), "Expected non-initialised event id to send error when attempting to set");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(osErrorParameter, osEventFlagsClear(_test_RTOSevent_id2, 0x00FF), "Expected non-initialised event id to send error when attempting to clear");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(osErrorParameter, osEventFlagsGet(_test_RTOSevent_id2), "Expected non-initialised event id to send error when attempting to get");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(osErrorParameter, osEventFlagsWait(_test_RTOSevent_id2, 0x00FF, osFlagsWaitAll, osWaitForever), "Expected non-initialised event id to send error when attempting to wait");
    
    #pragma GCC diagnostic pop
    #pragma warning ( pop )
}

TEST(RTOSevents, MultipleNewEvents)
{
    osEventFlagsId_t _test_RTOSevent_id2;
    _test_RTOSevent_id2 = osEventFlagsNew(&RTOSEvent_attr);
    // mockGetEventFromHandle(_test_RTOSevent_id2);
    TEST_ASSERT_NOT_EQUAL(_test_RTOSevent_id2, _test_RTOSevent_id);
}

// // TEST(RTOSevents, GuardAgainstMultipleofSame)
// // {
// //     osEventFlagsId_t _test_RTOSevent_id2;
// //     _test_RTOSevent_id2 = osEventFlagsNew(_RTOSevent_update2, NULL, &RTOSEvent_attr);
// //     mock_eventHandle_t *hEvent2;
// //     hEvent2 = mockGetEventFromHandle(_test_RTOSevent_id2);
// //     TEST_ASSERT_EQUAL_PTR(_RTOSevent_update2, hEvent2->CallbackFunc);
// // }

TEST(RTOSevents, GetEventFromListUsingName)
{
    osEventFlagsId_t _test_eventID, _target_eventID;
    osEventFlagsAttr_t test_event_attr = {.name = "dimmer"};
    _test_eventID = osEventFlagsNew(&test_event_attr);
    _target_eventID = spyGetEventId("dimmer");
    TEST_ASSERT_NOT_NULL(_target_eventID);
    TEST_ASSERT_EQUAL_PTR(_test_eventID, _target_eventID);
}

TEST(RTOSevents, GetEventFromListUsingNameNullIfWrong)
{
    osEventFlagsId_t _target_eventID;
    osEventFlagsAttr_t test_event_attr = {.name = "dimmer"};
    osEventFlagsNew(&test_event_attr);
    _target_eventID = spyGetEventId("dumber");
    TEST_ASSERT_NULL(_target_eventID);
}

TEST_GROUP_RUNNER(RTOSevents)
{
    RUN_TEST_CASE(RTOSevents, EventFlagNew);
    RUN_TEST_CASE(RTOSevents, EventFlagSetGet);
    RUN_TEST_CASE(RTOSevents, EventFlagClear);
    RUN_TEST_CASE(RTOSevents, EventFlagWait);
    RUN_TEST_CASE(RTOSevents, SetOrClearWithoutCreate);
    // RUN_TEST_CASE(RTOSevents, AttributesSet);
    RUN_TEST_CASE(RTOSevents, MultipleNewEvents);
    // RUN_TEST_CASE(RTOSevents, CleanUpEventsforTesting);
    RUN_TEST_CASE(RTOSevents, GetEventFromListUsingName);
    RUN_TEST_CASE(RTOSevents, GetEventFromListUsingNameNullIfWrong);
}