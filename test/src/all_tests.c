#include "unity_fixture.h"

static void RunAllTests(void)
{
    // Lab 2
    RUN_TEST_GROUP(RGB_LED);
    RUN_TEST_GROUP(Potentiometer);

    // Lab 3
    RUN_TEST_GROUP(RTOSHeartbeat);
    RUN_TEST_GROUP(RTOSDimmer);
    RUN_TEST_GROUP(Button);
}

int main(int argc, const char * argv[])
{
    return UnityMain(argc, argv, RunAllTests);
}