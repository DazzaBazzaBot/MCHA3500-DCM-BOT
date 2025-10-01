#include "unity_fixture.h"
#include "iospy.h"

#include <string.h>
#include "cmd_line_buffer.h"
#include "heartbeat_task.h"

CLB_CREATE_STATIC(clb, 80);

TEST_GROUP(CmdHeartbeat);

TEST_SETUP(CmdHeartbeat)
{
    clb_init(&clb);
    heartbeat_task_init();
}

TEST_TEAR_DOWN(CmdHeartbeat)
{
}

TEST(CmdHeartbeat, Start)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("heartbeat start\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Heartbeat has started\n",out);
}

TEST(CmdHeartbeat, Stop)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("heartbeat stop\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Heartbeat has stopped\n",out);
}

TEST(CmdHeartbeat, InvalidArgument)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("heartbeat intensifies\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("heartbeat: invalid argument \"intensifies\", syntax is: heartbeat [start|stop]\n", out);    
}

TEST(CmdHeartbeat, NoArgumentStart)
{
    char out[80];

    heartbeat_task_start();

    iospy_hook();
    iospy_push_in_str("heartbeat\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Heartbeat is currently running\n", out);    
}

TEST(CmdHeartbeat, NoArgumentStop)
{
    char out[80];

    heartbeat_task_stop();

    iospy_hook();
    iospy_push_in_str("heartbeat\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Heartbeat is not currently running\n", out);    
}

TEST(CmdHeartbeat, TrailingWhitespace)
{
    char out[80];

    heartbeat_task_start();

    iospy_hook();
    iospy_push_in_str("heartbeat \n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Heartbeat is currently running\n", out);    
}

TEST_GROUP_RUNNER(CmdHeartbeat)
{
    RUN_TEST_CASE(CmdHeartbeat, Start);
    RUN_TEST_CASE(CmdHeartbeat, Stop);
    RUN_TEST_CASE(CmdHeartbeat, InvalidArgument);
    RUN_TEST_CASE(CmdHeartbeat, NoArgumentStart);
    RUN_TEST_CASE(CmdHeartbeat, NoArgumentStop);
    RUN_TEST_CASE(CmdHeartbeat, TrailingWhitespace);
}