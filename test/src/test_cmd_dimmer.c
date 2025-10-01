#include "unity_fixture.h"
#include "iospy.h"

#include <string.h>
#include "cmd_line_buffer.h"
#include "dimmer_task.h"

CLB_CREATE_STATIC(clb, 80);

TEST_GROUP(CmdDimmer);

TEST_SETUP(CmdDimmer)
{
    clb_init(&clb);
    dimmer_task_init();
}

TEST_TEAR_DOWN(CmdDimmer)
{
}

TEST(CmdDimmer, Start)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("dimmer start\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Dimmer has started\n",out);
}

TEST(CmdDimmer, Stop)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("dimmer stop\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Dimmer has stopped\n",out);
}

TEST(CmdDimmer, InvalidArgument)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("dimmer derp\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("dimmer: invalid argument \"derp\", syntax is: dimmer [start|stop]\n", out);    
}

TEST(CmdDimmer, NoArgumentStart)
{
    char out[80];

    // dimmer_task_start();
    dimmer_task_resume();

    iospy_hook();
    iospy_push_in_str("dimmer\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Dimmer is currently running. Better catch it.\n", out);    
}

TEST(CmdDimmer, NoArgumentStop)
{
    char out[80];

    dimmer_task_stop();

    iospy_hook();
    iospy_push_in_str("dimmer\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Dimmer is not currently running.\n", out);    
}

TEST(CmdDimmer, TrailingWhitespace)
{
    char out[80];

    // dimmer_task_start();
    dimmer_task_resume();

    iospy_hook();
    iospy_push_in_str("dimmer \n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Dimmer is currently running. Better catch it.\n", out);    
}

TEST_GROUP_RUNNER(CmdDimmer)
{
    RUN_TEST_CASE(CmdDimmer, Start);
    RUN_TEST_CASE(CmdDimmer, Stop);
    RUN_TEST_CASE(CmdDimmer, InvalidArgument);
    RUN_TEST_CASE(CmdDimmer, NoArgumentStart);
    RUN_TEST_CASE(CmdDimmer, NoArgumentStop);
    RUN_TEST_CASE(CmdDimmer, TrailingWhitespace);
}