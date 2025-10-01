#include "unity_fixture.h"
#include "iospy.h"

#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmd_line_buffer.h"
#include "potentiometer.h"

CLB_CREATE_STATIC(clb, 80);

TEST_GROUP(CmdPot);

TEST_SETUP(CmdPot)
{
    clb_init(&clb);
}

TEST_TEAR_DOWN(CmdPot)
{
}

TEST(CmdPot, GetValue)
{
    pot_init();

    uint16_t expected = 321; // Test value should be <= 4095
    mock_adc1.DR = expected;

    char out[80];

    iospy_hook();
    iospy_push_in_str("pot\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Potentiometer ADC value is 321\n",out);
}

TEST(CmdPot, UnexpectedArgument)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("pot 420\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("pot: invalid argument \"420\", syntax is: pot\n", out);    
}

TEST_GROUP_RUNNER(CmdPot)
{
    RUN_TEST_CASE(CmdPot, GetValue);
    RUN_TEST_CASE(CmdPot, UnexpectedArgument);
}