#include "unity_fixture.h"
#include "iospy.h"

#include <string.h>
#include "cmd_line_buffer.h"
#include "light.h"

CLB_CREATE_STATIC(clb, 80);

TEST_GROUP(CmdLight);

TEST_SETUP(CmdLight)
{
    clb_init(&clb);
    light_init();
}

TEST_TEAR_DOWN(CmdLight)
{
}

TEST(CmdLight, GetHSV)
{
    char out[80];

    light_set_hue(25*LIGHT_HUE_DEGREE);
    light_set_saturation(20971);
    light_set_brightness(27525);

    iospy_hook();
    iospy_push_in_str("light\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Current LED hue = 25deg, sat = 32%, val = 42%\n", out);
}


TEST(CmdLight, TrailingWhitespace)
{
    char out[80];

    light_set_hue(11*LIGHT_HUE_DEGREE);
    light_set_saturation(64224);
    light_set_brightness(13107);

    iospy_hook();
    iospy_push_in_str("light \n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("Current LED hue = 11deg, sat = 98%, val = 20%\n",out);
}

TEST(CmdLight, SetHSV)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("light 73 16 63\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    // TEST_ASSERT_EQUAL_UINT16(73*LIGHT_HUE_DEGREE, light_get_hue());
    // TEST_ASSERT_EQUAL_UINT16(10486, light_get_saturation());
    // TEST_ASSERT_EQUAL_UINT16(41287, light_get_brightness());
    TEST_ASSERT_EQUAL_STRING("Set LED hue = 73deg, sat = 16%, val = 63%\n", out);
}

TEST(CmdLight, SetHSVOverflow)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("light 361 70000 80000\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_UINT16(1*LIGHT_HUE_DEGREE, light_get_hue());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_saturation());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_brightness());
    TEST_ASSERT_EQUAL_STRING("Set LED hue = 1deg, sat = 100%, val = 100%\n", out);
}

TEST(CmdLight, SetHSVUnderflow)
{
    char out[80];

    iospy_hook();
    iospy_push_in_str("light -725 -20 -30\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_UINT16(355*LIGHT_HUE_DEGREE, light_get_hue());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_saturation());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_brightness());
    TEST_ASSERT_EQUAL_STRING("Set LED hue = 355deg, sat = 0%, val = 0%\n", out);
}

TEST(CmdLight, InvalidArgument)
{
    char out[100];

    iospy_hook();
    iospy_push_in_str("light blah\n");
    clb_process(&clb);
    iospy_pop_out_str(out, sizeof(out));
    iospy_unhook();

    TEST_ASSERT_EQUAL_STRING("light: invalid argument \"blah\", syntax is: light [<hue> <saturation> <brightness>]\n",out);
}


TEST_GROUP_RUNNER(CmdLight)
{
    RUN_TEST_CASE(CmdLight, GetHSV);
    RUN_TEST_CASE(CmdLight, TrailingWhitespace);
    RUN_TEST_CASE(CmdLight, SetHSV);
    RUN_TEST_CASE(CmdLight, SetHSVOverflow);
    RUN_TEST_CASE(CmdLight, SetHSVUnderflow);
    RUN_TEST_CASE(CmdLight, InvalidArgument);
}