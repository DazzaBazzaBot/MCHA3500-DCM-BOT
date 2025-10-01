#include "unity_fixture.h"
#include "light.h"
#include <stdint.h>

TEST_GROUP(Light);

TEST_SETUP(Light)
{
    light_init();
}

TEST_TEAR_DOWN(Light)
{
}

TEST(Light, DefaultBrightness)
{
    TEST_ASSERT_EQUAL_UINT16(LIGHT_MAX_BRIGHTNESS/2, light_get_brightness());
}


TEST(Light, SetBrightness)
{
    light_set_brightness(42);
    TEST_ASSERT_EQUAL_UINT16(42, light_get_brightness());
}

TEST(Light, MinimumBrightness)
{
    light_set_brightness(0);
    TEST_ASSERT_EQUAL_UINT16(LIGHT_MIN_BRIGHTNESS, light_get_brightness());
}

TEST(Light, MaximumBrightness)
{
    light_set_brightness(65535);
    TEST_ASSERT_EQUAL_UINT16(LIGHT_MAX_BRIGHTNESS, light_get_brightness());
}

TEST(Light, DefaultSaturation)
{
    TEST_ASSERT_EQUAL_UINT16(LIGHT_MAX_SATURATION, light_get_saturation());
}

TEST(Light, SetSaturation)
{
    light_set_saturation(42);
    TEST_ASSERT_EQUAL_UINT16(42, light_get_saturation());
}

TEST(Light, MinimumSaturation)
{
    light_set_saturation(0);
    TEST_ASSERT_EQUAL_UINT16(LIGHT_MIN_SATURATION, light_get_saturation());
}

TEST(Light, MaximumSaturation)
{
    light_set_saturation(65535);
    TEST_ASSERT_EQUAL_UINT16(LIGHT_MAX_SATURATION, light_get_saturation());
}

TEST(Light, SanityCheckHueHueHue)
{
    TEST_ASSERT_MESSAGE(LIGHT_HUE_DEGREE % 4 == 0,
        "Expected LIGHT_HUE_DEGREE to be divisible by 4 so that we can exactly represent encoder angle with an integer variable");

    TEST_ASSERT_MESSAGE(360*LIGHT_HUE_DEGREE < UINT16_MAX,
        "Expected 360*LIGHT_HUE_DEGREE - 1 to be storable in uint16_t");
}

TEST(Light, DefaultHue)
{
    TEST_ASSERT_EQUAL_UINT16(LIGHT_DEFAULT_HUE, light_get_hue());
}

TEST(Light, SetHue)
{
    light_set_hue(42*LIGHT_HUE_DEGREE);
    TEST_ASSERT_EQUAL_UINT16(42*LIGHT_HUE_DEGREE, light_get_hue());
}

TEST(Light, IncrementHue)
{
    light_set_hue(35*LIGHT_HUE_DEGREE);

    light_incr_hue(5*LIGHT_HUE_DEGREE);
    light_incr_hue(5*LIGHT_HUE_DEGREE);
    TEST_ASSERT_EQUAL_UINT16(45*LIGHT_HUE_DEGREE, light_get_hue());

    light_incr_hue(-2*LIGHT_HUE_DEGREE);
    light_incr_hue(-2*LIGHT_HUE_DEGREE);
    TEST_ASSERT_EQUAL_UINT16(41*LIGHT_HUE_DEGREE, light_get_hue());
}

TEST(Light, IncrementHueOverflow)
{
    light_set_hue(350*LIGHT_HUE_DEGREE);
    light_incr_hue(20*LIGHT_HUE_DEGREE);

    TEST_ASSERT_EQUAL_UINT16(10*LIGHT_HUE_DEGREE, light_get_hue());
}

TEST(Light, IncrementHueUnderflow)
{
    light_set_hue(10*LIGHT_HUE_DEGREE);
    light_incr_hue(-20*LIGHT_HUE_DEGREE);

    TEST_ASSERT_EQUAL_UINT16(350*LIGHT_HUE_DEGREE, light_get_hue());
}

TEST(Light, ZeroBrightnessBlack)
{
    light_set_hue(71*LIGHT_HUE_DEGREE); // Arbitrary
    light_set_saturation(LIGHT_MAX_SATURATION);
    light_set_brightness(0);

    TEST_ASSERT_EQUAL_UINT16(0, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_blue());
}

TEST(Light, ZeroSaturationGrey)
{
    light_set_hue(12*LIGHT_HUE_DEGREE); // Arbitrary
    light_set_saturation(0);
    light_set_brightness(420);

    TEST_ASSERT_EQUAL_UINT16(420, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(420, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(420, light_get_blue());
}

TEST(Light, BrightWhite)
{
    light_set_hue(89*LIGHT_HUE_DEGREE); // Arbitrary
    light_set_saturation(0);
    light_set_brightness(LIGHT_MAX_BRIGHTNESS);

    TEST_ASSERT_EQUAL_UINT16(65535, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_blue());
}

TEST(Light, BrightRed)
{
    light_set_hue(0*LIGHT_HUE_DEGREE);
    light_set_saturation(LIGHT_MAX_SATURATION);
    light_set_brightness(LIGHT_MAX_BRIGHTNESS);

    TEST_ASSERT_EQUAL_UINT16(65535, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_blue());
}

TEST(Light, BrightGreen)
{
    light_set_hue(120*LIGHT_HUE_DEGREE);
    light_set_saturation(LIGHT_MAX_SATURATION);
    light_set_brightness(LIGHT_MAX_BRIGHTNESS);

    TEST_ASSERT_EQUAL_UINT16(0, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_blue());
}

TEST(Light, BrightBlue)
{
    light_set_hue(240*LIGHT_HUE_DEGREE);
    light_set_saturation(LIGHT_MAX_SATURATION);
    light_set_brightness(LIGHT_MAX_BRIGHTNESS);

    TEST_ASSERT_EQUAL_UINT16(0, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_blue());
}

TEST(Light, BrightYellow)
{
    light_set_hue(60*LIGHT_HUE_DEGREE);
    light_set_saturation(LIGHT_MAX_SATURATION);
    light_set_brightness(LIGHT_MAX_BRIGHTNESS);

    TEST_ASSERT_EQUAL_UINT16(65535, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_blue());
}

TEST(Light, BrightCyan)
{
    light_set_hue(180*LIGHT_HUE_DEGREE);
    light_set_saturation(LIGHT_MAX_SATURATION);
    light_set_brightness(LIGHT_MAX_BRIGHTNESS);

    TEST_ASSERT_EQUAL_UINT16(0, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_blue());
}

TEST(Light, BrightMagenta)
{
    light_set_hue(300*LIGHT_HUE_DEGREE);
    light_set_saturation(LIGHT_MAX_SATURATION);
    light_set_brightness(LIGHT_MAX_BRIGHTNESS);

    TEST_ASSERT_EQUAL_UINT16(65535, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(65535, light_get_blue());
}

TEST(Light, DarkRed)
{
    light_set_hue(0*LIGHT_HUE_DEGREE);
    light_set_saturation(LIGHT_MAX_SATURATION);
    light_set_brightness(420);

    TEST_ASSERT_EQUAL_UINT16(420, light_get_red());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_green());
    TEST_ASSERT_EQUAL_UINT16(0, light_get_blue());
}

TEST_GROUP_RUNNER(Light)
{
    RUN_TEST_CASE(Light, DefaultBrightness);
    RUN_TEST_CASE(Light, SetBrightness);
    RUN_TEST_CASE(Light, MinimumBrightness);
    RUN_TEST_CASE(Light, MaximumBrightness);
    RUN_TEST_CASE(Light, DefaultSaturation);
    RUN_TEST_CASE(Light, SetSaturation);
    RUN_TEST_CASE(Light, MinimumSaturation);
    RUN_TEST_CASE(Light, MaximumSaturation);
    RUN_TEST_CASE(Light, SanityCheckHueHueHue);
    RUN_TEST_CASE(Light, DefaultHue);
    RUN_TEST_CASE(Light, SetHue);
    RUN_TEST_CASE(Light, IncrementHue);
    RUN_TEST_CASE(Light, IncrementHueOverflow);
    RUN_TEST_CASE(Light, IncrementHueUnderflow);
    RUN_TEST_CASE(Light, ZeroBrightnessBlack);
    RUN_TEST_CASE(Light, ZeroSaturationGrey);
    RUN_TEST_CASE(Light, BrightWhite);
    RUN_TEST_CASE(Light, BrightRed);
    RUN_TEST_CASE(Light, BrightGreen);
    RUN_TEST_CASE(Light, BrightBlue);
    RUN_TEST_CASE(Light, BrightYellow);
    RUN_TEST_CASE(Light, BrightCyan);
    RUN_TEST_CASE(Light, BrightMagenta);
    RUN_TEST_CASE(Light, DarkRed);
}