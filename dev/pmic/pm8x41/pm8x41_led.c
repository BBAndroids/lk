#include <bits.h>
#include <reg.h>
#include <pm8x41_hw.h>

static int led_init_done = 0;
static int led_green = 0;
static int led_red = 0;
static int led_blue = 0;
static int led_need_to_call_led_enable = 0;

static void pm8x41_led_update_color(int id)
{
	int re1;
	int re2;
	int re3;
	int re4;
	int color;

	switch (id)
	{
	case 5:
		re1 = 0x1B541;
		re2 = 0x1B542;
		re3 = 0x1B544;
		re4 = 0x1B546;
		color = led_blue;
		break;
	case 6:
		re1 = 0x1B641;
		re2 = 0x1B642;
		re3 = 0x1B644;
		re4 = 0x1B646;
		color = led_green;
		break;
	case 7:
		re1 = 0x1B741;
		re2 = 0x1B742;
		re3 = 0x1B744;
		re4 = 0x1B746;
		color = led_red;
		break;
	default:
		return;
	}

	pm8x41_reg_write(re1, 3);
	pm8x41_reg_write(re2, 0x22);
	pm8x41_reg_write(re3, color);
	pm8x41_reg_write(re4, 0xE4);
}

void pm8x41_led_init()
{
	if (!led_init_done)
	{
		pm8x41_led_update_color(7);
		pm8x41_led_update_color(6);
		pm8x41_led_update_color(5);
		pm8x41_reg_write(0x1D045, 1);
		led_init_done = 1;
	}
}

void pm8x41_led_set_color(uint32_t red, uint32_t green, uint32_t blue)
{
	led_red = red >> 4;
	if (led_red)
		pm8x41_led_update_color(7);

	led_green = green >> 4;
	if (green >> 4)
		pm8x41_led_update_color(6);

	led_blue = blue >> 4;
	if (blue >> 4)
		pm8x41_led_update_color(5);

	if (led_need_to_call_led_enable)
		pm8x41_led_enable(1);
}

void pm8x41_led_enable(int enabled)
{
	led_need_to_call_led_enable = enabled;

	char value = 0;
	if (enabled)
	{
		char red_flag = led_red ? 0x80 : 0;
		char green_flag = led_green ? 0x40 : 0;
		char blue_flag = led_blue ? 0x20 : 0;
		value = red_flag | green_flag | blue_flag;
	}
	pm8x41_reg_write(0x1D046, value);
}
