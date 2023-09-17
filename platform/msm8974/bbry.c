#include <debug.h>
#include <bits.h>
#include <pm8x41.h>
#include <pm8x41_led.h>
#include <kernel/thread.h>

static thread_t *blink_thread = NULL;
static int curr_blink_code = 0;
static int is_blinking_code = 0;

void bbry_show_led_color(int color)
{
  int green = color << 30 >> 31;
  int blue = color << 29 >> 31;
  int red = (color & 1) ? 50 : 0;

  pm8x41_led_set_color(red, green, blue);

  pm8x41_led_enable(1);
  thread_sleep(1000);
  pm8x41_led_enable(0);

  thread_sleep(250);
}

static int bbry_blink(int *p_blink_code)
{
	if (!p_blink_code)
	{
		dprintf(CRITICAL, "Unable to show NULL blink code");
		return -1;
	}

	int blink_code = *p_blink_code;

	pm8x41_led_init();

	while (1)
	{
		bbry_show_led_color(7);
		bbry_show_led_color(4);
		if (blink_code)
		{
			int c = blink_code;
			do
			{
				bbry_show_led_color(c & 0xF);
				c >>= 4;
			}
			while (c);
		}
		thread_sleep(3000);
	}
}

static void create_blink_thread()
{
	if (!blink_thread)
	{
		blink_thread = thread_create("BLINK", bbry_blink, &curr_blink_code, 31, 0x2000);
		if (!blink_thread)
			dprintf(CRITICAL, "Unable to create thread for blink codes\n");
	}
}

void bbry_blink_code(int code)
{
	if (!blink_thread)
	{
		dprintf(CRITICAL, "Blink thread not created. Unable to show blink code\n");
		return;
	}

	if (is_blinking_code)
	{
		dprintf(CRITICAL, "blinking code 0x%x. Ignoring 0x%x\n", curr_blink_code, code);
		return;
	}

	curr_blink_code = code;
	thread_resume(blink_thread);
	is_blinking_code = 1;
}

void bbry_platform_init(void)
{
	create_blink_thread();
}
