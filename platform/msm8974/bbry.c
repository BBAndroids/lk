#include <debug.h>
#include <bits.h>
#include <pm8x41.h>
#include <kernel/thread.h>
#include <smem.h>
#include <string.h>
#include <pm8x41.h>
#include <dev/udc.h>
#include <platform/gpio.h>
#include <qpnp_led.h>

#if PON_VIB_SUPPORT
#include <vibrator.h>
#define VIBRATE_TIME 750
#endif

static thread_t *blink_thread = NULL;
static int curr_blink_code = 0;
static int is_blinking_code = 0;

void bbry_show_led_color(int color)
{
  int green = color << 30 >> 31;
  int blue = color << 29 >> 31;
  int red = (color & 1) ? 50 : 0;

  qpnp_led_set(red, green, blue);
  thread_sleep(1000);
  qpnp_led_set(0, 0, 0);

  thread_sleep(250);
}

void shutdown_device();
static int bbry_blink(void *p_blink_code)
{
	if (!p_blink_code)
	{
		dprintf(CRITICAL, "Unable to show NULL blink code");
		return -1;
	}

	int blink_code = *(int *)p_blink_code;

	for (int i = 0; (i < 5) || udc_is_online(); i++)
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

#if PON_VIB_SUPPORT
	vib_timed_turn_on(VIBRATE_TIME);
#endif

	qpnp_led_set(0x80, 0, 0);

	thread_sleep(1000);

	shutdown_device();
	
	return 0;
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
	qpnp_led_init(QPNP_LED_CTRL_BASE, QPNP_RED_LPG_CTRL_BASE, QPNP_GREEN_LPG_CTRL_BASE, QPNP_BLUE_LPG_CTRL_BASE);

	create_blink_thread();
}

static const void *bbry_hwi = NULL;
static int bbry_rev = 0;
static int bbry_hwid = 0;
static const char *bbry_product = NULL;
static const char *bbry_variant = NULL;

struct hwi_header
{
	uint32_t len;
	uint32_t max_len;
	uint16_t unk;
	uint16_t entry_count;
};

struct hwi_entry_header
{
	uint16_t value_len;
	uint16_t key_len; // -1, because it's buggy.
};

const char *bbry_hwi_get_entry(const char *name, uint16_t *p_len)
{
	const struct hwi_header *hh = bbry_hwi;

	const void *hh_ptr = bbry_hwi + sizeof(struct hwi_header);

	for (int i = 0; i < hh->entry_count; i++)
	{
		const struct hwi_entry_header *heh = hh_ptr;

		const char *key = hh_ptr + sizeof(struct hwi_entry_header) + heh->value_len + 1;
		const char *value = hh_ptr + sizeof(struct hwi_entry_header);

		if (strcmp(key, name) == 0)
		{
			if (p_len)
				*p_len = heh->value_len;

			return value;
		}

		hh_ptr += sizeof(struct hwi_entry_header) + heh->value_len + heh->key_len;
	}

	return 0;
}

static int bbry_calc_hwid(const char *product, const char *variant)
{
	if (!variant || !product)
		return 0;

	uint32_t product_sum = 0xFC95D400;
	if (*product)
	{
		uint32_t product_checksum = 0x811C9DC5;
		uint8_t c_product = (uint8_t)*product;
		do
		{
			product_checksum = c_product ^ (0x1000193 * product_checksum);
			c_product = (uint8_t) *++product;
		} while (c_product);
		product_sum = (((product_checksum ^ (product_checksum >> 20)) & 0xFFFFF) << 8) | 0xF0000000;
	}

	uint8_t variant_sum = 0x58;
	if (*variant)
	{
		uint32_t variant_checksum = 0x9DC5;
		uint8_t c_variant = (uint8_t) *variant;
		do
		{
			variant_checksum = c_variant ^ (0x193 * variant_checksum);
			c_variant = (uint8_t) *++variant;
		} while (c_variant);
		variant_sum = (variant_checksum ^ (variant_checksum >> 8));
	}

	return variant_sum | product_sum;
}

static int bbry_str2num(const char *str, int *num)
{
	if (!num || !str)
		return -1;

	if (str[0] != '0' || str[1] != 'x')
	{
		int sum = 0;
		uint8_t c_ten = *str;
		while ((uint8_t)(c_ten - '0') <= 9u)
		{
			sum = c_ten + 10 * sum - '0';

			c_ten = *++str;
			if (!c_ten)
			{
				*num = sum;
				return 0;
			}
		}
		return -1;
	}
	else
	{
		const uint8_t *hex_ptr = (uint8_t *) str + 2;
		uint8_t c_hex = *hex_ptr;
		int sum = 0;
		do
		{
			if ((uint8_t)(c_hex - '0') > 9u)
			{
				if ((uint8_t)(c_hex - 'a') > 5u)
				{
					if ((uint8_t)(c_hex - 'A') > 5u)
						return -1;

					sum = (c_hex - '7') | (0x10 * sum);
				}
				else
				{
					sum = (c_hex - 'W') | (0x10 * sum);
				}
			}
			else
			{
				sum = (c_hex - '0') | (0x10 * sum);
			}
			c_hex = *++hex_ptr;
		} while (c_hex);

		*num = sum;
		return 0;
	}
}

struct bbry_remap_table
{
	int old;
	int new;
};

static struct bbry_remap_table wolverine_na_table[] =
{
	{6, 3},
	{7, 4},
	{8, 4},
	{9, 5}
};

static struct bbry_remap_table wolverine_vzw_table[] =
{
	{2, 1},
	{7, 1},
	{3, 2},
	{4, 3},
	{5, 3},
	{6, 3},
	{8, 4},
	{9, 4},
	{10, 4}
};

static struct bbry_remap_table wolverine_emea_table[] =
{
	{3, 2},
	{4, 3},
	{5, 3},
	{6, 3},
	{7, 4},
	{8, 4}
};

static int bbry_wolverine_rev_remap(int rev)
{
	struct bbry_remap_table *table = NULL;
	int table_len = 0;
	if (strcmp(bbry_variant, "na") == 0) {
		table = &wolverine_na_table;
		table_len = ARRAY_SIZE(wolverine_na_table);
	} else if (strcmp(bbry_variant, "vzw") == 0) {
		table = &wolverine_vzw_table;
		table_len = ARRAY_SIZE(wolverine_vzw_table);
	} else if (strcmp(bbry_variant, "emea") == 0) {
		table = &wolverine_emea_table;
		table_len = ARRAY_SIZE(wolverine_emea_table);
	} else
		return rev;

	for (int i = 0; i < table_len; i++) {
		if (table[i].old == rev)
		{
			dprintf(ALWAYS, "Remapping board rev from %d to %d\n", rev, table[i].new);
			return table[i].new;
		}
	}

	return rev;
}

static void bbry_load_hwi_from_smem()
{
	uint32_t hwi_len;
	void *hwi = smem_get_alloc_entry(SMEM_ID_VENDOR1, &hwi_len);
	if (!hwi)
	{
		dprintf(CRITICAL, "ERROR: Failed to read SMEM_ID_VENDOR1\n");
		return;
	}

	// TODO: Verify sizes

	bbry_hwi = hwi;

	bbry_product = bbry_hwi_get_entry("product", 0);
	bbry_variant = bbry_hwi_get_entry("variant", 0);

	if (!bbry_product || !bbry_variant)
	{
		dprintf(CRITICAL, "Unable to get product/variant from hwi\n");
		return;
	}

	bbry_hwid = bbry_calc_hwid(bbry_product, bbry_variant);

	const char *pcb_rev = bbry_hwi_get_entry("pcb_rev", 0);
	if (!pcb_rev)
	{
		dprintf(CRITICAL, "Unable to get pcb rev from hwi\n");
		return;
	}

	int pcb_rev_int;
	bbry_str2num(pcb_rev, &pcb_rev_int);

	if (strcmp(bbry_product, "wolverine") == 0)	{
		bbry_rev = bbry_wolverine_rev_remap(pcb_rev_int);
	} else {
		const char *pop_rev = bbry_hwi_get_entry("pop_rev", 0);
		if (!pop_rev)
		{
			dprintf(CRITICAL, "Unable to get pop rev from hwi\n");
			return;
		}

		int pop_rev_int;
		bbry_str2num(pop_rev, &pop_rev_int);

		bbry_rev = pop_rev_int | (pcb_rev_int << 16);
	}
}

int bbry_get_rev()
{
	if (!bbry_rev)
		bbry_load_hwi_from_smem();

	return bbry_rev;
}

int bbry_get_hwid()
{
	if (!bbry_hwid)
		bbry_load_hwi_from_smem();

	return bbry_hwid;
}

const char *bbry_get_product()
{
	if (!bbry_product)
		bbry_load_hwi_from_smem();

	return bbry_product;
}

const char *bbry_get_variant()
{
	if (!bbry_variant)
		bbry_load_hwi_from_smem();

	return bbry_variant;
}

int bbry_get_device_variant()
{
	if (!bbry_rev)
		bbry_load_hwi_from_smem();

	const char *entry = bbry_hwi_get_entry("device_variant", 0);
	if (!entry)
		return -1;

	int num;
	if (bbry_str2num(entry, &num))
		return 0;

	return num;
}

int is_backup_bootchain()
{
	const char *entry = bbry_hwi_get_entry("backup_bootchain", 0);
	if (!entry)
		return 0;

	int num;
	if (bbry_str2num(entry, &num))
		return 0;

	return num;
}

void analogix_usb_passthrough()
{
	// analogix,v10-ctrl
	gpio_tlmm_config(53, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_ENABLE);
	gpio_set(53, 2);

	// analogix,reset
	gpio_tlmm_config(55, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_ENABLE);
	gpio_set(55, 2);

	// analogix,p-dwn
	gpio_tlmm_config(16, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_ENABLE);
	gpio_set(16, 2);
}

void bbry_uart_on_jack(int enable)
{
	if (strcmp(bbry_get_product(), "oslo") == 0)
	{
		if (enable)
		{
			gpio_tlmm_config(45, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_DISABLE);
			gpio_set(45, 0);
		}
		else
			gpio_tlmm_config(45, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_ENABLE);

		enable = 0;
	}

	struct pm8x41_gpio gpio_in;
	gpio_in.direction = PM_GPIO_DIR_IN;
	gpio_in.output_buffer = 0;
	gpio_in.output_value = 0;
	gpio_in.pull = PM_GPIO_PULL_UP_30;
	gpio_in.vin_sel = 3;
	gpio_in.out_strength = 0;
	gpio_in.function = 2;
	gpio_in.inv_int_pol = 0;
	gpio_in.disable_pin = enable;

	struct pm8x41_gpio gpio_out;
	gpio_out.direction = PM_GPIO_DIR_OUT;
	gpio_out.output_buffer = 0;
	gpio_out.output_value = 0;
	gpio_out.pull = PM_GPIO_PULL_RESV_2;
	gpio_out.vin_sel = 3;
	gpio_out.out_strength = 1;
	gpio_out.function = 2;
	gpio_out.inv_int_pol = 0;
	gpio_out.disable_pin = enable;

	pm8x41_gpio_config(25, &gpio_in);
	pm8x41_gpio_config(26, &gpio_out);
	pm8x41_gpio_config(29, &gpio_in);
	pm8x41_gpio_config(30, &gpio_out);
}
