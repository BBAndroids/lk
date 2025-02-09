#ifndef __BBRY_H__
#define __BBRY_H__

void bbry_platform_init(void);
void bbry_show_led_color(int color);
void bbry_blink_code(int code);

char *bbry_hwi_get_entry(const char *name, uint16_t *p_len);

int bbry_get_rev();
int bbry_get_hwid();
const char *bbry_get_product();
const char *bbry_get_variant();
int bbry_get_device_variant();

int is_backup_bootchain();

void analogix_usb_passthrough();
void bbry_uart_on_jack(int enable);

#endif
