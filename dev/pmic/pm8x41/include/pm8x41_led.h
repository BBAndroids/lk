#ifndef _PM8x41_LED_H_
#define _PM8x41_LED_H_

#include <sys/types.h>

void pm8x41_led_init();
void pm8x41_led_set_color(uint32_t red, uint32_t green,uint32_t blue);
void pm8x41_led_enable(int enabled);
#endif
