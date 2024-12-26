 /* Copyright (c) 2015, The Linux Foundation. All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <qpnp_led.h>
#include <platform/iomap.h>
#include <pm8x41_wled.h>

#define PERPH_SUBTYPE(base)            (base + 0x05)
#define RGB_LED_PERPH_TYPE(base)       (base + 0x04)

#define RGB_LED_SRC_SEL(base)          (base + 0x45)
#define RGB_LED_EN_CTL(base)           (base + 0x46)
#define RGB_LED_ATC_CTL(base)          (base + 0x47)

#define LPG_PERPH_TYPE(base)           (base + 0x04)
#define LPG_PERPH_SUBTYPE(base)        (base + 0x05)
#define LPG_PATTERN_CONFIG(base)       (base + 0x40)
#define LPG_PWM_SIZE_CLK(base)         (base + 0x41)
#define LPG_PWM_FREQ_PREDIV_CLK(base)  (base + 0x42)
#define LPG_PWM_TYPE_CONFIG(base)      (base + 0x43)
#define PWM_VALUE_LSB(base)            (base + 0x44)
#define PWM_VALUE_MSB(base)            (base + 0x45)
#define LPG_ENABLE_CONTROL(base)       (base + 0x46)
#define PWM_SYNC(base)                 (base + 0x47)

#define RGB_LED_ENABLE_BLUE       0x20
#define RGB_LED_ENABLE_GREEN      0x40
#define RGB_LED_ENABLE_RED        0x80

#define RGB_LED_SOURCE_VPH_PWR    0x01
#define PWM_SIZE_CLK              0x03
#define PWM_FREQ                  0x22
#define RGB_LED_ENABLE_PWM        0xe4

static struct qpnp_led_data led;

static void qpnp_led_config(uint16_t lpg_base, uint8_t value)
{
	pm8x41_wled_reg_write(LPG_PATTERN_CONFIG(lpg_base), 0x00);
	pm8x41_wled_reg_write(LPG_PWM_SIZE_CLK(lpg_base), PWM_SIZE_CLK);
	pm8x41_wled_reg_write(LPG_PWM_FREQ_PREDIV_CLK(lpg_base), PWM_FREQ);
	pm8x41_wled_reg_write(LPG_PWM_TYPE_CONFIG(lpg_base), 0x00);
	pm8x41_wled_reg_write(PWM_VALUE_LSB(lpg_base), value);
	pm8x41_wled_reg_write(PWM_VALUE_MSB(lpg_base), 0x00);
	//pm8x41_wled_reg_write(PWM_SYNC(lpg_base), 0x01);
	pm8x41_wled_reg_write(LPG_ENABLE_CONTROL(lpg_base), RGB_LED_ENABLE_PWM);
}

void qpnp_led_init(uint16_t led_base_addr, uint16_t red_lpg_base_addr, uint16_t green_lpg_base_addr, uint16_t blue_lpg_base_addr)
{
	int rc;

	led.base = led_base_addr;
	led.red_lpg_base = red_lpg_base_addr;
	led.green_lpg_base = green_lpg_base_addr;
	led.blue_lpg_base = blue_lpg_base_addr;

	pm8x41_wled_reg_write(RGB_LED_SRC_SEL(led.base), RGB_LED_SOURCE_VPH_PWR);
}

void qpnp_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
	qpnp_led_config(led.red_lpg_base, red >> 4);
	qpnp_led_config(led.green_lpg_base, green >> 4);
	qpnp_led_config(led.blue_lpg_base, blue >> 4);

	uint8_t reg = 0;
	if (red)
		reg |= RGB_LED_ENABLE_RED;
	if (green)
		reg |= RGB_LED_ENABLE_GREEN;
	if (blue)
		reg |= RGB_LED_ENABLE_BLUE;
	pm8x41_wled_reg_write(RGB_LED_EN_CTL(led.base), reg);
}
