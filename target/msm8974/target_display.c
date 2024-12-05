/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <debug.h>
#include <smem.h>
#include <err.h>
#include <msm_panel.h>
#include <mipi_dsi.h>
#include <pm8x41.h>
#include <pm8x41_wled.h>
#include <board.h>
#include <mdp5.h>
#include <i2c_qup.h>
#include <blsp_qup.h>
#include <platform/gpio.h>
#include <platform/clock.h>
#include <platform/iomap.h>
#include <target/display.h>
#include "include/panel.h"
#include "include/display_resource.h"

static struct msm_fb_panel_data panel;
static uint8_t edp_enable;

#define HFPLL_LDO_ID 12

static struct pm8x41_wled_data wled_ctrl = {
	.mod_scheme      = 0x00,
	.led1_brightness = (0x0F << 8) | 0xEF,
	.led2_brightness = (0x0F << 8) | 0xEF,
	.led3_brightness = (0x0F << 8) | 0xEF,
	.max_duty_cycle  = 0x01,
	.ovp = 0x2,
	.full_current_scale = 0x19,
	.fdbck = 0x01
};

static uint32_t dsi_pll_lock_status(uint32_t ctl_base)
{
	uint32_t counter, status;

	udelay(100);
	mdss_dsi_uniphy_pll_lock_detect_setting(ctl_base);

	status = readl(ctl_base + 0x02c0) & 0x01;
	for (counter = 0; counter < 5 && !status; counter++) {
		udelay(100);
		status = readl(ctl_base + 0x02c0) & 0x01;
	}

	return status;
}

static uint32_t dsi_pll_enable_seq_b(uint32_t ctl_base)
{
	mdss_dsi_uniphy_pll_sw_reset(ctl_base);

	writel(0x01, ctl_base + 0x0220); /* GLB CFG */
	udelay(1);
	writel(0x05, ctl_base + 0x0220); /* GLB CFG */
	udelay(200);
	writel(0x07, ctl_base + 0x0220); /* GLB CFG */
	udelay(500);
	writel(0x0f, ctl_base + 0x0220); /* GLB CFG */
	udelay(500);

	return dsi_pll_lock_status(ctl_base);
}

static uint32_t dsi_pll_enable_seq_d(uint32_t ctl_base)
{
	mdss_dsi_uniphy_pll_sw_reset(ctl_base);

	writel(0x01, ctl_base + 0x0220); /* GLB CFG */
	udelay(1);
	writel(0x05, ctl_base + 0x0220); /* GLB CFG */
	udelay(200);
	writel(0x07, ctl_base + 0x0220); /* GLB CFG */
	udelay(250);
	writel(0x05, ctl_base + 0x0220); /* GLB CFG */
	udelay(200);
	writel(0x07, ctl_base + 0x0220); /* GLB CFG */
	udelay(500);
	writel(0x0f, ctl_base + 0x0220); /* GLB CFG */
	udelay(500);

	return dsi_pll_lock_status(ctl_base);
}

static void dsi_pll_enable_seq(uint32_t ctl_base)
{
	uint32_t counter, status;

	for (counter = 0; counter < 3; counter++) {
		status = dsi_pll_enable_seq_b(ctl_base);
		if (status)
			break;
		status = dsi_pll_enable_seq_d(ctl_base);
		if (status)
			break;
		status = dsi_pll_enable_seq_d(ctl_base);
		if(status)
			break;
	}

	if (!status)
		dprintf(CRITICAL, "Pll lock sequence failed\n");
}

static int msm8974_wled_backlight_ctrl(uint8_t enable)
{
	uint32_t platform_id = board_platform_id();
	uint32_t hardware_id = board_hardware_id();
	uint8_t slave_id = 1;

	if (enable) {
		if (platform_id == MSM8974AC)
			if ((hardware_id == HW_PLATFORM_MTP)
			    || (hardware_id == HW_PLATFORM_LIQUID))
				slave_id = 3;

		pm8x41_wled_config_slave_id(slave_id);
		pm8x41_wled_config(&wled_ctrl);
		pm8x41_wled_sink_control(enable);
		pm8x41_wled_iled_sync_control(enable);
		pm8x41_wled_led_mod_enable(enable);
	}
	pm8x41_wled_enable(enable);

	return NO_ERROR;
}

static int msm8974_pwm_backlight_ctrl(int gpio_num, int lpg_chan, int enable)
{
	struct pm8x41_gpio gpio_param = {
		.direction = PM_GPIO_DIR_OUT,
		.function = PM_GPIO_FUNC_2,
		.vin_sel = 2,   /* VIN_2 */
		.pull = PM_GPIO_PULL_UP_1_5 | PM_GPIO_PULLDOWN_10,
		.output_buffer = PM_GPIO_OUT_CMOS,
		.out_strength = PM_GPIO_OUT_DRIVE_HIGH,
	};

	dprintf(SPEW, "%s: gpio=%d lpg=%d enable=%d\n", __func__,
				gpio_num, lpg_chan, enable);

	if (enable) {
		pm8x41_gpio_config(gpio_num, &gpio_param);
		pm8x41_lpg_write(lpg_chan, 0x41, 0x33); /* LPG_PWM_SIZE_CLK, */
		pm8x41_lpg_write(lpg_chan, 0x42, 0x01); /* LPG_PWM_FREQ_PREDIV */
		pm8x41_lpg_write(lpg_chan, 0x43, 0x20); /* LPG_PWM_TYPE_CONFIG */
		pm8x41_lpg_write(lpg_chan, 0x44, 0xb2); /* LPG_VALUE_LSB */
		pm8x41_lpg_write(lpg_chan, 0x45, 0x01);  /* LPG_VALUE_MSB */
		pm8x41_lpg_write(lpg_chan, 0x46, 0xe4); /* LPG_ENABLE_CONTROL */
	} else {
		pm8x41_lpg_write(lpg_chan, 0x46, 0x00);
	}

	return NO_ERROR;
}

static int stled110_write_byte(struct qup_i2c_dev *dev, uint8_t reg, uint8_t value)
{
	uint8_t send_buf[2] = { reg, value };
	struct i2c_msg msg =
	{
		.len = sizeof(send_buf),
		.buf = send_buf,
		.addr = 0x31,
		.flags = I2C_M_WR
	};

	return qup_i2c_xfer(dev, &msg, 1) != 1;
}

static uint8_t stled110_read_byte(struct qup_i2c_dev *dev, uint8_t reg)
{
	uint8_t send_buf[1] = { reg };
	uint8_t recv_buf[1] = { 0 };
	struct i2c_msg msg[] =
	{
		{
			.len = sizeof(send_buf),
			.buf = send_buf,
			.addr = 0x31,
			.flags = I2C_M_WR
		},
		{
			.len = sizeof(recv_buf),
			.buf = recv_buf,
			.addr = 0x31,
			.flags = I2C_M_RD
		}
	};

	qup_i2c_xfer(dev, msg, 2);

	return *recv_buf;
}

static int stled110_power_on(struct qup_i2c_dev *dev)
{
	if (stled110_write_byte(dev, 0x02, 0x7F))
	{
		dprintf(CRITICAL, "Failed to i2c write reg 0x02\n");
		return ERR_IO;
	}

	if (stled110_write_byte(dev, 0x03, 0x80))
	{
		dprintf(CRITICAL, "Failed to i2c write reg 0x03\n");
		return ERR_IO;
	}

	if (stled110_write_byte(dev, 0x08, 0x00))
	{
		dprintf(CRITICAL, "Failed to i2c write reg 0x08\n");
		return ERR_IO;
	}

	if (stled110_write_byte(dev, 0x1b, 0x01))
	{
		dprintf(CRITICAL, "Failed to i2c write reg 0x1b\n");
		return ERR_IO;
	}

	// Enable cabc
	if (stled110_write_byte(dev, 0x01, 0x31))
	{
		dprintf(CRITICAL, "Failed to i2c write reg 0x01\n");
		return ERR_IO;
	}

	if (stled110_write_byte(dev, 0x17, 0x10))
	{
		dprintf(CRITICAL, "Failed to i2c write reg 0x17\n");
		return ERR_IO;
	}

	return NO_ERROR;
}

static int stled110_power_off(struct qup_i2c_dev *dev)
{
	uint8_t R01 = stled110_read_byte(dev, 0x01);
	if (R01 < 0)
	{
		dprintf(CRITICAL, "Failed to read register R01, use powered-on default (0x2F/0x27)\n");
		R01 = 0x31;
	}

	R01 &= ~0x20;
	if (stled110_write_byte(dev, 0x01, R01))
	{
		dprintf(CRITICAL, "Failed to i2c write reg 0x01\n");
		return ERR_IO;
	}

	if ((R01 & 4) != 0) // Charge pump
	{
		R01 &= ~4;
		if (stled110_write_byte(dev, 0x01, R01))
		{
			dprintf(CRITICAL, "Failed to i2c write reg 0x01\n");
			return ERR_IO;
		}
	}

	udelay(1000);

	if ((R01 & 2) != 0)
	{
		R01 &= ~2;
		if (stled110_write_byte(dev, 0x01, R01))
		{
			dprintf(CRITICAL, "Failed to i2c write reg 0x01\n");
			return ERR_IO;
		}
	}

	if ((R01 & 1) != 0)
	{
		udelay(100000);
		R01 &= ~1;
		if (stled110_write_byte(dev, 0x01, R01))
		{
			dprintf(CRITICAL, "Failed to i2c write reg 0x01\n");
			return ERR_IO;
		}
	}

	return NO_ERROR;
}

static int msm8974_stled110_backlight_ctrl(uint8_t enable, int version)
{
	struct qup_i2c_dev *dev;
	int ldo_en = 0;
	int bl_en = 9;

	if (strcmp(bbry_get_variant(), "wichita") == 0) {
		ldo_en = 94;
		bl_en = 109;
	}

	dprintf(CRITICAL, "Samanta version: %d\n", version);

	dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_2, 100000, 19200000);
	if (!dev)
	{
		dprintf(CRITICAL, "Failed initializing I2c\n");
		return ERR_IO;
	}

	if (enable)
	{
		gpio_tlmm_config(ldo_en, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_DISABLE); // ldo en
		gpio_tlmm_config(bl_en, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_DISABLE);	// bl en

		gpio_set(ldo_en, 0); // ldo en
		gpio_set(bl_en, 0);	 // bl en

		udelay(1000);

		gpio_set(ldo_en, 2); // ldo en

		udelay(1000);

		if (version == 1)
		{
			uint8_t cmd_buf[] = {0x40, 0xAA};
			struct i2c_msg cmd = {
				.len = sizeof(cmd_buf),
				.buf = cmd_buf,
				.addr = 0x31,
				.flags = I2C_M_WR | I2C_M_IGNORE_NAK
			};

			qup_i2c_xfer(dev, &cmd, 1);

			gpio_set(bl_en, 2); // bl en

			udelay(10000);

			if (stled110_write_byte(dev, 0x47, 1)) {
				dprintf(CRITICAL, "Failed to i2c write reg 0x47\n");
				return ERR_IO;
			}

			if (stled110_write_byte(dev, 0x48, 0xFF)) {
				dprintf(CRITICAL, "Failed to i2c write reg 0x48\n");
				return ERR_IO;
			}

			if (stled110_write_byte(dev, 0x33, 0x12)) {
				dprintf(CRITICAL, "Failed to i2c write reg 0x33\n");
				return ERR_IO;
			}
		}
		else
		{
			uint8_t cmd_buf[] = {0x48, 0x1F};
			struct i2c_msg cmd =
			{
				.len = sizeof(cmd_buf),
				.buf = cmd_buf,
				.addr = 0x31,
				.flags = I2C_M_WR | I2C_M_IGNORE_NAK
			};

			qup_i2c_xfer(dev, &cmd, 1);

			gpio_set(bl_en, 2); // bl en

			udelay(10000);
		}

		stled110_power_on(dev);

		for (int retry = 0; retry < 100; retry++)
		{
			uint8_t send_buf[5] = {0x0F, 0x00, 0xFF, 0x0F, 0x01};
			struct i2c_msg msg =
			{
				.len = sizeof(send_buf),
				.buf = send_buf,
				.addr = 0x31,
				.flags = I2C_M_WR
			};

			qup_i2c_xfer(dev, &msg, 1) != 1;

			if (stled110_read_byte(dev, 0x12) == 0)
				break;

			uint8_t err = stled110_read_byte(dev, 0x04);
			if (err != 0)
			{
				dprintf(CRITICAL, "stled110 error detected! #2 %02x\n", err);
				stled110_power_off(dev);
				stled110_power_on(dev);
				continue;
			}

			dprintf(CRITICAL, "Brightness not set, retry...\n");
		}
	}
	else
	{
		stled110_power_off(dev);

		gpio_set(bl_en, 0);
		gpio_set(ldo_en, 0);
	}

	return NO_ERROR;
}

int target_backlight_ctrl(struct backlight *bl, uint8_t enable)
{
	uint32_t ret = NO_ERROR;

	if (!bl) {
		dprintf(CRITICAL, "backlight structure is not available\n");
		return ERR_INVALID_ARGS;
	}

	switch (bl->bl_interface_type) {
	case BL_WLED:
		ret = msm8974_wled_backlight_ctrl(enable);
		break;
	case BL_PWM:
		ret = msm8974_pwm_backlight_ctrl(pwm_gpio.pin_id,
						PWM_BL_LPG_CHAN_ID,
						enable);
		break;
	case BL_SAMANTA_V1:
		ret = msm8974_stled110_backlight_ctrl(enable, 1);
		break;
	case BL_SAMANTA_V2:
		ret = msm8974_stled110_backlight_ctrl(enable, 2);
		break;
	default:
		dprintf(CRITICAL, "backlight type:%d not supported\n",
						bl->bl_interface_type);
		return ERR_NOT_SUPPORTED;
	}

	return ret;
}

int target_panel_clock(uint8_t enable, struct msm_panel_info *pinfo)
{
	struct mdss_dsi_pll_config *pll_data;
	uint32_t dual_dsi = pinfo->mipi.dual_dsi;
	dprintf(SPEW, "target_panel_clock\n");

	pll_data = pinfo->mipi.dsi_pll_config;
	if (enable) {
		mdp_gdsc_ctrl(enable);
		mdp_clock_init();
		mdss_dsi_auto_pll_config(MIPI_DSI0_BASE, pll_data);
		dsi_pll_enable_seq(MIPI_DSI0_BASE);
		if (panel.panel_info.mipi.dual_dsi &&
				!(panel.panel_info.mipi.broadcast)) {
			mdss_dsi_auto_pll_config(MIPI_DSI1_BASE, pll_data);
			dsi_pll_enable_seq(MIPI_DSI1_BASE);
		}
		mmss_clock_auto_pll_init(DSI0_PHY_PLL_OUT, dual_dsi,
					pll_data->pclk_m,
					pll_data->pclk_n,
					pll_data->pclk_d);
	} else if(!target_cont_splash_screen()) {
		// * Add here for continuous splash  *
		mmss_clock_disable(dual_dsi);
		mdp_clock_disable(dual_dsi);
	}

	return NO_ERROR;
}

/* Pull DISP_RST_N high to get panel out of reset */
int target_panel_reset(uint8_t enable, struct panel_reset_sequence *resetseq,
					struct msm_panel_info *pinfo)
{
		return NO_ERROR;
}

int target_ldo_ctrl(uint8_t enable)
{
	if (enable) {
		if (strcmp(bbry_get_product(), "oslo") == 0)
			gpio_tlmm_config(46, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_ENABLE);

		gpio_tlmm_config(49, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_DISABLE);
		gpio_set(49, 0);
		mdelay(1);
	} else {
		gpio_set(49, 0);
	}

	uint32_t ldocounter = 0;
	uint32_t pm8x41_ldo_base = 0x13F00;

	while (ldocounter < TOTAL_LDO_DEFINED) {
		struct pm8x41_ldo ldo_entry = LDO((pm8x41_ldo_base +
			0x100 * ldo_entry_array[ldocounter].ldo_id),
			ldo_entry_array[ldocounter].ldo_type);

		dprintf(SPEW, "Setting %s\n",
				ldo_entry_array[ldocounter].ldo_name);

		/* Set voltage during power on */
		if (enable) {
			pm8x41_ldo_set_voltage(&ldo_entry,
					ldo_entry_array[ldocounter].ldo_voltage);
			pm8x41_ldo_control(&ldo_entry, enable);
		} else if(ldo_entry_array[ldocounter].ldo_id != HFPLL_LDO_ID) {
			pm8x41_ldo_control(&ldo_entry, enable);
		}
		ldocounter++;
	}

	if (enable) {
		udelay(6);
		gpio_tlmm_config(8, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA, GPIO_DISABLE);
		gpio_set(8, 2);
		udelay(200);
		gpio_set(49, 2);
		mdelay(60);
	} else {
		gpio_set(8, 0);
	}

	return NO_ERROR;
}

static int msm8974_mdss_edp_panel_clock(int enable)
{
	if (enable) {
		mdp_gdsc_ctrl(enable);
		mdp_clock_init();
		edp_clk_enable();
	} else if (!target_cont_splash_screen()) {
		/* Add here for continuous splash */
		edp_clk_disable();
		mdp_clock_disable();
		mdp_gdsc_ctrl(enable);
	}

	return 0;
}

static int msm8974_edp_panel_power(int enable)
{
	struct pm8x41_gpio gpio36_param = {
		.direction = PM_GPIO_DIR_OUT,
		.function = PM_GPIO_FUNC_2,
		.vin_sel = 2,	/* VIN_2 */
		.pull = PM_GPIO_PULL_UP_1_5 | PM_GPIO_PULLDOWN_10,
		.output_buffer = PM_GPIO_OUT_CMOS,
		.out_strength = PM_GPIO_OUT_DRIVE_HIGH,
	};

	struct pm8x41_ldo ldo12 = LDO(PM8x41_LDO12, PLDO_TYPE);

	if (enable) {
		/* Enable backlight */
		dprintf(SPEW, "Enable Backlight\n");
		msm8974_pwm_backlight_ctrl(36, 8, 1);
		dprintf(SPEW, "Enable Backlight Done\n");

		/* Turn on LDO12 for edp vdda */
		dprintf(SPEW, "Setting LDO12 n");
		pm8x41_ldo_set_voltage(&ldo12, 1800000);
		pm8x41_ldo_control(&ldo12, enable);
		dprintf(SPEW, "Setting LDO12 Done\n");

		/* Panel Enable */
		dprintf(SPEW, "Panel Enable\n");
		gpio_tlmm_config(58, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA,
				GPIO_DISABLE);
		gpio_set(58, 2);
		dprintf(SPEW, "Panel Enable Done\n");
	} else {
		/* Keep LDO12 on, otherwise kernel will not boot */
		gpio_set(58, 0);
		msm8974_pwm_backlight_ctrl(36, 8, 0);
	}

	return 0;
}

bool target_display_panel_node(char *panel_name, char *pbuf, uint16_t buf_size)
{
	int prefix_string_len = strlen(DISPLAY_CMDLINE_PREFIX);
	bool ret = true;

	panel_name += strspn(panel_name, " ");

	if (!strcmp(panel_name, HDMI_PANEL_NAME)) {
		if (buf_size < (prefix_string_len + LK_OVERRIDE_PANEL_LEN +
				HDMI_CONTROLLER_STRING)) {
			dprintf(CRITICAL, "command line argument is greater than buffer size\n");
			return false;
		}

		strlcpy(pbuf, DISPLAY_CMDLINE_PREFIX, buf_size);
		buf_size -= prefix_string_len;
		strlcat(pbuf, LK_OVERRIDE_PANEL, buf_size);
		buf_size -= LK_OVERRIDE_PANEL_LEN;
		strlcat(pbuf, HDMI_CONTROLLER_STRING, buf_size);
	} else {
		ret = gcdb_display_cmdline_arg(pbuf, buf_size);
	}

	return ret;
}

void target_display_init(const char *panel_name)
{
	uint32_t hw_id = board_hardware_id();
	uint32_t panel_loop = 0;
	uint32_t ret = 0;

	panel_name += strspn(panel_name, " ");

	if (!strcmp(panel_name, NO_PANEL_CONFIG)) {
		dprintf(INFO, "Skip panel configuration\n");
		return;
	}

	if (!strcmp(panel_name, HDMI_PANEL_NAME)) {
		dprintf(INFO, "%s: HDMI is primary\n", __func__);
		return;
	}

	switch (hw_id) {
	case HW_PLATFORM_LIQUID:
		edp_panel_init(&(panel.panel_info));
		panel.clk_func = msm8974_mdss_edp_panel_clock;
		panel.power_func = msm8974_edp_panel_power;
		panel.fb.base = (void *)EDP_FB_ADDR;
		panel.fb.format = FB_FORMAT_RGB888;
		panel.mdp_rev = MDP_REV_50;

		if (msm_display_init(&panel)) {
			dprintf(CRITICAL, "edp init failed!\n");
			return;
		}

		edp_enable = 1;
		break;
	default:
		do {
			ret = gcdb_display_init(panel_name, MDP_REV_50,
				MIPI_FB_ADDR);
			if (!ret || ret == ERR_NOT_SUPPORTED) {
				break;
			} else {
				target_force_cont_splash_disable(true);
				msm_display_off();
				target_force_cont_splash_disable(false);
			}
		} while (++panel_loop <= oem_panel_max_auto_detect_panels());
		break;
	}
}

void target_display_shutdown(void)
{
	uint32_t hw_id = board_hardware_id();
	switch (hw_id) {
	case HW_PLATFORM_LIQUID:
		if (edp_enable)
			msm_display_off();
		break;
	default:
		gcdb_display_shutdown();
		break;
	}
}
