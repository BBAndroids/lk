/* Copyright (c) 2012-2014, 2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#include <debug.h>
#include <platform/iomap.h>
#include <platform/irqs.h>
#include <platform/gpio.h>
#include <reg.h>
#include <string.h>
#include <target.h>
#include <platform.h>
#include <dload_util.h>
#include <uart_dm.h>
#include <mmc.h>
#include <spmi.h>
#include <board.h>
#include <smem.h>
#include <baseband.h>
#include <dev/keys.h>
#include <pm8x41.h>
#include <crypto5_wrapper.h>
#include <hsusb.h>
#include <clock.h>
#include <partition_parser.h>
#include <sdhci_msm.h>
#include <qtimer.h>
#include <scm.h>
#include <platform/clock.h>
#include <platform/gpio.h>
#include <stdlib.h>
#include <qpnp_led.h>
#include <pm8x41_hw.h>
#include <pm8x41_adc.h>
#include <platform/bbry.h>

#if PON_VIB_SUPPORT
#include <vibrator.h>
#define VIBRATE_TIME 250
#endif

extern int platform_is_8974();
extern int platform_is_8974ac();
static void set_sdc_power_ctrl();

static uint32_t pmic_ver;

#if MMC_SDHCI_SUPPORT
struct mmc_device *dev;
#endif

#define PMIC_ARB_CHANNEL_NUM    0
#define PMIC_ARB_OWNER_ID       0

#define WDOG_DEBUG_DISABLE_BIT  17

#define CE_INSTANCE             2
#define CE_EE                   1
#define CE_FIFO_SIZE            64
#define CE_READ_PIPE            3
#define CE_WRITE_PIPE           2
#define CE_READ_PIPE_LOCK_GRP   0
#define CE_WRITE_PIPE_LOCK_GRP  0
#define CE_ARRAY_SIZE           20

#ifdef SSD_ENABLE
#define SSD_CE_INSTANCE_1       1
#define SSD_PARTITION_SIZE      8192
#endif

#define FASTBOOT_MODE           0x77665500

#define BOARD_SOC_VERSION1(soc_rev) (soc_rev >= 0x10000 && soc_rev < 0x20000)

#if MMC_SDHCI_SUPPORT
static uint32_t mmc_sdhci_base[] =
	{ MSM_SDC1_SDHCI_BASE, MSM_SDC2_SDHCI_BASE, MSM_SDC3_SDHCI_BASE, MSM_SDC4_SDHCI_BASE };
#endif

static uint32_t mmc_sdc_base[] =
	{ MSM_SDC1_BASE, MSM_SDC2_BASE, MSM_SDC3_BASE, MSM_SDC4_BASE };

static uint32_t mmc_sdc_pwrctl_irq[] =
	{ SDCC1_PWRCTL_IRQ, SDCC2_PWRCTL_IRQ, SDCC3_PWRCTL_IRQ, SDCC4_PWRCTL_IRQ };

void target_early_init(void)
{
#if WITH_DEBUG_UART
	bbry_uart_on_jack(1);

	uart_dm_init(1, 0, BLSP1_UART1_BASE);
#endif
}

uint32_t target_hw_interposer()
{
	return 0;
}

/* Return 1 if vol_up pressed */
int target_volume_up()
{
	static uint8_t first_time = 0;
	uint8_t status = 0;
	struct pm8x41_gpio gpio;

	if (!first_time) {
		/* Configure the GPIO */
		gpio.direction = PM_GPIO_DIR_IN;
		gpio.function  = 0;
		gpio.pull      = PM_GPIO_PULL_UP_30;
		gpio.vin_sel   = 2;

		pm8x41_gpio_config(22, &gpio);

		/* Wait for the pmic gpio config to take effect */
		udelay(10000);

		first_time = 1;
	}

	/* Get status of P_GPIO_5 */
	pm8x41_gpio_get(22, &status);

	return !status; /* active low */
}

/* Return 1 if vol_down pressed */
uint32_t target_volume_down()
{
	static uint8_t first_time = 0;
	uint8_t status = 0;
	struct pm8x41_gpio gpio;

	if (!first_time) {
		/* Configure the GPIO */
		gpio.direction = PM_GPIO_DIR_IN;
		gpio.function  = 0;
		gpio.pull      = PM_GPIO_PULL_UP_30;
		gpio.vin_sel   = 2;

		pm8x41_gpio_config(23, &gpio);

		/* Wait for the pmic gpio config to take effect */
		udelay(10000);

		first_time = 1;
	}

	/* Get status of P_GPIO_5 */
	pm8x41_gpio_get(23, &status);

	return !status; /* active low */
}

static void target_keystatus()
{
	keys_init();

	if(target_volume_down())
		keys_post_event(KEY_VOLUMEDOWN, 1);

	if(target_volume_up())
		keys_post_event(KEY_VOLUMEUP, 1);
}

/* Set up params for h/w CE. */
void target_crypto_init_params()
{
	struct crypto_init_params ce_params;

	/* Set up base addresses and instance. */
	ce_params.crypto_instance  = CE_INSTANCE;
	ce_params.crypto_base      = MSM_CE2_BASE;
	ce_params.bam_base         = MSM_CE2_BAM_BASE;

	/* Set up BAM config. */
	ce_params.bam_ee               = CE_EE;
	ce_params.pipes.read_pipe      = CE_READ_PIPE;
	ce_params.pipes.write_pipe     = CE_WRITE_PIPE;
	ce_params.pipes.read_pipe_grp  = CE_READ_PIPE_LOCK_GRP;
	ce_params.pipes.write_pipe_grp = CE_WRITE_PIPE_LOCK_GRP;

	/* Assign buffer sizes. */
	ce_params.num_ce           = CE_ARRAY_SIZE;
	ce_params.read_fifo_size   = CE_FIFO_SIZE;
	ce_params.write_fifo_size  = CE_FIFO_SIZE;

	/* BAM is initialized by TZ for this platform.
	 * Do not do it again as the initialization address space
	 * is locked.
	 */
	ce_params.do_bam_init      = 0;

	crypto_init_params(&ce_params);
}

crypto_engine_type board_ce_type(void)
{
	return CRYPTO_ENGINE_TYPE_HW;
}

#if MMC_SDHCI_SUPPORT
static void target_mmc_sdhci_init()
{
	struct mmc_config_data config = {0};
	config.bus_width = DATA_BUS_WIDTH_8BIT;

	/* Trying Slot 1*/
	config.slot = 1;
	/*
	 * For 8974 AC platform the software clock
	 * plan recommends to use the following frequencies:
	 * 200 MHz --> 192 MHZ
	 * 400 MHZ --> 384 MHZ
	 * only for emmc slot
	 */
	if (platform_is_8974ac())
		config.max_clk_rate = MMC_CLK_192MHZ;
	else
		config.max_clk_rate = MMC_CLK_200MHZ;
	config.sdhc_base = mmc_sdhci_base[config.slot - 1];
	config.pwrctl_base = mmc_sdc_base[config.slot - 1];
	config.pwr_irq     = mmc_sdc_pwrctl_irq[config.slot - 1];
	config.hs400_support = 1;

	if (!(dev = mmc_init(&config))) {
		/* Trying Slot 2 next */
		config.slot = 2;
		config.max_clk_rate = MMC_CLK_200MHZ;
		config.sdhc_base = mmc_sdhci_base[config.slot - 1];
		config.pwrctl_base = mmc_sdc_base[config.slot - 1];
		config.pwr_irq     = mmc_sdc_pwrctl_irq[config.slot - 1];

		if (!(dev = mmc_init(&config))) {
			dprintf(CRITICAL, "mmc init failed!");
			ASSERT(0);
		}
	}

	/*
	 * MMC initialization is complete, read the partition table info
	 */
	if (partition_read_table()) {
		dprintf(CRITICAL, "Error reading the partition table info\n");
		ASSERT(0);
	}
}

void *target_mmc_device()
{
	return (void *) dev;
}

#else
static void target_mmc_mci_init()
{
	uint32_t base_addr;
	uint8_t slot;

	/* Trying Slot 1 */
	slot = 1;
	base_addr = mmc_sdc_base[slot - 1];

	if (mmc_boot_main(slot, base_addr))
	{
		/* Trying Slot 2 next */
		slot = 2;
		base_addr = mmc_sdc_base[slot - 1];
		if (mmc_boot_main(slot, base_addr)) {
			dprintf(CRITICAL, "mmc init failed!");
			ASSERT(0);
		}
	}
}

/*
 * Function to set the capabilities for the host
 */
void target_mmc_caps(struct mmc_host *host)
{
	host->caps.bus_width = MMC_BOOT_BUS_WIDTH_8_BIT;

	host->caps.ddr_mode = 1;
	host->caps.hs200_mode = 1;
	host->caps.hs_clk_rate = MMC_CLK_96MHZ;
}
#endif


void target_init(void)
{
	dprintf(INFO, "target_init()\n");

	spmi_init(PMIC_ARB_CHANNEL_NUM, PMIC_ARB_OWNER_ID);

	/* Save PM8941 version info. */
	pmic_ver = pm8x41_get_pmic_rev();

	target_keystatus();

#if PON_VIB_SUPPORT
	vib_timed_turn_on(VIBRATE_TIME);
#endif

	if (pm8x41_get_batt_voltage() < 3400000)
	{
		pm8xxx_enable_charging();
		while (pm8x41_get_batt_voltage() < 3400000)
		{
			if (!pm8xxx_is_charger_present())
				shutdown_device();

			qpnp_led_set(0x80, 0, 0);
			thread_sleep(1000);

			qpnp_led_set(0x0, 0x80, 0);
			thread_sleep(1000);

			pm8x41_wd_reset_pet();
		}
		pm8xxx_disable_charging();
		qpnp_led_set(0, 0, 0);
	}

	/*
	 * Set drive strength & pull ctrl for
	 * emmc
	 */
	set_sdc_power_ctrl();

#if MMC_SDHCI_SUPPORT
	target_mmc_sdhci_init();
#else
	target_mmc_mci_init();
#endif
}

unsigned board_machtype(void)
{
	return 0;
}

/* Do any target specific intialization needed before entering fastboot mode */
#ifdef SSD_ENABLE
static void ssd_load_keystore_from_emmc()
{
	uint64_t           ptn    = 0;
	int                index  = -1;
	uint32_t           size   = SSD_PARTITION_SIZE;
	int                ret    = -1;

	uint32_t *buffer = (uint32_t *)memalign(CACHE_LINE,
								   ROUNDUP(SSD_PARTITION_SIZE, CACHE_LINE));

	if (!buffer) {
		dprintf(CRITICAL, "Error Allocating memory for SSD buffer\n");
		ASSERT(0);
	}

	index = partition_get_index("ssd");

	ptn   = partition_get_offset(index);
	if(ptn == 0){
		dprintf(CRITICAL,"ERROR: ssd parition not found");
		return;
	}

	if(mmc_read(ptn, buffer, size)){
		dprintf(CRITICAL,"ERROR:Cannot read data\n");
		return;
	}

	ret = scm_protect_keystore((uint32_t *)&buffer[0],size);
	if(ret != 0)
		dprintf(CRITICAL,"ERROR: scm_protect_keystore Failed\n");

	free(buffer);
}
#endif

void target_fastboot_init(void)
{
	/* Set the BOOT_DONE flag in PM8921 */
	pm8x41_set_boot_done();
	analogix_usb_passthrough();

#ifdef SSD_ENABLE
	clock_ce_enable(SSD_CE_INSTANCE_1);
	ssd_load_keystore_from_emmc();
#endif
}

/* Initialize target specific USB handlers */
target_usb_iface_t* target_usb30_init()
{
	target_usb_iface_t *t_usb_iface;

	t_usb_iface = calloc(1, sizeof(target_usb_iface_t));
	ASSERT(t_usb_iface);

	t_usb_iface->mux_config = target_usb_phy_mux_configure;
	t_usb_iface->clock_init = clock_usb30_init;

	return t_usb_iface;
}

/* Detect the target type */
void target_detect(struct board_data *board)
{
	/* This property is filled in board.c */
}

/* Detect the modem type */
void target_baseband_detect(struct board_data *board)
{
	uint32_t platform;

	platform = board->platform;

	switch(platform) {
	case MSM8974:
	case MSM8274:
	case MSM8674:
	case MSM8274AA:
	case MSM8274AB:
	case MSM8274AC:
	case MSM8674AA:
	case MSM8674AB:
	case MSM8674AC:
	case MSM8974AA:
	case MSM8974AB:
	case MSM8974AC:
		board->baseband = BASEBAND_MSM;
		break;
	case APQ8074:
	case APQ8074AA:
	case APQ8074AB:
	case APQ8074AC:
		board->baseband = BASEBAND_APQ;
		break;
	default:
		dprintf(CRITICAL, "Platform type: %u is not supported\n",platform);
		ASSERT(0);
	};
}

unsigned target_baseband()
{
	return board_baseband();
}

void target_serialno(unsigned char *buf)
{
	unsigned int serialno;
	if (target_is_emmc_boot()) {
		serialno = mmc_get_psn();
		snprintf((char *)buf, 13, "%x", serialno);
	}
}

unsigned check_reboot_mode(void)
{
	uint32_t restart_reason = 0;
	uint32_t soc_ver = 0;
	uint32_t restart_reason_addr;

	soc_ver = board_soc_version();

	if (platform_is_8974() && BOARD_SOC_VERSION1(soc_ver))
		restart_reason_addr = RESTART_REASON_ADDR;
	else
		restart_reason_addr = RESTART_REASON_ADDR_V2;

	/* Read reboot reason and scrub it */
	restart_reason = readl(restart_reason_addr);
	writel(0x00, restart_reason_addr);

	return restart_reason;
}

int set_download_mode(enum reboot_reason mode)
{
	if (mode == NORMAL_DLOAD || mode == EMERGENCY_DLOAD)
		dload_util_write_cookie(mode == NORMAL_DLOAD ?
			DLOAD_MODE_ADDR_V2 : EMERGENCY_DLOAD_MODE_ADDR_V2, mode);

	return 0;
}

void reboot_device(unsigned reboot_reason)
{
	uint32_t soc_ver = 0;
	uint8_t reset_type = 0;

	/* Set cookie for dload mode */
	if(set_download_mode(reboot_reason)) {
		dprintf(CRITICAL, "HALT: set_download_mode not supported\n");
		return;
	}

	soc_ver = board_soc_version();

	/* Write the reboot reason */
	if (platform_is_8974() && BOARD_SOC_VERSION1(soc_ver))
		writel(reboot_reason, RESTART_REASON_ADDR);
	else
		writel(reboot_reason, RESTART_REASON_ADDR_V2);

	if(reboot_reason == FASTBOOT_MODE)
		reset_type = PON_PSHOLD_WARM_RESET;
	else
		reset_type = PON_PSHOLD_HARD_RESET;

	/* Configure PMIC for warm reset */
	if (platform_is_8974() && (pmic_ver == PM8X41_VERSION_V2))
		pm8x41_v2_reset_configure(reset_type);
	else
		pm8x41_reset_configure(reset_type);

	/* Drop PS_HOLD for MSM */
	writel(0x00, MPM2_MPM_PS_HOLD);

	mdelay(5000);

	dprintf(CRITICAL, "Rebooting failed\n");
}

/* Check if MSM needs VBUS mimic for USB */
static int target_needs_vbus_mimic()
{
	if (platform_is_8974())
		return 0;

	return 1;
}

/* Do target specific usb initialization */
void target_usb_init(void)
{
	uint32_t val;

	if (target_needs_vbus_mimic())
	{
		/* Select and enable external configuration with USB PHY */
		ulpi_write(ULPI_MISC_A_VBUSVLDEXTSEL | ULPI_MISC_A_VBUSVLDEXT, ULPI_MISC_A_SET);

		/* Enable sess_vld */
		val = readl(USB_GENCONFIG_2) | GEN2_SESS_VLD_CTRL_EN;
		writel(val, USB_GENCONFIG_2);

		/* Enable external vbus configuration in the LINK */
		val = readl(USB_USBCMD);
		val |= SESS_VLD_CTRL;
		writel(val, USB_USBCMD);
	}
}

uint8_t target_panel_auto_detect_enabled()
{
	return 0;
}

uint8_t target_is_edp()
{
	return 0;
}

static uint8_t splash_override;
/* Returns 1 if target supports continuous splash screen. */
int target_cont_splash_screen()
{
	uint8_t splash_screen = 0;
	if(!splash_override) {
		splash_screen = 1;
	}
	return splash_screen;
}

void target_force_cont_splash_disable(uint8_t override)
{
	splash_override = override;
}

unsigned target_pause_for_battery_charge(void)
{
	uint8_t pon_reason = pm8x41_get_pon_reason();

	if ((pon_reason == USB_CHG) || (pon_reason == DC_CHG))
		return 1;

	return 0;
}

void target_uninit(void)
{
#if MMC_SDHCI_SUPPORT
	mmc_put_card_to_sleep(dev);
#else
	mmc_put_card_to_sleep();
#endif
#ifdef SSD_ENABLE
	clock_ce_disable(SSD_CE_INSTANCE_1);
#endif
	if (crypto_initialized())
		crypto_eng_cleanup();

	/* Disable HC mode before jumping to kernel */
	sdhci_mode_disable(&dev->host);
}

void shutdown_device()
{
	dprintf(CRITICAL, "Going down for shutdown.\n");

	/* Configure PMIC for shutdown. */
	if (platform_is_8974() && (pmic_ver == PM8X41_VERSION_V2))
		pm8x41_v2_reset_configure(PON_PSHOLD_SHUTDOWN);
	else
		pm8x41_reset_configure(PON_PSHOLD_SHUTDOWN);

	/* Drop PS_HOLD for MSM */
	writel(0x00, MPM2_MPM_PS_HOLD);

	mdelay(5000);

	dprintf(CRITICAL, "Shutdown failed\n");
}

static void set_sdc_power_ctrl()
{
	/* Drive strength configs for sdc pins */
	struct tlmm_cfgs sdc1_hdrv_cfg[] =
	{
		{ SDC1_CLK_HDRV_CTL_OFF,  TLMM_CUR_VAL_16MA, TLMM_HDRV_MASK },
		{ SDC1_CMD_HDRV_CTL_OFF,  TLMM_CUR_VAL_10MA, TLMM_HDRV_MASK },
		{ SDC1_DATA_HDRV_CTL_OFF, TLMM_CUR_VAL_10MA, TLMM_HDRV_MASK },
	};

	/* Pull configs for sdc pins */
	struct tlmm_cfgs sdc1_pull_cfg[] =
	{
		{ SDC1_CLK_PULL_CTL_OFF,  TLMM_NO_PULL, TLMM_PULL_MASK },
		{ SDC1_CMD_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK },
		{ SDC1_DATA_PULL_CTL_OFF, TLMM_PULL_UP, TLMM_PULL_MASK },
	};

	struct tlmm_cfgs sdc1_rclk_cfg[] =
	{
		{ SDC1_RCLK_PULL_CTL_OFF, TLMM_PULL_DOWN, TLMM_PULL_MASK },
	};

	/* Set the drive strength & pull control values */
	tlmm_set_hdrive_ctrl(sdc1_hdrv_cfg, ARRAY_SIZE(sdc1_hdrv_cfg));
	tlmm_set_pull_ctrl(sdc1_pull_cfg, ARRAY_SIZE(sdc1_pull_cfg));

	/* RCLK is supported only with 8974 pro, set rclk to pull down
	 * only for 8974 pro targets
	 */
	if (!platform_is_8974())
		tlmm_set_pull_ctrl(sdc1_rclk_cfg, ARRAY_SIZE(sdc1_rclk_cfg));
}

int emmc_recovery_init(void)
{
	return 0;
}

void target_usb_stop(void)
{

	/* Disable VBUS mimicing in the controller. */
	if (target_needs_vbus_mimic())
		ulpi_write(ULPI_MISC_A_VBUSVLDEXTSEL | ULPI_MISC_A_VBUSVLDEXT, ULPI_MISC_A_CLEAR);
}

/* identify the usb controller to be used for the target */
const char * target_usb_controller()
{
	return "ci";
}

/* UTMI MUX configuration to connect PHY to SNPS controller:
 * Configure primary HS phy mux to use UTMI interface
 * (connected to usb30 controller).
 */
static void tcsr_hs_phy_mux_configure(void)
{
	uint32_t reg;

	reg = readl(USB2_PHY_SEL);

	writel(reg | 0x1, USB2_PHY_SEL);
}

/* configure hs phy mux if using dwc controller */
void target_usb_phy_mux_configure(void)
{
	if(!strcmp(target_usb_controller(), "dwc"))
	{
		tcsr_hs_phy_mux_configure();
	}
}

uint32_t target_get_pmic()
{
	return PMIC_IS_PM8941;
}
