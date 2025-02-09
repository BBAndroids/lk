/*
 * Copyright (c) 2009, Google Inc.
 * All rights reserved.
 *
 * Copyright (c) 2009-2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The Linux Foundation nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <app.h>
#include <debug.h>
#include <arch/arm.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <kernel/thread.h>
#include <arch/ops.h>

#include <dev/flash.h>
#include <dev/flash-ubi.h>
#include <lib/ptable.h>
#include <dev/keys.h>
#include <dev/fbcon.h>
#include <baseband.h>
#include <target.h>
#include <mmc.h>
#include <partition_parser.h>
#include <ab_partition_parser.h>
#include <platform.h>
#include <crypto_hash.h>
#include <malloc.h>
#include <boot_stats.h>
#include <sha.h>
#include <platform/iomap.h>
#include <boot_device.h>
#include <image_verify.h>
#include <decompress.h>
#include <platform/timer.h>
#include <sys/types.h>
#if USE_RPMB_FOR_DEVINFO
#include <rpmb.h>
#endif
#include <panel.h>

#if ENABLE_WBC
#include <pm_app_smbchg.h>
#endif

#if DEVICE_TREE
#include <libfdt.h>
#include <dev_tree.h>
#endif

#if WDOG_SUPPORT
#include <wdog.h>
#endif

#include <reboot.h>
#include "image_verify.h"
#include "recovery.h"
#include "bootimg.h"
#include "fastboot.h"
#include "sparse_format.h"
#include "mmc.h"
#include "devinfo.h"
#include "board.h"
#include "scm.h"
#include "mdtp.h"
#include <menu_keys_detect.h>
#include <display_menu.h>
#include "fastboot_test.h"
#include <platform/bbry.h>
#include <qpnp_led.h>

int target_backlight_type();
extern void platform_uninit(void);
extern void target_uninit(void);
extern int get_target_boot_params(const char *cmdline, const char *part,
				  char **buf);

void *info_buf;
void write_device_info_mmc(device_info *dev);
void write_device_info_flash(device_info *dev);
static inline uint64_t validate_partition_size();
bool pwr_key_is_pressed = false;
static void publish_getvar_multislot_vars();
/* fastboot command function pointer */
typedef void (*fastboot_cmd_fn) (const char *, void *, unsigned);

struct fastboot_cmd_desc {
	char * name;
	fastboot_cmd_fn cb;
};

#define EXPAND(NAME) #NAME
#define TARGET(NAME) EXPAND(NAME)

#define DISPLAY_PANEL_HDMI "hdmi"

#ifdef MEMBASE
#define EMMC_BOOT_IMG_HEADER_ADDR (0xFF000+(MEMBASE))
#else
#define EMMC_BOOT_IMG_HEADER_ADDR 0xFF000
#endif

#ifndef MEMSIZE
#define MEMSIZE 1024*1024
#endif

#define MAX_TAGS_SIZE   1024

/* make 4096 as default size to ensure EFS,EXT4's erasing */
#define DEFAULT_ERASE_SIZE  4096
#define MAX_PANEL_BUF_SIZE 196
#define FOOTER_SIZE 16384

#define DISPLAY_DEFAULT_PREFIX "mdss_mdp"
#define BOOT_DEV_MAX_LEN  64

#define IS_ARM64(ptr) (ptr->magic_64 == KERNEL64_HDR_MAGIC) ? true : false

#define ADD_OF(a, b) (UINT_MAX - b > a) ? (a + b) : UINT_MAX

//Size of the header that is used in case the boot image has
//a uncompressed kernel + appended dtb
#define PATCHED_KERNEL_HEADER_SIZE 20

//String used to determine if the boot image has
//a uncompressed kernel + appended dtb
#define PATCHED_KERNEL_MAGIC "UNCOMPRESSED_IMG"

#if USE_BOOTDEV_CMDLINE
static const char *emmc_cmdline = " androidboot.bootdevice=";
#else
static const char *emmc_cmdline = " androidboot.emmc=true";
#endif
static const char *usb_sn_cmdline = " androidboot.serialno=";

static const char *alarmboot_cmdline = " androidboot.alarmboot=true";
static const char *battchg_pause = " androidboot.mode=charger";
static const char *secondary_gpt_enable = " gpt";
static const char *mdtp_activated_flag = " mdtp";

static const char *baseband_apq     = " androidboot.baseband=apq";
static const char *baseband_msm     = " androidboot.baseband=msm";
static const char *baseband_csfb    = " androidboot.baseband=csfb";
static const char *baseband_svlte2a = " androidboot.baseband=svlte2a";
static const char *baseband_mdm     = " androidboot.baseband=mdm";
static const char *baseband_mdm2    = " androidboot.baseband=mdm2";
static const char *baseband_sglte   = " androidboot.baseband=sglte";
static const char *baseband_dsda    = " androidboot.baseband=dsda";
static const char *baseband_dsda2   = " androidboot.baseband=dsda2";
static const char *baseband_sglte2  = " androidboot.baseband=sglte2";
static const char *warmboot_cmdline = " qpnp-power-on.warm_boot=1";
static const char *baseband_apq_nowgr   = " androidboot.baseband=baseband_apq_nowgr";
static const char *androidboot_slot_suffix = " androidboot.slot_suffix=";
static const char *skip_ramfs = " skip_initramfs";
static const char *sys_path_cmdline = " rootwait ro init=/init";
static const char *sys_path = "  root=/dev/mmcblk0p";
#if WITH_DEBUG_UART
static const char *uart_cmdline = " console=ttyHSL0,115200,n8 earlyprintk";
#endif
static const char *bbry_hwid = " androidboot.binfo.hwid=";
static const char *bbry_rev = " androidboot.binfo.rev=";
static const char *bbry_name = " androidboot.binfo.name=";
static const char *bbry_product = " androidboot.binfo.product=";
static const char *bbry_variant = " androidboot.binfo.variant=";
static const char *bbry_model_wolverine = " androidboot.binfo.model=Passport";
static const char *bbry_model_oslo = " androidboot.binfo.model=\"Passport Silver Edition\"";
static const char *bbry_model_num_unknown = " androidboot.binfo.model_num=Unknown";
static const char *bbry_model_num_sqw100_1 = " androidboot.binfo.model_num=SQW100-1";
static const char *bbry_model_num_sqw100_2 = " androidboot.binfo.model_num=SQW100-2";
static const char *bbry_model_num_sqw100_3 = " androidboot.binfo.model_num=SQW100-3";
static const char *bbry_model_num_sqw100_4 = " androidboot.binfo.model_num=SQW100-4";
static const char *bbry_bsis_type = " androidboot.binfo.bsis_type=bsis_lite";
static const char *bbry_bbss_wp_type = " androidboot.binfo.bbss_wp_type=none";
static const char *bbry_bbss_insecure = " androidboot.binfo.bbss_insecure=false";
static const char *bbry_backup_bc_ver = " androidboot.binfo.backup_bc_ver=";
static const char *bbry_primary_bc_ver = " androidboot.binfo.primary_bc_ver=";
static const char *bbry_samanta_v1 = " backlight=samantaV1";
static const char *bbry_samanta_v2 = " backlight=samantaV2";

/*As per spec delay wait time before shutdown in Red state*/
#define DELAY_WAIT 30000
static unsigned page_size = 0;
static unsigned page_mask = 0;
static unsigned mmc_blocksize = 0;
static unsigned mmc_blocksize_mask = 0;
static char *target_boot_params = NULL;
static bool boot_reason_alarm;
static bool devinfo_present = true;
bool boot_into_fastboot = false;
static uint32_t dt_size = 0;

static device_info device = {DEVICE_MAGIC, 1, 0, {0}};

struct atag_ptbl_entry
{
	char name[16];
	unsigned offset;
	unsigned size;
	unsigned flags;
};

/*
 * Partition info, required to be published
 * for fastboot
 */
struct getvar_partition_info {
	const char part_name[MAX_GPT_NAME_SIZE]; /* Partition name */
	char getvar_size[MAX_GET_VAR_NAME_SIZE]; /* fastboot get var name for size */
	char getvar_type[MAX_GET_VAR_NAME_SIZE]; /* fastboot get var name for type */
	char size_response[MAX_RSP_SIZE];        /* fastboot response for size */
	char type_response[MAX_RSP_SIZE];        /* fastboot response for type */
};

/*
 * Right now, we are publishing the info for only
 * three partitions
 */
struct getvar_partition_info part_info[] =
{
	{ "system"  , "partition-size:", "partition-type:", "", "ext4" },
	{ "userdata", "partition-size:", "partition-type:", "", "ext4" },
	{ "cache"   , "partition-size:", "partition-type:", "", "ext4" },
};

char max_download_size[MAX_RSP_SIZE];
char charger_screen_enabled[MAX_RSP_SIZE];
char sn_buf[13];
char display_panel_buf[MAX_PANEL_BUF_SIZE];
char panel_display_mode[MAX_RSP_SIZE];
char hwid_str[20];
char rev_str[20];

#if CHECK_BAT_VOLTAGE
char battery_voltage[MAX_RSP_SIZE];
char battery_soc_ok [MAX_RSP_SIZE];
#endif

char get_variant[MAX_RSP_SIZE];

extern int emmc_recovery_init(void);

#if NO_KEYPAD_DRIVER
extern int fastboot_trigger(void);
#endif

static void update_ker_tags_rdisk_addr(struct boot_img_hdr *hdr, bool is_arm64)
{
	/* overwrite the destination of specified for the project */
#ifdef ABOOT_IGNORE_BOOT_HEADER_ADDRS
	if (is_arm64)
		hdr->kernel_addr = ABOOT_FORCE_KERNEL64_ADDR;
	else
		hdr->kernel_addr = ABOOT_FORCE_KERNEL_ADDR;
	hdr->ramdisk_addr = ABOOT_FORCE_RAMDISK_ADDR;
	hdr->tags_addr = ABOOT_FORCE_TAGS_ADDR;
#endif
}

static void ptentry_to_tag(unsigned **ptr, struct ptentry *ptn)
{
	struct atag_ptbl_entry atag_ptn;

	memcpy(atag_ptn.name, ptn->name, 16);
	atag_ptn.name[15] = '\0';
	atag_ptn.offset = ptn->start;
	atag_ptn.size = ptn->length;
	atag_ptn.flags = ptn->flags;
	memcpy(*ptr, &atag_ptn, sizeof(struct atag_ptbl_entry));
	*ptr += sizeof(struct atag_ptbl_entry) / sizeof(unsigned);
}

#if CHECK_BAT_VOLTAGE
void update_battery_status(void)
{
	snprintf(battery_voltage,MAX_RSP_SIZE, "%d",target_get_battery_voltage());
	snprintf(battery_soc_ok ,MAX_RSP_SIZE, "%s",target_battery_soc_ok()? "yes":"no");
}
#endif

unsigned char *update_cmdline(const char * cmdline)
{
	int cmdline_len = 0;
	int have_cmdline = 0;
	unsigned char *cmdline_final = NULL;
	int pause_at_bootup = 0;
	bool warm_boot = false;
	bool gpt_exists = partition_gpt_exists();
	int have_target_boot_params = 0;
	char *boot_dev_buf = NULL;
    	bool is_mdtp_activated = 0;
	int current_active_slot = INVALID;
	int system_ptn_index = -1;
	unsigned int lun = 0;
	char lun_char_base = 'a';
	int syspath_buflen = strlen(sys_path) + sizeof(int) + 1; /*allocate buflen for largest possible string*/
	char syspath_buf[syspath_buflen];

#ifdef MDTP_SUPPORT
    mdtp_activated(&is_mdtp_activated);
#endif /* MDTP_SUPPORT */

	if (cmdline && cmdline[0]) {
		cmdline_len = strlen(cmdline);
		have_cmdline = 1;
	}
	if (target_is_emmc_boot()) {
		cmdline_len += strlen(emmc_cmdline);
#if USE_BOOTDEV_CMDLINE
		boot_dev_buf = (char *) malloc(sizeof(char) * BOOT_DEV_MAX_LEN);
		ASSERT(boot_dev_buf);
		platform_boot_dev_cmdline(boot_dev_buf);
		cmdline_len += strlen(boot_dev_buf);
#endif
	}

	cmdline_len += strlen(usb_sn_cmdline);
	cmdline_len += strlen(sn_buf);

	if (boot_into_recovery && gpt_exists)
		cmdline_len += strlen(secondary_gpt_enable);

	if(is_mdtp_activated)
		cmdline_len += strlen(mdtp_activated_flag);

	if (boot_reason_alarm) {
		cmdline_len += strlen(alarmboot_cmdline);
	} else if ((target_build_variant_user() || device.charger_screen_enabled)
			&& target_pause_for_battery_charge()) {
		pause_at_bootup = 1;
		cmdline_len += strlen(battchg_pause);
	}

	if (get_target_boot_params(cmdline, boot_into_recovery ? "recoveryfs" :
								 "system",
						&target_boot_params) == 0) {
		have_target_boot_params = 1;
		cmdline_len += strlen(target_boot_params);
	}

	/* Determine correct androidboot.baseband to use */
	switch(target_baseband())
	{
		case BASEBAND_APQ:
			cmdline_len += strlen(baseband_apq);
			break;

		case BASEBAND_MSM:
			cmdline_len += strlen(baseband_msm);
			break;

		case BASEBAND_CSFB:
			cmdline_len += strlen(baseband_csfb);
			break;

		case BASEBAND_SVLTE2A:
			cmdline_len += strlen(baseband_svlte2a);
			break;

		case BASEBAND_MDM:
			cmdline_len += strlen(baseband_mdm);
			break;

		case BASEBAND_MDM2:
			cmdline_len += strlen(baseband_mdm2);
			break;

		case BASEBAND_SGLTE:
			cmdline_len += strlen(baseband_sglte);
			break;

		case BASEBAND_SGLTE2:
			cmdline_len += strlen(baseband_sglte2);
			break;

		case BASEBAND_DSDA:
			cmdline_len += strlen(baseband_dsda);
			break;

		case BASEBAND_DSDA2:
			cmdline_len += strlen(baseband_dsda2);
			break;
		case BASEBAND_APQ_NOWGR:
			cmdline_len += strlen(baseband_apq_nowgr);
			break;
	}

#if ENABLE_DISPLAY
	if (cmdline) {
		if ((strstr(cmdline, DISPLAY_DEFAULT_PREFIX) == NULL) &&
			target_display_panel_node(display_panel_buf,
			MAX_PANEL_BUF_SIZE) &&
			strlen(display_panel_buf)) {
			cmdline_len += strlen(display_panel_buf);
		}
	}
#endif

	if (target_warm_boot()) {
		warm_boot = true;
		cmdline_len += strlen(warmboot_cmdline);
	}

	if (partition_multislot_is_supported())
	{
		current_active_slot = partition_find_active_slot();
		cmdline_len += (strlen(androidboot_slot_suffix)+
					strlen(SUFFIX_SLOT(current_active_slot)));

		system_ptn_index = partition_get_index("system");
		if (platform_boot_dev_isemmc())
		{
			snprintf(syspath_buf, syspath_buflen, " root=/dev/mmcblk0p%d",
				system_ptn_index + 1);
		}
		else
		{
			lun = partition_get_lun(system_ptn_index);
			snprintf(syspath_buf, syspath_buflen, " root=/dev/sd%c%d",
					lun_char_base + lun,
					partition_get_index_in_lun("system", lun));
		}

		cmdline_len += strlen(sys_path_cmdline);
		cmdline_len += strlen(syspath_buf);
		if (!boot_into_recovery)
			cmdline_len += strlen(skip_ramfs);
	}

#if WITH_DEBUG_UART
	if (device.jack_uart_enabled)
		cmdline_len += strlen(uart_cmdline);
#endif

	char *product = bbry_get_product();
	char *variant = bbry_get_variant();
	char *backup_bc_ver = bbry_hwi_get_entry("backup_bc_ver", 0);
	char *primary_bc_ver = bbry_hwi_get_entry("primary_bc_ver", 0);

	snprintf(hwid_str, 20, "0x%x", bbry_get_hwid());
	snprintf(rev_str, 20, "%d", bbry_get_rev());

	cmdline_len += strlen(bbry_hwid);
	cmdline_len += strlen(hwid_str);

	cmdline_len += strlen(bbry_rev);
	cmdline_len += strlen(rev_str);

	cmdline_len += strlen(bbry_name);
	cmdline_len += strlen(product);
	cmdline_len += strlen(variant);

	cmdline_len += strlen(bbry_product);
	cmdline_len += strlen(product);

	cmdline_len += strlen(bbry_variant);
	cmdline_len += strlen(variant);

	if (strcmp(bbry_get_product(), "wolverine") == 0) {
		cmdline_len += strlen(bbry_model_wolverine);

		if (strcmp(bbry_get_variant(), "emea") == 0) {
			cmdline_len += strlen(bbry_model_num_sqw100_1);
		} else if (strcmp(bbry_get_variant(), "vzw") == 0) {
			cmdline_len += strlen(bbry_model_num_sqw100_2);
		} else if (strcmp(bbry_get_variant(), "na") == 0) {
			cmdline_len += strlen(bbry_model_num_sqw100_3);
		} else {
			cmdline_len += strlen(bbry_model_num_unknown);
		}
	} else {
		cmdline_len += strlen(bbry_model_oslo);

		cmdline_len += strlen(bbry_model_num_sqw100_4);
	}

	cmdline_len += strlen(bbry_bsis_type);

	cmdline_len += strlen(bbry_bbss_wp_type);

	cmdline_len += strlen(bbry_bbss_insecure);

	if (backup_bc_ver) {
		cmdline_len += strlen(bbry_backup_bc_ver);
		cmdline_len += strlen(backup_bc_ver);
	}

	if (primary_bc_ver) {
		cmdline_len += strlen(bbry_primary_bc_ver);
		cmdline_len += strlen(primary_bc_ver);
	}

	if (target_backlight_type() == BL_SAMANTA_V1) {
		cmdline_len += strlen(bbry_samanta_v1);
	} else if (target_backlight_type() == BL_SAMANTA_V2) {
		cmdline_len += strlen(bbry_samanta_v2);
	}

	if (cmdline_len > 0) {
		const char *src;
		unsigned char *dst;

		cmdline_final = (unsigned char*) malloc((cmdline_len + 4) & (~3));
		ASSERT(cmdline_final != NULL);
		memset((void *)cmdline_final, 0, sizeof(*cmdline_final));
		dst = cmdline_final;

		/* Save start ptr for debug print */
		if (have_cmdline) {
			src = cmdline;
			while ((*dst++ = *src++));
		}
		if (target_is_emmc_boot()) {
			src = emmc_cmdline;
			if (have_cmdline) --dst;
			have_cmdline = 1;
			while ((*dst++ = *src++));
#if USE_BOOTDEV_CMDLINE
			src = boot_dev_buf;
			if (have_cmdline) --dst;
			while ((*dst++ = *src++));
#endif
		}

		src = usb_sn_cmdline;
		if (have_cmdline) --dst;
		have_cmdline = 1;
		while ((*dst++ = *src++));
		src = sn_buf;
		if (have_cmdline) --dst;
		have_cmdline = 1;
		while ((*dst++ = *src++));
		if (warm_boot) {
			if (have_cmdline) --dst;
			src = warmboot_cmdline;
			while ((*dst++ = *src++));
		}

		if (boot_into_recovery && gpt_exists) {
			src = secondary_gpt_enable;
			if (have_cmdline) --dst;
			while ((*dst++ = *src++));
		}

		if (is_mdtp_activated) {
			src = mdtp_activated_flag;
			if (have_cmdline) --dst;
			while ((*dst++ = *src++));
		}

		if (boot_reason_alarm) {
			src = alarmboot_cmdline;
			if (have_cmdline) --dst;
			while ((*dst++ = *src++));
		} else if (pause_at_bootup) {
			src = battchg_pause;
			if (have_cmdline) --dst;
			while ((*dst++ = *src++));
		}

		switch(target_baseband())
		{
			case BASEBAND_APQ:
				src = baseband_apq;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_MSM:
				src = baseband_msm;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_CSFB:
				src = baseband_csfb;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_SVLTE2A:
				src = baseband_svlte2a;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_MDM:
				src = baseband_mdm;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_MDM2:
				src = baseband_mdm2;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_SGLTE:
				src = baseband_sglte;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_SGLTE2:
				src = baseband_sglte2;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_DSDA:
				src = baseband_dsda;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;

			case BASEBAND_DSDA2:
				src = baseband_dsda2;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;
			case BASEBAND_APQ_NOWGR:
				src = baseband_apq_nowgr;
				if (have_cmdline) --dst;
				while ((*dst++ = *src++));
				break;
		}

		if (strlen(display_panel_buf)) {
			src = display_panel_buf;
			if (have_cmdline) --dst;
			while ((*dst++ = *src++));
		}

		if (have_target_boot_params) {
			if (have_cmdline) --dst;
			src = target_boot_params;
			while ((*dst++ = *src++));
			free(target_boot_params);
		}

		if (partition_multislot_is_supported() && have_cmdline)
		{
				src = androidboot_slot_suffix;
				--dst;
				while ((*dst++ = *src++));
				--dst;
				src = SUFFIX_SLOT(current_active_slot);
				while ((*dst++ = *src++));

				if (!boot_into_recovery)
				{
					src = skip_ramfs;
					--dst;
					while ((*dst++ = *src++));
				}

				src = sys_path_cmdline;
				--dst;
				while ((*dst++ = *src++));

				src = syspath_buf;
				--dst;
				while ((*dst++ = *src++));
		}

#if WITH_DEBUG_UART
		if (device.jack_uart_enabled) {
			if (have_cmdline) --dst;
			src = uart_cmdline;
			while ((*dst++ = *src++));
		}
#endif

		if (have_cmdline) --dst;
		src = bbry_hwid;
		while ((*dst++ = *src++));
		if (have_cmdline) --dst;
		src = hwid_str;
		while ((*dst++ = *src++));

		if (have_cmdline) --dst;
		src = bbry_rev;
		while ((*dst++ = *src++));
		if (have_cmdline) --dst;
		src = rev_str;
		while ((*dst++ = *src++));

		if (have_cmdline) --dst;
		src = bbry_name;
		while ((*dst++ = *src++));
		if (have_cmdline) --dst;
		src = product;
		while ((*dst++ = *src++));
		if (have_cmdline) --dst;
		src = variant;
		while ((*dst++ = *src++));

		if (have_cmdline) --dst;
		src = bbry_product;
		while ((*dst++ = *src++));
		if (have_cmdline) --dst;
		src = product;
		while ((*dst++ = *src++));

		if (have_cmdline) --dst;
		src = bbry_variant;
		while ((*dst++ = *src++));
		if (have_cmdline) --dst;
		src = variant;
		while ((*dst++ = *src++));

		if (strcmp(bbry_get_product(), "wolverine") == 0) {
			if (have_cmdline)
			--dst;
			src = bbry_model_wolverine;
			while ((*dst++ = *src++));

			if (strcmp(bbry_get_variant(), "emea") == 0) {
				if (have_cmdline) --dst;
				src = bbry_model_num_sqw100_1;
				while ((*dst++ = *src++));
			} else if (strcmp(bbry_get_variant(), "vzw") == 0) {
				if (have_cmdline) --dst;
				src = bbry_model_num_sqw100_2;
				while ((*dst++ = *src++));
			} else if (strcmp(bbry_get_variant(), "na") == 0) {
				if (have_cmdline) --dst;
				src = bbry_model_num_sqw100_3;
				while ((*dst++ = *src++));
			} else {
				if (have_cmdline) --dst;
				src = bbry_model_num_unknown;
				while ((*dst++ = *src++));
			}
		} else {
			if (have_cmdline)
			--dst;
			src = bbry_model_oslo;
			while ((*dst++ = *src++));

			if (have_cmdline) --dst;
			src = bbry_model_num_sqw100_4;
			while ((*dst++ = *src++));
		}

		if (have_cmdline) --dst;
		src = bbry_bsis_type;
		while ((*dst++ = *src++));

		if (have_cmdline) --dst;
		src = bbry_bbss_wp_type;
		while ((*dst++ = *src++));

		if (have_cmdline) --dst;
		src = bbry_bbss_insecure;
		while ((*dst++ = *src++));

		if (backup_bc_ver) {
			if (have_cmdline) --dst;
			src = bbry_backup_bc_ver;
			while ((*dst++ = *src++));
			if (have_cmdline) --dst;
			src = backup_bc_ver;
			while ((*dst++ = *src++));
		}

		if (primary_bc_ver) {
			if (have_cmdline) --dst;
			src = bbry_primary_bc_ver;
			while ((*dst++ = *src++));
			if (have_cmdline) --dst;
			src = primary_bc_ver;
			while ((*dst++ = *src++));
		}

		if (target_backlight_type() == BL_SAMANTA_V1) {
			if (have_cmdline) --dst;
			src = bbry_samanta_v1;
			while ((*dst++ = *src++));
		} else if (target_backlight_type() == BL_SAMANTA_V2) {
			if (have_cmdline) --dst;
			src = bbry_samanta_v2;
			while ((*dst++ = *src++));
		}
	}

	if (boot_dev_buf)
		free(boot_dev_buf);

	if (cmdline_final)
		dprintf(INFO, "cmdline: %s\n", cmdline_final);
	else
		dprintf(INFO, "cmdline is NULL\n");
	return cmdline_final;
}

unsigned *atag_core(unsigned *ptr)
{
	/* CORE */
	*ptr++ = 2;
	*ptr++ = 0x54410001;

	return ptr;

}

unsigned *atag_ramdisk(unsigned *ptr, void *ramdisk,
							   unsigned ramdisk_size)
{
	if (ramdisk_size) {
		*ptr++ = 4;
		*ptr++ = 0x54420005;
		*ptr++ = (unsigned)ramdisk;
		*ptr++ = ramdisk_size;
	}

	return ptr;
}

unsigned *atag_ptable(unsigned **ptr_addr)
{
	int i;
	struct ptable *ptable;

	if ((ptable = flash_get_ptable()) && (ptable->count != 0)) {
		*(*ptr_addr)++ = 2 + (ptable->count * (sizeof(struct atag_ptbl_entry) /
							sizeof(unsigned)));
		*(*ptr_addr)++ = 0x4d534d70;
		for (i = 0; i < ptable->count; ++i)
			ptentry_to_tag(ptr_addr, ptable_get(ptable, i));
	}

	return (*ptr_addr);
}

unsigned *atag_cmdline(unsigned *ptr, const char *cmdline)
{
	int cmdline_length = 0;
	int n;
	char *dest;

	cmdline_length = strlen((const char*)cmdline);
	n = (cmdline_length + 4) & (~3);

	*ptr++ = (n / 4) + 2;
	*ptr++ = 0x54410009;
	dest = (char *) ptr;
	while ((*dest++ = *cmdline++));
	ptr += (n / 4);

	return ptr;
}

unsigned *atag_end(unsigned *ptr)
{
	/* END */
	*ptr++ = 0;
	*ptr++ = 0;

	return ptr;
}

void generate_atags(unsigned *ptr, const char *cmdline,
                    void *ramdisk, unsigned ramdisk_size)
{
	unsigned *orig_ptr = ptr;
	ptr = atag_core(ptr);
	ptr = atag_ramdisk(ptr, ramdisk, ramdisk_size);
	ptr = target_atag_mem(ptr);

	/* Skip NAND partition ATAGS for eMMC boot */
	if (!target_is_emmc_boot()){
		ptr = atag_ptable(&ptr);
	}

	/*
	 * Atags size filled till + cmdline size + 1 unsigned for 4-byte boundary + 4 unsigned
	 * for atag identifier in atag_cmdline and atag_end should be with in MAX_TAGS_SIZE bytes
	 */
	if (((ptr - orig_ptr) + strlen(cmdline) + 5 * sizeof(unsigned)) <  MAX_TAGS_SIZE) {
		ptr = atag_cmdline(ptr, cmdline);
		ptr = atag_end(ptr);
	}
	else {
		dprintf(CRITICAL,"Crossing ATAGs Max size allowed\n");
		ASSERT(0);
	}
}

typedef void entry_func_ptr(unsigned, unsigned, unsigned*);
void boot_linux(void *kernel, unsigned *tags,
		const char *cmdline, unsigned machtype,
		void *ramdisk, unsigned ramdisk_size)
{
	unsigned char *final_cmdline;
#if DEVICE_TREE
	int ret = 0;
#endif

	void (*entry)(unsigned, unsigned, unsigned*) = (entry_func_ptr*)(PA((addr_t)kernel));
	uint32_t tags_phys = PA((addr_t)tags);
	struct kernel64_hdr *kptr = ((struct kernel64_hdr*)(PA((addr_t)kernel)));

	ramdisk = (void *)PA((addr_t)ramdisk);

	final_cmdline = update_cmdline((const char*)cmdline);

#if DEVICE_TREE
	dprintf(INFO, "Updating device tree: start\n");

	/* Update the Device Tree */
	ret = update_device_tree((void *)tags,(const char *)final_cmdline, ramdisk, ramdisk_size);
	if(ret)
	{
		dprintf(CRITICAL, "ERROR: Updating Device Tree Failed \n");
		ASSERT(0);
	}
	dprintf(INFO, "Updating device tree: done\n");
#else
	/* Generating the Atags */
	generate_atags(tags, final_cmdline, ramdisk, ramdisk_size);
#endif

	free(final_cmdline);

	/* Turn off splash screen if enabled */
#if DISPLAY_SPLASH_SCREEN
	target_display_shutdown();
#endif

	/* Perform target specific cleanup */
	target_uninit();

	dprintf(INFO, "booting linux @ %p, ramdisk @ %p (%d), tags/device tree @ %p\n",
		entry, ramdisk, ramdisk_size, (void *)tags_phys);

	if (!device.jack_uart_enabled)
		bbry_uart_on_jack(0);

	enter_critical_section();

	/* do any platform specific cleanup before kernel entry */
	platform_uninit();

	arch_disable_cache(UCACHE);

#if ARM_WITH_MMU
	arch_disable_mmu();
#endif
	bs_set_timestamp(BS_KERNEL_ENTRY);

	if (IS_ARM64(kptr))
		/* Jump to a 64bit kernel */
		scm_elexec_call((paddr_t)kernel, tags_phys);
	else
		/* Jump to a 32bit kernel */
		entry(0, machtype, (unsigned*)tags_phys);
}

/* Function to check if the memory address range falls within the aboot
 * boundaries.
 * start: Start of the memory region
 * size: Size of the memory region
 */
int check_aboot_addr_range_overlap(uintptr_t start, uint32_t size)
{
	/* Check for boundary conditions. */
	if ((UINT_MAX - start) < size)
		return -1;

	/* Check for memory overlap. */
	if ((start < MEMBASE) && ((start + size) <= MEMBASE))
		return 0;
	else if (start >= (MEMBASE + MEMSIZE))
		return 0;
	else
		return -1;
}

/* Function to check if the memory address range falls beyond ddr region.
 * start: Start of the memory region
 * size: Size of the memory region
 */
int check_ddr_addr_range_bound(uintptr_t start, uint32_t size)
{
	uintptr_t ddr_pa_start_addr = PA(get_ddr_start());
	uint64_t ddr_size = smem_get_ddr_size();
	uint64_t ddr_pa_end_addr = ddr_pa_start_addr + ddr_size;
	uintptr_t pa_start_addr = PA(start);

	/* Check for boundary conditions. */
	if ((UINT_MAX - start) < size)
		return -1;

	/* Check if memory range is beyond the ddr range. */
	if (pa_start_addr < ddr_pa_start_addr ||
		pa_start_addr >= (ddr_pa_end_addr) ||
		(pa_start_addr + size) > ddr_pa_end_addr)
		return -1;
	else
		return 0;
}

BUF_DMA_ALIGN(buf, BOOT_IMG_MAX_PAGE_SIZE); //Equal to max-supported pagesize

static bool check_format_bit()
{
	bool ret = false;
	int index;
	uint64_t offset;
	struct boot_selection_info *in = NULL;
	char *buf = NULL;

	index = partition_get_index("bootselect");
	if (index == INVALID_PTN)
	{
		dprintf(INFO, "Unable to locate /bootselect partition\n");
		return ret;
	}
	offset = partition_get_offset(index);
	if(!offset)
	{
		dprintf(INFO, "partition /bootselect doesn't exist\n");
		return ret;
	}
	buf = (char *) memalign(CACHE_LINE, ROUNDUP(page_size, CACHE_LINE));
	mmc_set_lun(partition_get_lun(index));
	ASSERT(buf);
	if (mmc_read(offset, (uint32_t *)buf, page_size))
	{
		dprintf(INFO, "mmc read failure /bootselect %d\n", page_size);
		free(buf);
		return ret;
	}
	in = (struct boot_selection_info *) buf;
	if ((in->signature == BOOTSELECT_SIGNATURE) &&
			(in->version == BOOTSELECT_VERSION)) {
		if ((in->state_info & BOOTSELECT_FORMAT) &&
				!(in->state_info & BOOTSELECT_FACTORY))
			ret = true;
	} else {
		dprintf(CRITICAL, "Signature: 0x%08x or version: 0x%08x mismatched of /bootselect\n",
				in->signature, in->version);
		ASSERT(0);
	}
	free(buf);
	return ret;
}

int boot_linux_from_mmc(void)
{
	struct boot_img_hdr *hdr = (void*) buf;
	struct boot_img_hdr *uhdr;
	unsigned offset = 0;
	int rcode;
	unsigned long long ptn = 0;
	int index = INVALID_PTN;

	unsigned char *image_addr = 0;
	unsigned kernel_actual;
	unsigned ramdisk_actual;
	unsigned imagesize_actual;
	unsigned second_actual = 0;

	unsigned int dtb_size = 0;
	unsigned int out_len = 0;
	unsigned int out_avai_len = 0;
	unsigned char *out_addr = NULL;
	uint32_t dtb_offset = 0;
	unsigned char *kernel_start_addr = NULL;
	unsigned int kernel_size = 0;
	unsigned int patched_kernel_hdr_size = 0;
	int rc;
#if DEVICE_TREE
	struct dt_table *table;
	struct dt_entry dt_entry;
	unsigned dt_table_offset;
	uint32_t dt_actual;
	uint32_t dt_hdr_size;
	unsigned char *best_match_dt_addr = NULL;
#endif
	struct kernel64_hdr *kptr = NULL;
	int current_active_slot = INVALID;

	if (check_format_bit())
		boot_into_recovery = 1;

	uhdr = (struct boot_img_hdr *)EMMC_BOOT_IMG_HEADER_ADDR;
	if (!memcmp(uhdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
		dprintf(INFO, "Unified boot method!\n");
		hdr = uhdr;
		goto unified_boot;
	}
	if (!boot_into_recovery) {
		qpnp_led_set(0, 0, 0);
		index = partition_get_index("boot");
		ptn = partition_get_offset(index);
		if(ptn == 0) {
			dprintf(CRITICAL, "ERROR: No boot partition found\n");
                    return -1;
		}
	}
	else {
		qpnp_led_set(0x80, 0x80, 0);
		index = partition_get_index("recovery");
		ptn = partition_get_offset(index);
		if(ptn == 0) {
			dprintf(CRITICAL, "ERROR: No recovery partition found\n");
                    return -1;
		}
	}
	/* Set Lun for boot & recovery partitions */
	mmc_set_lun(partition_get_lun(index));

	if (mmc_read(ptn + offset, (uint32_t *) buf, page_size)) {
		dprintf(CRITICAL, "ERROR: Cannot read boot image header\n");
                return -1;
	}

	if (memcmp(hdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
		dprintf(CRITICAL, "ERROR: Invalid boot image header\n");
                return ERR_INVALID_BOOT_MAGIC;
	}

	if (hdr->page_size && (hdr->page_size != page_size)) {

		if (hdr->page_size > BOOT_IMG_MAX_PAGE_SIZE) {
			dprintf(CRITICAL, "ERROR: Invalid page size\n");
			return -1;
		}
		page_size = hdr->page_size;
		page_mask = page_size - 1;
	}

	kernel_actual  = ROUND_TO_PAGE(hdr->kernel_size,  page_mask);
	ramdisk_actual = ROUND_TO_PAGE(hdr->ramdisk_size, page_mask);
	second_actual  = ROUND_TO_PAGE(hdr->second_size, page_mask);

	image_addr = (unsigned char *)target_get_scratch_address();
	memcpy(image_addr, (void *)buf, page_size);

	/* ensure commandline is terminated */
        hdr->cmdline[BOOT_ARGS_SIZE-1] = 0;

#if DEVICE_TREE
#ifndef OSVERSION_IN_BOOTIMAGE
	dt_size = hdr->dt_size;
#endif
	dt_actual = ROUND_TO_PAGE(dt_size, page_mask);
	if (UINT_MAX < ((uint64_t)kernel_actual + (uint64_t)ramdisk_actual+ (uint64_t)second_actual + (uint64_t)dt_actual + page_size)) {
		dprintf(CRITICAL, "Integer overflow detected in bootimage header fields at %u in %s\n",__LINE__,__FILE__);
		return -1;
	}
	imagesize_actual = (page_size + kernel_actual + ramdisk_actual + second_actual + dt_actual);
#else
	if (UINT_MAX < ((uint64_t)kernel_actual + (uint64_t)ramdisk_actual + (uint64_t)second_actual + page_size)) {
		dprintf(CRITICAL, "Integer overflow detected in bootimage header fields at %u in %s\n",__LINE__,__FILE__);
		return -1;
	}
	imagesize_actual = (page_size + kernel_actual + ramdisk_actual + second_actual);
#endif

	if (check_aboot_addr_range_overlap((uintptr_t) image_addr, imagesize_actual))
	{
		dprintf(CRITICAL, "Boot image buffer address overlaps with aboot addresses.\n");
		return -1;
	}

	/*
	 * Update loading flow of bootimage to support compressed/uncompressed
	 * bootimage on both 64bit and 32bit platform.
	 * 1. Load bootimage from emmc partition onto DDR.
	 * 2. Check if bootimage is gzip format. If yes, decompress compressed kernel
	 * 3. Check kernel header and update kernel load addr for 64bit and 32bit
	 *    platform accordingly.
	 * 4. Sanity Check on kernel_addr and ramdisk_addr and copy data.
	 */
	if (partition_multislot_is_supported())
	{
		current_active_slot = partition_find_active_slot();
		dprintf(INFO, "Loading boot image (%d) active_slot(%s): start\n",
				imagesize_actual, SUFFIX_SLOT(current_active_slot));
	}
	else
	{
		dprintf(INFO, "Loading (%s) image (%d): start\n",
				(!boot_into_recovery ? "boot" : "recovery"),imagesize_actual);
	}
	bs_set_timestamp(BS_KERNEL_LOAD_START);

	if ((target_get_max_flash_size() - page_size) < imagesize_actual)
	{
		dprintf(CRITICAL, "booimage  size is greater than DDR can hold\n");
		return -1;
	}
	offset = page_size;
	/* Read image without signature and header*/
	if (mmc_read(ptn + offset, (void *)(image_addr + offset), imagesize_actual - page_size))
	{
		dprintf(CRITICAL, "ERROR: Cannot read boot image\n");
		return -1;
	}

	if (partition_multislot_is_supported())
	{
		dprintf(INFO, "Loading boot image (%d) active_slot(%s): done\n",
				imagesize_actual, SUFFIX_SLOT(current_active_slot));
	}
	else
	{
		dprintf(INFO, "Loading (%s) image (%d): done\n",
			(!boot_into_recovery ? "boot" : "recovery"),imagesize_actual);

	}
	bs_set_timestamp(BS_KERNEL_LOAD_DONE);

	/*
	 * Check if the kernel image is a gzip package. If yes, need to decompress it.
	 * If not, continue booting.
	 */
	if (is_gzip_package((unsigned char *)(image_addr + page_size), hdr->kernel_size))
	{
		out_addr = (unsigned char *)(image_addr + imagesize_actual + page_size);
		out_avai_len = target_get_max_flash_size() - imagesize_actual - page_size;
		dprintf(INFO, "decompressing kernel image: start\n");
		rc = decompress((unsigned char *)(image_addr + page_size),
				hdr->kernel_size, out_addr, out_avai_len,
				&dtb_offset, &out_len);
		if (rc)
		{
			dprintf(CRITICAL, "decompressing kernel image failed!!!\n");
			ASSERT(0);
		}

		dprintf(INFO, "decompressing kernel image: done\n");
		kptr = (struct kernel64_hdr *)out_addr;
		kernel_start_addr = out_addr;
		kernel_size = out_len;
	} else {
		dprintf(INFO, "Uncpmpressed kernel in use\n");
		if (!strncmp((char*)(image_addr + page_size),
					PATCHED_KERNEL_MAGIC,
					sizeof(PATCHED_KERNEL_MAGIC) - 1)) {
			dprintf(INFO, "Patched kernel detected\n");
			kptr = (struct kernel64_hdr *)(image_addr + page_size +
					PATCHED_KERNEL_HEADER_SIZE);
			//The size of the kernel is stored at start of kernel image + 16
			//The dtb would start just after the kernel
			dtb_offset = *((uint32_t*)((unsigned char*)
						(image_addr + page_size +
						 sizeof(PATCHED_KERNEL_MAGIC) -
						 1)));
			//The actual kernel starts after the 20 byte header.
			kernel_start_addr = (unsigned char*)(image_addr +
					page_size + PATCHED_KERNEL_HEADER_SIZE);
			kernel_size = hdr->kernel_size;
			patched_kernel_hdr_size = PATCHED_KERNEL_HEADER_SIZE;
		} else {
			dprintf(INFO, "Kernel image not patched..Unable to locate dt offset\n");
			kptr = (struct kernel64_hdr *)(image_addr + page_size);
			kernel_start_addr = (unsigned char *)(image_addr + page_size);
			kernel_size = hdr->kernel_size;
		}
	}

	/*
	 * Update the kernel/ramdisk/tags address if the boot image header
	 * has default values, these default values come from mkbootimg when
	 * the boot image is flashed using fastboot flash:raw
	 */
	update_ker_tags_rdisk_addr(hdr, IS_ARM64(kptr));

	/* Get virtual addresses since the hdr saves physical addresses. */
	hdr->kernel_addr = VA((addr_t)(hdr->kernel_addr));
	hdr->ramdisk_addr = VA((addr_t)(hdr->ramdisk_addr));
	hdr->tags_addr = VA((addr_t)(hdr->tags_addr));

	kernel_size = ROUND_TO_PAGE(kernel_size,  page_mask);
	/* Check if the addresses in the header are valid. */
	if (check_aboot_addr_range_overlap(hdr->kernel_addr, kernel_size) ||
		check_ddr_addr_range_bound(hdr->kernel_addr, kernel_size) ||
		check_aboot_addr_range_overlap(hdr->ramdisk_addr, ramdisk_actual) ||
		check_ddr_addr_range_bound(hdr->ramdisk_addr, ramdisk_actual))
	{
		dprintf(CRITICAL, "kernel/ramdisk addresses are not valid.\n");
		return -1;
	}

#ifndef DEVICE_TREE
	if (check_aboot_addr_range_overlap(hdr->tags_addr, MAX_TAGS_SIZE) ||
		check_ddr_addr_range_bound(hdr->tags_addr, MAX_TAGS_SIZE))
	{
		dprintf(CRITICAL, "Tags addresses are not valid.\n");
		return -1;
	}
#endif

	/* Move kernel, ramdisk and device tree to correct address */
	memmove((void*) hdr->kernel_addr, kernel_start_addr, kernel_size);
	memmove((void*) hdr->ramdisk_addr, (char *)(image_addr + page_size + kernel_actual), hdr->ramdisk_size);

	#if DEVICE_TREE
	if(dt_size) {
		dt_table_offset = ((uint32_t)image_addr + page_size + kernel_actual + ramdisk_actual + second_actual);
		table = (struct dt_table*) dt_table_offset;

		if (dev_tree_validate(table, hdr->page_size, &dt_hdr_size) != 0) {
			dprintf(CRITICAL, "ERROR: Cannot validate Device Tree Table \n");
			return -1;
		}

		/* Its Error if, dt_hdr_size (table->num_entries * dt_entry size + Dev_Tree Header)
		goes beyound hdr->dt_size*/
		if (dt_hdr_size > ROUND_TO_PAGE(dt_size,hdr->page_size)) {
			dprintf(CRITICAL, "ERROR: Invalid Device Tree size \n");
			return -1;
		}

		/* Find index of device tree within device tree table */
		if(dev_tree_get_entry_info(table, &dt_entry) != 0){
			dprintf(CRITICAL, "ERROR: Getting device tree address failed\n");
			return -1;
		}

		if(dt_entry.offset > (UINT_MAX - dt_entry.size)) {
			dprintf(CRITICAL, "ERROR: Device tree contents are Invalid\n");
			return -1;
		}

		/* Ensure we are not overshooting dt_size with the dt_entry selected */
		if ((dt_entry.offset + dt_entry.size) > dt_size) {
			dprintf(CRITICAL, "ERROR: Device tree contents are Invalid\n");
			return -1;
		}

		if (is_gzip_package((unsigned char *)dt_table_offset + dt_entry.offset, dt_entry.size))
		{
			unsigned int compressed_size = 0;
			out_addr += out_len;
			out_avai_len -= out_len;
			dprintf(INFO, "decompressing dtb: start\n");
			rc = decompress((unsigned char *)dt_table_offset + dt_entry.offset,
					dt_entry.size, out_addr, out_avai_len,
					&compressed_size, &dtb_size);
			if (rc)
			{
				dprintf(CRITICAL, "decompressing dtb failed!!!\n");
				ASSERT(0);
			}

			dprintf(INFO, "decompressing dtb: done\n");
			best_match_dt_addr = out_addr;
		} else {
			best_match_dt_addr = (unsigned char *)dt_table_offset + dt_entry.offset;
			dtb_size = dt_entry.size;
		}

		/* Validate and Read device device tree in the tags_addr */
		if (check_aboot_addr_range_overlap(hdr->tags_addr, dtb_size) ||
			check_ddr_addr_range_bound(hdr->tags_addr, dtb_size))
		{
			dprintf(CRITICAL, "Device tree addresses are not valid\n");
			return -1;
		}

		memmove((void *)hdr->tags_addr, (char *)best_match_dt_addr, dtb_size);
	} else {
		/* Validate the tags_addr */
		if (check_aboot_addr_range_overlap(hdr->tags_addr, kernel_actual) ||
			check_ddr_addr_range_bound(hdr->tags_addr, kernel_actual))
		{
			dprintf(CRITICAL, "Device tree addresses are not valid.\n");
			return -1;
		}
		/*
		 * If appended dev tree is found, update the atags with
		 * memory address to the DTB appended location on RAM.
		 * Else update with the atags address in the kernel header
		 */
		void *dtb;
		dtb = dev_tree_appended(
				(void*)(image_addr + page_size +
					patched_kernel_hdr_size),
				hdr->kernel_size, dtb_offset,
				(void *)hdr->tags_addr);
		if (!dtb) {
			dprintf(CRITICAL, "ERROR: Appended Device Tree Blob not found\n");
			return -1;
		}
	}
	#endif

unified_boot:

	boot_linux((void *)hdr->kernel_addr, (void *)hdr->tags_addr,
		   (const char *)hdr->cmdline, board_machtype(),
		   (void *)hdr->ramdisk_addr, hdr->ramdisk_size);

	return 0;
}

int boot_linux_from_flash(void)
{
	struct boot_img_hdr *hdr = (void*) buf;
	struct ptentry *ptn;
	struct ptable *ptable;
	unsigned offset = 0;

	unsigned char *image_addr = 0;
	unsigned kernel_actual;
	unsigned ramdisk_actual;
	unsigned imagesize_actual;
	unsigned second_actual = 0;

#if DEVICE_TREE
	struct dt_table *table;
	struct dt_entry dt_entry;
	unsigned dt_table_offset;
	uint32_t dt_actual;
	uint32_t dt_hdr_size;
	unsigned int dtb_size = 0;
	unsigned char *best_match_dt_addr = NULL;
#endif

	if (target_is_emmc_boot()) {
		hdr = (struct boot_img_hdr *)EMMC_BOOT_IMG_HEADER_ADDR;
		if (memcmp(hdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
			dprintf(CRITICAL, "ERROR: Invalid boot image header\n");
			return -1;
		}
		goto continue_boot;
	}

	ptable = flash_get_ptable();
	if (ptable == NULL) {
		dprintf(CRITICAL, "ERROR: Partition table not found\n");
		return -1;
	}

	if(!boot_into_recovery)
	{
	        ptn = ptable_find(ptable, "boot");

	        if (ptn == NULL) {
		        dprintf(CRITICAL, "ERROR: No boot partition found\n");
		        return -1;
	        }
	}
	else
	{
	        ptn = ptable_find(ptable, "recovery");
	        if (ptn == NULL) {
		        dprintf(CRITICAL, "ERROR: No recovery partition found\n");
		        return -1;
	        }
	}

	/* Read boot.img header from flash */
	if (flash_read(ptn, offset, buf, page_size)) {
		dprintf(CRITICAL, "ERROR: Cannot read boot image header\n");
		return -1;
	}

	if (memcmp(hdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
		dprintf(CRITICAL, "ERROR: Invalid boot image header\n");
		return -1;
	}

	if (hdr->page_size != page_size) {
		dprintf(CRITICAL, "ERROR: Invalid boot image pagesize. Device pagesize: %d, Image pagesize: %d\n",page_size,hdr->page_size);
		return -1;
	}

	image_addr = (unsigned char *)target_get_scratch_address();
	memcpy(image_addr, (void *)buf, page_size);

	/*
	 * Update the kernel/ramdisk/tags address if the boot image header
	 * has default values, these default values come from mkbootimg when
	 * the boot image is flashed using fastboot flash:raw
	 */
	update_ker_tags_rdisk_addr(hdr, false);

	/* Get virtual addresses since the hdr saves physical addresses. */
	hdr->kernel_addr = VA((addr_t)(hdr->kernel_addr));
	hdr->ramdisk_addr = VA((addr_t)(hdr->ramdisk_addr));
	hdr->tags_addr = VA((addr_t)(hdr->tags_addr));

	kernel_actual  = ROUND_TO_PAGE(hdr->kernel_size,  page_mask);
	ramdisk_actual = ROUND_TO_PAGE(hdr->ramdisk_size, page_mask);
	second_actual = ROUND_TO_PAGE(hdr->second_size, page_mask);

	/* ensure commandline is terminated */
	hdr->cmdline[BOOT_ARGS_SIZE-1] = 0;

	/* Check if the addresses in the header are valid. */
	if (check_aboot_addr_range_overlap(hdr->kernel_addr, kernel_actual) ||
		check_ddr_addr_range_bound(hdr->kernel_addr, kernel_actual) ||
		check_aboot_addr_range_overlap(hdr->ramdisk_addr, ramdisk_actual) ||
		check_ddr_addr_range_bound(hdr->ramdisk_addr, ramdisk_actual))
	{
		dprintf(CRITICAL, "kernel/ramdisk addresses are not valid.\n");
		return -1;
	}

#ifndef DEVICE_TREE
	if (UINT_MAX < ((uint64_t)kernel_actual + (uint64_t)ramdisk_actual+ (uint64_t)second_actual + page_size)) {
		dprintf(CRITICAL, "Integer overflow detected in bootimage header fields\n");
		return -1;
	}
	imagesize_actual = (page_size + kernel_actual + ramdisk_actual + second_actual);

	if (check_aboot_addr_range_overlap(hdr->tags_addr, MAX_TAGS_SIZE) ||
		check_ddr_addr_range_bound(hdr->tags_addr, MAX_TAGS_SIZE))
	{
		dprintf(CRITICAL, "Tags addresses are not valid.\n");
		return -1;
	}
#else

#ifndef OSVERSION_IN_BOOTIMAGE
	dt_size = hdr->dt_size;
#endif
	dt_actual = ROUND_TO_PAGE(dt_size, page_mask);
	if (UINT_MAX < ((uint64_t)kernel_actual + (uint64_t)ramdisk_actual+ (uint64_t)second_actual + (uint64_t)dt_actual + page_size)) {
		dprintf(CRITICAL, "Integer overflow detected in bootimage header fields\n");
		return -1;
	}

	imagesize_actual = (page_size + kernel_actual + ramdisk_actual + second_actual + dt_actual);

	if (check_aboot_addr_range_overlap(hdr->tags_addr, dt_size) ||
		check_ddr_addr_range_bound(hdr->tags_addr, dt_size))
	{
		dprintf(CRITICAL, "Device tree addresses are not valid.\n");
		return -1;
	}
#endif

	/* Read full boot.img from flash */
	dprintf(INFO, "Loading (%s) image (%d): start\n",
		(!boot_into_recovery ? "boot" : "recovery"),imagesize_actual);
	bs_set_timestamp(BS_KERNEL_LOAD_START);

	if (UINT_MAX - page_size < imagesize_actual)
	{
		dprintf(CRITICAL,"Integer overflow detected in bootimage header fields %u %s\n", __LINE__,__func__);
		return -1;
	}

	/*Check the availability of RAM before reading boot image + max signature length from flash*/
	if (target_get_max_flash_size() < (imagesize_actual + page_size))
	{
		dprintf(CRITICAL, "bootimage  size is greater than DDR can hold\n");
		return -1;
	}

	offset = page_size;
	/* Read image without signature and header */
	if (flash_read(ptn, offset, (void *)(image_addr + offset), imagesize_actual - page_size))
	{
		dprintf(CRITICAL, "ERROR: Cannot read boot image\n");
			return -1;
	}

	dprintf(INFO, "Loading (%s) image (%d): done\n",
		(!boot_into_recovery ? "boot" : "recovery"), imagesize_actual);
	bs_set_timestamp(BS_KERNEL_LOAD_DONE);

	offset = page_size;
	if(hdr->second_size != 0) {
		if (UINT_MAX - offset < second_actual)
		{
			dprintf(CRITICAL, "ERROR: Integer overflow in boot image header %s\t%d\n",__func__,__LINE__);
			return -1;
		}
		offset += second_actual;
		/* Second image loading not implemented. */
		ASSERT(0);
	}

	/* Move kernel and ramdisk to correct address */
	memmove((void*) hdr->kernel_addr, (char*) (image_addr + page_size), hdr->kernel_size);
	memmove((void*) hdr->ramdisk_addr, (char*) (image_addr + page_size + kernel_actual), hdr->ramdisk_size);

#if DEVICE_TREE
	if(dt_size != 0) {

		dt_table_offset = ((uint32_t)image_addr + page_size + kernel_actual + ramdisk_actual + second_actual);

		table = (struct dt_table*) dt_table_offset;

		if (dev_tree_validate(table, hdr->page_size, &dt_hdr_size) != 0) {
			dprintf(CRITICAL, "ERROR: Cannot validate Device Tree Table \n");
			return -1;
		}

		/* Its Error if, dt_hdr_size (table->num_entries * dt_entry size + Dev_Tree Header)
		goes beyound hdr->dt_size*/
		if (dt_hdr_size > ROUND_TO_PAGE(dt_size,hdr->page_size)) {
			dprintf(CRITICAL, "ERROR: Invalid Device Tree size \n");
			return -1;
		}

		/* Find index of device tree within device tree table */
		if(dev_tree_get_entry_info(table, &dt_entry) != 0){
			dprintf(CRITICAL, "ERROR: Getting device tree address failed\n");
			return -1;
		}

		/* Validate and Read device device tree in the "tags_add */
		if (check_aboot_addr_range_overlap(hdr->tags_addr, dt_entry.size) ||
			check_ddr_addr_range_bound(hdr->tags_addr, dt_entry.size))
		{
			dprintf(CRITICAL, "Device tree addresses are not valid.\n");
			return -1;
		}

		if(dt_entry.offset > (UINT_MAX - dt_entry.size)) {
			dprintf(CRITICAL, "ERROR: Device tree contents are Invalid\n");
			return -1;
		}

		/* Ensure we are not overshooting dt_size with the dt_entry selected */
		if ((dt_entry.offset + dt_entry.size) > dt_size) {
			dprintf(CRITICAL, "ERROR: Device tree contents are Invalid\n");
			return -1;
		}

		best_match_dt_addr = (unsigned char *)table + dt_entry.offset;
		dtb_size = dt_entry.size;
		memmove((void *)hdr->tags_addr, (char *)best_match_dt_addr, dtb_size);
	}
#endif

continue_boot:

	/* TODO: create/pass atags to kernel */

	boot_linux((void *)hdr->kernel_addr, (void *)hdr->tags_addr,
		   (const char *)hdr->cmdline, board_machtype(),
		   (void *)hdr->ramdisk_addr, hdr->ramdisk_size);

	return 0;
}

void write_device_info_mmc(device_info *dev)
{
	unsigned long long ptn = 0;
	unsigned long long size;
	int index = INVALID_PTN;
	uint32_t blocksize;
	uint8_t lun = 0;
	uint32_t ret = 0;

	if (devinfo_present)
		index = partition_get_index("devinfo");
	else
		index = partition_get_index("aboot");

	ptn = partition_get_offset(index);
	if(ptn == 0)
	{
		return;
	}

	lun = partition_get_lun(index);
	mmc_set_lun(lun);

	size = partition_get_size(index);

	blocksize = mmc_get_device_blocksize();

	if (devinfo_present)
		ret = mmc_write(ptn, blocksize, (void *)info_buf);
	else
		ret = mmc_write((ptn + size - blocksize), blocksize, (void *)info_buf);
	if (ret)
	{
		dprintf(CRITICAL, "ERROR: Cannot write device info\n");
		ASSERT(0);
	}
}

void read_device_info_mmc(struct device_info *info)
{
	unsigned long long ptn = 0;
	unsigned long long size;
	int index = INVALID_PTN;
	uint32_t blocksize;
	uint32_t ret  = 0;

	if ((index = partition_get_index("devinfo")) < 0)
	{
		devinfo_present = false;
		index = partition_get_index("aboot");
	}

	ptn = partition_get_offset(index);
	if(ptn == 0)
	{
		return;
	}

	mmc_set_lun(partition_get_lun(index));

	size = partition_get_size(index);

	blocksize = mmc_get_device_blocksize();

	if (devinfo_present)
		ret = mmc_read(ptn, (void *)info_buf, blocksize);
	else
		ret = mmc_read((ptn + size - blocksize), (void *)info_buf, blocksize);
	if (ret)
	{
		dprintf(CRITICAL, "ERROR: Cannot read device info\n");
		ASSERT(0);
	}
}

void write_device_info_flash(device_info *dev)
{
	struct device_info *info = memalign(PAGE_SIZE, ROUNDUP(BOOT_IMG_MAX_PAGE_SIZE, PAGE_SIZE));
	struct ptentry *ptn;
	struct ptable *ptable;
	if(info == NULL)
	{
		dprintf(CRITICAL, "Failed to allocate memory for device info struct\n");
		ASSERT(0);
	}
	info_buf = info;
	ptable = flash_get_ptable();
	if (ptable == NULL)
	{
		dprintf(CRITICAL, "ERROR: Partition table not found\n");
		return;
	}

	ptn = ptable_find(ptable, "devinfo");
	if (ptn == NULL)
	{
		dprintf(CRITICAL, "ERROR: No devinfo partition found\n");
			return;
	}

	memset(info, 0, BOOT_IMG_MAX_PAGE_SIZE);
	memcpy(info, dev, sizeof(device_info));

	if (flash_write(ptn, 0, (void *)info_buf, page_size))
	{
		dprintf(CRITICAL, "ERROR: Cannot write device info\n");
			return;
	}
	free(info);
}

void read_device_info_flash(device_info *dev)
{
	struct device_info *info = memalign(PAGE_SIZE, ROUNDUP(BOOT_IMG_MAX_PAGE_SIZE, PAGE_SIZE));
	struct ptentry *ptn;
	struct ptable *ptable;
	if(info == NULL)
	{
		dprintf(CRITICAL, "Failed to allocate memory for device info struct\n");
		ASSERT(0);
	}
	info_buf = info;
	ptable = flash_get_ptable();
	if (ptable == NULL)
	{
		dprintf(CRITICAL, "ERROR: Partition table not found\n");
		return;
	}

	ptn = ptable_find(ptable, "devinfo");
	if (ptn == NULL)
	{
		dprintf(CRITICAL, "ERROR: No devinfo partition found\n");
			return;
	}

	if (flash_read(ptn, 0, (void *)info_buf, page_size))
	{
		dprintf(CRITICAL, "ERROR: Cannot write device info\n");
			return;
	}

	if (memcmp(info->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE))
	{
		memcpy(info->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE);
		write_device_info_flash(info);
	}
	memcpy(dev, info, sizeof(device_info));
	free(info);
}

void write_device_info(device_info *dev)
{
	if(target_is_emmc_boot())
	{
		struct device_info *info = memalign(PAGE_SIZE, ROUNDUP(BOOT_IMG_MAX_PAGE_SIZE, PAGE_SIZE));
		if(info == NULL)
		{
			dprintf(CRITICAL, "Failed to allocate memory for device info struct\n");
			ASSERT(0);
		}
		info_buf = info;
		memcpy(info, dev, sizeof(struct device_info));

#if USE_RPMB_FOR_DEVINFO
		if (is_secure_boot_enable()) {
			if((write_device_info_rpmb((void*) info, PAGE_SIZE)) < 0)
				ASSERT(0);
		}
		else
			write_device_info_mmc(info);
#else
		write_device_info_mmc(info);
#endif
		free(info);
	}
	else
	{
		write_device_info_flash(dev);
	}
}

void read_device_info(device_info *dev)
{
	if(target_is_emmc_boot())
	{
		struct device_info *info = memalign(PAGE_SIZE, ROUNDUP(BOOT_IMG_MAX_PAGE_SIZE, PAGE_SIZE));
		if(info == NULL)
		{
			dprintf(CRITICAL, "Failed to allocate memory for device info struct\n");
			ASSERT(0);
		}
		info_buf = info;

#if USE_RPMB_FOR_DEVINFO
		if (is_secure_boot_enable()) {
			if((read_device_info_rpmb((void*) info, PAGE_SIZE)) < 0)
				ASSERT(0);
		}
		else
			read_device_info_mmc(info);
#else
		read_device_info_mmc(info);
#endif

		if (memcmp(info->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE))
		{
			memcpy(info->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE);
			info->charger_screen_enabled = 1;
			write_device_info(info);
		}
		memcpy(dev, info, sizeof(device_info));
		free(info);
	}
	else
	{
		read_device_info_flash(dev);
	}
}

#if DEVICE_TREE
int copy_dtb(uint8_t *boot_image_start, unsigned int scratch_offset)
{
	uint32 dt_image_offset = 0;
	uint32_t n;
	struct dt_table *table;
	struct dt_entry dt_entry;
	uint32_t dt_hdr_size;
	unsigned int compressed_size = 0;
	unsigned int dtb_size = 0;
	unsigned int out_avai_len = 0;
	unsigned char *out_addr = NULL;
	unsigned char *best_match_dt_addr = NULL;
	int rc;

	struct boot_img_hdr *hdr = (struct boot_img_hdr *) (boot_image_start);

#ifndef OSVERSION_IN_BOOTIMAGE
	dt_size = hdr->dt_size;
#endif

	if(dt_size != 0) {
		/* add kernel offset */
		dt_image_offset += page_size;
		n = ROUND_TO_PAGE(hdr->kernel_size, page_mask);
		dt_image_offset += n;

		/* add ramdisk offset */
		n = ROUND_TO_PAGE(hdr->ramdisk_size, page_mask);
		dt_image_offset += n;

		/* add second offset */
		if(hdr->second_size != 0) {
			n = ROUND_TO_PAGE(hdr->second_size, page_mask);
			dt_image_offset += n;
		}

		/* offset now point to start of dt.img */
		table = (struct dt_table*)(boot_image_start + dt_image_offset);

		if (dev_tree_validate(table, hdr->page_size, &dt_hdr_size) != 0) {
			dprintf(CRITICAL, "ERROR: Cannot validate Device Tree Table \n");
			return -1;
		}

		/* Its Error if, dt_hdr_size (table->num_entries * dt_entry size + Dev_Tree Header)
		goes beyound hdr->dt_size*/
		if (dt_hdr_size > ROUND_TO_PAGE(dt_size,hdr->page_size)) {
			dprintf(CRITICAL, "ERROR: Invalid Device Tree size \n");
			return -1;
		}

		/* Find index of device tree within device tree table */
		if(dev_tree_get_entry_info(table, &dt_entry) != 0){
			dprintf(CRITICAL, "ERROR: Getting device tree address failed\n");
			return -1;
		}

		best_match_dt_addr = (unsigned char *)boot_image_start + dt_image_offset + dt_entry.offset;
		if (is_gzip_package(best_match_dt_addr, dt_entry.size))
		{
			out_addr = (unsigned char *)target_get_scratch_address() + scratch_offset;
			out_avai_len = target_get_max_flash_size() - scratch_offset;
			dprintf(INFO, "decompressing dtb: start\n");
			rc = decompress(best_match_dt_addr,
					dt_entry.size, out_addr, out_avai_len,
					&compressed_size, &dtb_size);
			if (rc)
			{
				dprintf(CRITICAL, "decompressing dtb failed!!!\n");
				ASSERT(0);
			}

			dprintf(INFO, "decompressing dtb: done\n");
			best_match_dt_addr = out_addr;
		} else {
			dtb_size = dt_entry.size;
		}
		/* Validate and Read device device tree in the "tags_add */
		if (check_aboot_addr_range_overlap(hdr->tags_addr, dtb_size) ||
			check_ddr_addr_range_bound(hdr->tags_addr, dtb_size))
		{
			dprintf(CRITICAL, "Device tree addresses are not valid.\n");
			return -1;
		}

		/* Read device device tree in the "tags_add */
		memmove((void*) hdr->tags_addr, (void *)best_match_dt_addr, dtb_size);
	} else
		return -1;

	/* Everything looks fine. Return success. */
	return 0;
}
#endif

void cmd_boot(const char *arg, void *data, unsigned sz)
{
#ifdef MDTP_SUPPORT
	static bool is_mdtp_activated = 0;
#endif /* MDTP_SUPPORT */
	unsigned kernel_actual;
	unsigned ramdisk_actual;
	unsigned second_actual;
	uint32_t image_actual;
	uint32_t dt_actual = 0;
	uint32_t sig_actual = 0;
	uint32_t sig_size = 0;
	struct boot_img_hdr *hdr = NULL;
	struct kernel64_hdr *kptr = NULL;
	char *ptr = ((char*) data);
	int ret = 0;
	uint8_t dtb_copied = 0;
	unsigned int out_len = 0;
	unsigned int out_avai_len = 0;
	unsigned char *out_addr = NULL;
	uint32_t dtb_offset = 0;
	unsigned char *kernel_start_addr = NULL;
	unsigned int kernel_size = 0;
	unsigned int scratch_offset = 0;

#if FBCON_DISPLAY_MSG
	/* Exit keys' detection thread firstly */
	exit_menu_keys_detection();
#endif

	if (sz < sizeof(hdr)) {
		fastboot_fail("invalid bootimage header");
		goto boot_failed;
	}

	hdr = (struct boot_img_hdr *)data;

	/* ensure commandline is terminated */
	hdr->cmdline[BOOT_ARGS_SIZE-1] = 0;

	if(target_is_emmc_boot() && hdr->page_size) {
		page_size = hdr->page_size;
		page_mask = page_size - 1;
	}

	kernel_actual = ROUND_TO_PAGE(hdr->kernel_size, page_mask);
	ramdisk_actual = ROUND_TO_PAGE(hdr->ramdisk_size, page_mask);
	second_actual = ROUND_TO_PAGE(hdr->second_size, page_mask);
#if DEVICE_TREE
#ifndef OSVERSION_IN_BOOTIMAGE
	dt_size = hdr->dt_size;
#endif
	dt_actual = ROUND_TO_PAGE(dt_size, page_mask);
#endif

	image_actual = ADD_OF(page_size, kernel_actual);
	image_actual = ADD_OF(image_actual, ramdisk_actual);
	image_actual = ADD_OF(image_actual, second_actual);
	image_actual = ADD_OF(image_actual, dt_actual);

	/* Checking to prevent oob access in read_der_message_length */
	if (image_actual > sz) {
		fastboot_fail("bootimage header fields are invalid");
		goto boot_failed;
	}
	sig_size = sz - image_actual;

	/* Handle overflow if the input image size is greater than
	 * boot image buffer can hold
	 */
	if ((target_get_max_flash_size() - (image_actual - sig_actual)) < page_size)
	{
		fastboot_fail("booimage: size is greater than boot image buffer can hold");
		goto boot_failed;
	}
#ifdef MDTP_SUPPORT
	else
	{
		/* fastboot boot is not allowed when MDTP is activated */
		mdtp_ext_partition_verification_t ext_partition;

		if (!is_mdtp_activated) {
			ext_partition.partition = MDTP_PARTITION_NONE;
			mdtp_fwlock_verify_lock(&ext_partition);
		}
	}

	/* If mdtp state cannot be validate, block fastboot boot*/
	if(mdtp_activated(&is_mdtp_activated)){
		dprintf(CRITICAL, "mdtp_activated cannot validate state.\n");
		dprintf(CRITICAL, "Can not proceed with fastboot boot command.\n");
		goto boot_failed;
	}
	if(is_mdtp_activated){
		dprintf(CRITICAL, "fastboot boot command is not available.\n");
		goto boot_failed;
	}
#endif /* MDTP_SUPPORT */

	/*
	 * Check if the kernel image is a gzip package. If yes, need to decompress it.
	 * If not, continue booting.
	 */
	if (is_gzip_package((unsigned char *)(data + page_size), hdr->kernel_size))
	{
		out_addr = (unsigned char *)target_get_scratch_address();
		out_addr = (unsigned char *)(out_addr + image_actual + page_size);
		out_avai_len = target_get_max_flash_size() - image_actual - page_size;
		dprintf(INFO, "decompressing kernel image: start\n");
		ret = decompress((unsigned char *)(ptr + page_size),
				hdr->kernel_size, out_addr, out_avai_len,
				&dtb_offset, &out_len);
		if (ret)
		{
			dprintf(CRITICAL, "decompressing image failed!!!\n");
			ASSERT(0);
		}

		dprintf(INFO, "decompressing kernel image: done\n");
		kptr = (struct kernel64_hdr *)out_addr;
		kernel_start_addr = out_addr;
		kernel_size = out_len;
	} else {
		kptr = (struct kernel64_hdr*)((char *)data + page_size);
		kernel_start_addr = (unsigned char *)((char *)data + page_size);
		kernel_size = hdr->kernel_size;
	}

	/*
	 * Update the kernel/ramdisk/tags address if the boot image header
	 * has default values, these default values come from mkbootimg when
	 * the boot image is flashed using fastboot flash:raw
	 */
	update_ker_tags_rdisk_addr(hdr, IS_ARM64(kptr));

	/* Get virtual addresses since the hdr saves physical addresses. */
	hdr->kernel_addr = VA(hdr->kernel_addr);
	hdr->ramdisk_addr = VA(hdr->ramdisk_addr);
	hdr->tags_addr = VA(hdr->tags_addr);

	kernel_size  = ROUND_TO_PAGE(kernel_size,  page_mask);
	/* Check if the addresses in the header are valid. */
	if (check_aboot_addr_range_overlap(hdr->kernel_addr, kernel_size) ||
		check_ddr_addr_range_bound(hdr->kernel_addr, kernel_size) ||
		check_aboot_addr_range_overlap(hdr->ramdisk_addr, ramdisk_actual) ||
		check_ddr_addr_range_bound(hdr->ramdisk_addr, ramdisk_actual))
	{
		dprintf(CRITICAL, "kernel/ramdisk addresses are not valid.\n");
		goto boot_failed;
	}

#if DEVICE_TREE
	scratch_offset = image_actual + page_size + out_len;
	/* find correct dtb and copy it to right location */
	ret = copy_dtb(data, scratch_offset);

	dtb_copied = !ret ? 1 : 0;
#else
	if (check_aboot_addr_range_overlap(hdr->tags_addr, MAX_TAGS_SIZE) ||
		check_ddr_addr_range_bound(hdr->tags_addr, MAX_TAGS_SIZE))
	{
		dprintf(CRITICAL, "Tags addresses are not valid.\n");
		goto boot_failed;
	}
#endif

	/* Load ramdisk & kernel */
	memmove((void*) hdr->ramdisk_addr, ptr + page_size + kernel_actual, hdr->ramdisk_size);
	memmove((void*) hdr->kernel_addr, (char*) (kernel_start_addr), kernel_size);

#if DEVICE_TREE
	if (check_aboot_addr_range_overlap(hdr->tags_addr, kernel_actual) ||
		check_ddr_addr_range_bound(hdr->tags_addr, kernel_actual))
	{
		dprintf(CRITICAL, "Tags addresses are not valid.\n");
		goto boot_failed;
	}

	/*
	 * If dtb is not found look for appended DTB in the kernel.
	 * If appended dev tree is found, update the atags with
	 * memory address to the DTB appended location on RAM.
	 * Else update with the atags address in the kernel header
	 */
	if (!dtb_copied) {
		void *dtb;
		dtb = dev_tree_appended((void*)(ptr + page_size),
					hdr->kernel_size, dtb_offset,
					(void *)hdr->tags_addr);
		if (!dtb) {
			fastboot_fail("dtb not found");
			goto boot_failed;
		}
	}
#endif

	fastboot_okay("");
	fastboot_stop();

	boot_linux((void*) hdr->kernel_addr, (void*) hdr->tags_addr,
		   (const char*) hdr->cmdline, board_machtype(),
		   (void*) hdr->ramdisk_addr, hdr->ramdisk_size);

	/* fastboot already stop, it's no need to show fastboot menu */
	return;
boot_failed:
#if FBCON_DISPLAY_MSG
	/* revert to fastboot menu if boot failed */
	display_fastboot_menu();
#endif
	return;
}

void cmd_erase_nand(const char *arg, void *data, unsigned sz)
{
	struct ptentry *ptn;
	struct ptable *ptable;

	ptable = flash_get_ptable();
	if (ptable == NULL) {
		fastboot_fail("partition table doesn't exist");
		return;
	}

	ptn = ptable_find(ptable, arg);
	if (ptn == NULL) {
		fastboot_fail("unknown partition name");
		return;
	}

	if (flash_erase(ptn)) {
		fastboot_fail("failed to erase partition");
		return;
	}
	fastboot_okay("");
}


void cmd_erase_mmc(const char *arg, void *data, unsigned sz)
{
	unsigned long long ptn = 0;
	unsigned long long size = 0;
	int index = INVALID_PTN;
	uint8_t lun = 0;
	char *footer = NULL;

	index = partition_get_index(arg);
	ptn = partition_get_offset(index);
	size = partition_get_size(index);

	if(ptn == 0) {
		fastboot_fail("Partition table doesn't exist\n");
		return;
	}

	lun = partition_get_lun(index);
	mmc_set_lun(lun);

	if (platform_boot_dev_isemmc())
	{
		if (mmc_erase_card(ptn, size)) {
			fastboot_fail("failed to erase partition\n");
			return;
		}
	} else {
		BUF_DMA_ALIGN(out, DEFAULT_ERASE_SIZE);
		size = partition_get_size(index);
		if (size > DEFAULT_ERASE_SIZE)
			size = DEFAULT_ERASE_SIZE;

		/* Simple inefficient version of erase. Just writing
	       0 in first several blocks */
		if (mmc_write(ptn , size, (unsigned int *)out)) {
			fastboot_fail("failed to erase partition");
			return;
		}
		/*Erase FDE metadata at the userdata footer*/
		if(!(strncmp(arg, "userdata", 8)))
		{
			footer = memalign(CACHE_LINE, FOOTER_SIZE);
			memset((void *)footer, 0, FOOTER_SIZE);

			size = partition_get_size(index);

			if (mmc_write((ptn + size) - FOOTER_SIZE , FOOTER_SIZE, (unsigned int *)footer)) {
				fastboot_fail("failed to erase userdata footer");
				free(footer);
				return;
			}
			free(footer);
		}
	}

	fastboot_okay("");
}

void cmd_erase(const char *arg, void *data, unsigned sz)
{
	if(target_is_emmc_boot())
		cmd_erase_mmc(arg, data, sz);
	else
		cmd_erase_nand(arg, data, sz);
}

void cmd_flash_mmc_img(const char *arg, void *data, unsigned sz)
{
	unsigned long long ptn = 0;
	unsigned long long size = 0;
	int index = INVALID_PTN;
	char *token = NULL;
	char *pname = NULL;
	char *sp;
	uint8_t lun = 0;
	bool lun_set = false;
	int current_active_slot = INVALID;

	token = strtok_r((char *)arg, ":", &sp);
	pname = token;
	token = strtok_r(NULL, ":", &sp);
	if(token)
	{
		lun = atoi(token);
		mmc_set_lun(lun);
		lun_set = true;
	}

	if (pname)
	{
		if (!strcmp(pname, "partition"))
		{
			dprintf(INFO, "Attempt to write partition image.\n");
			if (write_partition(sz, (unsigned char *) data)) {
				fastboot_fail("failed to write partition");
				return;
			}

			/* Rescan partition table to ensure we have multislot support*/
			if (partition_scan_for_multislot())
			{
				current_active_slot = partition_find_active_slot();
				dprintf(INFO, "Multislot supported: Slot %s active",
					(SUFFIX_SLOT(current_active_slot)));
			}
			partition_mark_active_slot(current_active_slot);
		}
		else
		{
			index = partition_get_index(pname);
			ptn = partition_get_offset(index);
			if(ptn == 0) {
				fastboot_fail("partition table doesn't exist");
				return;
			}

			if (!strncmp(pname, "boot", strlen("boot"))
					|| !strcmp(pname, "recovery"))
			{
				if (memcmp((void *)data, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
					fastboot_fail("image is not a boot image");
					return;
				}

				/* Reset multislot_partition attributes in case of flashing boot */
				if (partition_multislot_is_supported())
				{
					partition_reset_attributes(index);
				}
			}

			if(!lun_set)
			{
				lun = partition_get_lun(index);
				mmc_set_lun(lun);
			}

			size = partition_get_size(index);
			if (ROUND_TO_PAGE(sz, mmc_blocksize_mask) > size) {
				fastboot_fail("size too large");
				return;
			}
			else if (mmc_write(ptn , sz, (unsigned int *)data)) {
				fastboot_fail("flash write failure");
				return;
			}
		}
	}
	fastboot_okay("");
	return;
}

void cmd_flash_mmc_sparse_img(const char *arg, void *data, unsigned sz)
{
	unsigned int chunk;
	uint64_t chunk_data_sz;
	uint32_t *fill_buf = NULL;
	uint32_t fill_val;
	uint32_t blk_sz_actual = 0;
	sparse_header_t *sparse_header;
	chunk_header_t *chunk_header;
	uint32_t total_blocks = 0;
	unsigned long long ptn = 0;
	unsigned long long size = 0;
	int index = INVALID_PTN;
	uint32_t i;
	uint8_t lun = 0;
	/*End of the sparse image address*/
	uintptr_t data_end = (uintptr_t)data + sz;

	index = partition_get_index(arg);
	ptn = partition_get_offset(index);
	if(ptn == 0) {
		fastboot_fail("partition table doesn't exist");
		return;
	}

	size = partition_get_size(index);

	lun = partition_get_lun(index);
	mmc_set_lun(lun);

	if (sz < sizeof(sparse_header_t)) {
		fastboot_fail("size too low");
		return;
	}

	/* Read and skip over sparse image header */
	sparse_header = (sparse_header_t *) data;

	if (!sparse_header->blk_sz || (sparse_header->blk_sz % 4)){
		fastboot_fail("Invalid block size\n");
		return;
	}

	if (((uint64_t)sparse_header->total_blks * (uint64_t)sparse_header->blk_sz) > size) {
		fastboot_fail("size too large");
		return;
	}

	data += sizeof(sparse_header_t);

	if (data_end < (uintptr_t)data) {
		fastboot_fail("buffer overreads occured due to invalid sparse header");
		return;
	}

	if(sparse_header->file_hdr_sz != sizeof(sparse_header_t))
	{
		fastboot_fail("sparse header size mismatch");
		return;
	}

	dprintf (SPEW, "=== Sparse Image Header ===\n");
	dprintf (SPEW, "magic: 0x%x\n", sparse_header->magic);
	dprintf (SPEW, "major_version: 0x%x\n", sparse_header->major_version);
	dprintf (SPEW, "minor_version: 0x%x\n", sparse_header->minor_version);
	dprintf (SPEW, "file_hdr_sz: %d\n", sparse_header->file_hdr_sz);
	dprintf (SPEW, "chunk_hdr_sz: %d\n", sparse_header->chunk_hdr_sz);
	dprintf (SPEW, "blk_sz: %d\n", sparse_header->blk_sz);
	dprintf (SPEW, "total_blks: %d\n", sparse_header->total_blks);
	dprintf (SPEW, "total_chunks: %d\n", sparse_header->total_chunks);

	/* Start processing chunks */
	for (chunk=0; chunk<sparse_header->total_chunks; chunk++)
	{
		/* Make sure the total image size does not exceed the partition size */
		if(((uint64_t)total_blocks * (uint64_t)sparse_header->blk_sz) >= size) {
			fastboot_fail("size too large");
			return;
		}
		/* Read and skip over chunk header */
		chunk_header = (chunk_header_t *) data;
		data += sizeof(chunk_header_t);

		if (data_end < (uintptr_t)data) {
			fastboot_fail("buffer overreads occured due to invalid sparse header");
			return;
		}

		dprintf (SPEW, "=== Chunk Header ===\n");
		dprintf (SPEW, "chunk_type: 0x%x\n", chunk_header->chunk_type);
		dprintf (SPEW, "chunk_data_sz: 0x%x\n", chunk_header->chunk_sz);
		dprintf (SPEW, "total_size: 0x%x\n", chunk_header->total_sz);

		if(sparse_header->chunk_hdr_sz != sizeof(chunk_header_t))
		{
			fastboot_fail("chunk header size mismatch");
			return;
		}

		chunk_data_sz = (uint64_t)sparse_header->blk_sz * chunk_header->chunk_sz;

		/* Make sure that the chunk size calculated from sparse image does not
		 * exceed partition size
		 */
		if ((uint64_t)total_blocks * (uint64_t)sparse_header->blk_sz + chunk_data_sz > size)
		{
			fastboot_fail("Chunk data size exceeds partition size");
			return;
		}

		switch (chunk_header->chunk_type)
		{
			case CHUNK_TYPE_RAW:
			if((uint64_t)chunk_header->total_sz != ((uint64_t)sparse_header->chunk_hdr_sz +
											chunk_data_sz))
			{
				fastboot_fail("Bogus chunk size for chunk type Raw");
				return;
			}

			if (data_end < (uintptr_t)data + chunk_data_sz) {
				fastboot_fail("buffer overreads occured due to invalid sparse header");
				return;
			}

			/* chunk_header->total_sz is uint32,So chunk_data_sz is now less than 2^32
			   otherwise it will return in the line above
			 */
			if(mmc_write(ptn + ((uint64_t)total_blocks*sparse_header->blk_sz),
						(uint32_t)chunk_data_sz,
						(unsigned int*)data))
			{
				fastboot_fail("flash write failure");
				return;
			}
			if(total_blocks > (UINT_MAX - chunk_header->chunk_sz)) {
				fastboot_fail("Bogus size for RAW chunk type");
				return;
			}
			total_blocks += chunk_header->chunk_sz;
			data += (uint32_t)chunk_data_sz;
			break;

			case CHUNK_TYPE_FILL:
			if(chunk_header->total_sz != (sparse_header->chunk_hdr_sz +
											sizeof(uint32_t)))
			{
				fastboot_fail("Bogus chunk size for chunk type FILL");
				return;
			}

			blk_sz_actual = ROUNDUP(sparse_header->blk_sz, CACHE_LINE);
			/* Integer overflow detected */
			if (blk_sz_actual < sparse_header->blk_sz)
			{
				fastboot_fail("Invalid block size");
				return;
			}

			fill_buf = (uint32_t *)memalign(CACHE_LINE, blk_sz_actual);
			if (!fill_buf)
			{
				fastboot_fail("Malloc failed for: CHUNK_TYPE_FILL");
				return;
			}

			if (data_end < (uintptr_t)data + sizeof(uint32_t)) {
				fastboot_fail("buffer overreads occured due to invalid sparse header");
				free(fill_buf);
				return;
			}
			fill_val = *(uint32_t *)data;
			data = (char *) data + sizeof(uint32_t);

			for (i = 0; i < (sparse_header->blk_sz / sizeof(fill_val)); i++)
			{
				fill_buf[i] = fill_val;
			}

			if(total_blocks > (UINT_MAX - chunk_header->chunk_sz))
			{
				fastboot_fail("bogus size for chunk FILL type");
				free(fill_buf);
				return;
			}

			for (i = 0; i < chunk_header->chunk_sz; i++)
			{
				/* Make sure that the data written to partition does not exceed partition size */
				if ((uint64_t)total_blocks * (uint64_t)sparse_header->blk_sz + sparse_header->blk_sz > size)
				{
					fastboot_fail("Chunk data size for fill type exceeds partition size");
					free(fill_buf);
					return;
				}

				if(mmc_write(ptn + ((uint64_t)total_blocks*sparse_header->blk_sz),
							sparse_header->blk_sz,
							fill_buf))
				{
					fastboot_fail("flash write failure");
					free(fill_buf);
					return;
				}

				total_blocks++;
			}

			free(fill_buf);
			break;

			case CHUNK_TYPE_DONT_CARE:
			if(total_blocks > (UINT_MAX - chunk_header->chunk_sz)) {
				fastboot_fail("bogus size for chunk DONT CARE type");
				return;
			}
			total_blocks += chunk_header->chunk_sz;
			break;

			case CHUNK_TYPE_CRC:
			if(chunk_header->total_sz != sparse_header->chunk_hdr_sz)
			{
				fastboot_fail("Bogus chunk size for chunk type CRC");
				return;
			}
			if(total_blocks > (UINT_MAX - chunk_header->chunk_sz)) {
				fastboot_fail("bogus size for chunk CRC type");
				return;
			}
			total_blocks += chunk_header->chunk_sz;
			if ((uintptr_t)data > UINT_MAX - chunk_data_sz) {
				fastboot_fail("integer overflow occured");
				return;
			}
			data += (uint32_t)chunk_data_sz;
			if (data_end < (uintptr_t)data) {
				fastboot_fail("buffer overreads occured due to invalid sparse header");
				return;
			}
			break;

			default:
			dprintf(CRITICAL, "Unkown chunk type: %x\n",chunk_header->chunk_type);
			fastboot_fail("Unknown chunk type");
			return;
		}
	}

	dprintf(INFO, "Wrote %d blocks, expected to write %d blocks\n",
					total_blocks, sparse_header->total_blks);

	if(total_blocks != sparse_header->total_blks)
	{
		fastboot_fail("sparse image write failure");
	}

	fastboot_okay("");
	return;
}

void cmd_flash_mmc(const char *arg, void *data, unsigned sz)
{
	sparse_header_t *sparse_header;

#ifdef SSD_ENABLE
	/* 8 Byte Magic + 2048 Byte xml + Encrypted Data */
	unsigned int *magic_number = (unsigned int *) data;
	int              ret=0;
	uint32           major_version=0;
	uint32           minor_version=0;

	ret = scm_svc_version(&major_version,&minor_version);
	if(!ret)
	{
		if(major_version >= 2)
		{
			if( !strcmp(arg, "ssd") || !strcmp(arg, "tqs") )
			{
				ret = encrypt_scm((uint32 **) &data, &sz);
				if (ret != 0) {
					dprintf(CRITICAL, "ERROR: Encryption Failure\n");
					return;
				}

				/* Protect only for SSD */
				if (!strcmp(arg, "ssd")) {
					ret = scm_protect_keystore((uint32 *) data, sz);
					if (ret != 0) {
						dprintf(CRITICAL, "ERROR: scm_protect_keystore Failed\n");
						return;
					}
				}
			}
			else
			{
				ret = decrypt_scm_v2((uint32 **) &data, &sz);
				if(ret != 0)
				{
					dprintf(CRITICAL,"ERROR: Decryption Failure\n");
					return;
				}
			}
		}
		else
		{
			if (magic_number[0] == DECRYPT_MAGIC_0 &&
			magic_number[1] == DECRYPT_MAGIC_1)
			{
				ret = decrypt_scm((uint32 **) &data, &sz);
				if (ret != 0) {
					dprintf(CRITICAL, "ERROR: Invalid secure image\n");
					return;
				}
			}
			else if (magic_number[0] == ENCRYPT_MAGIC_0 &&
				magic_number[1] == ENCRYPT_MAGIC_1)
			{
				ret = encrypt_scm((uint32 **) &data, &sz);
				if (ret != 0) {
					dprintf(CRITICAL, "ERROR: Encryption Failure\n");
					return;
				}
			}
		}
	}
	else
	{
		dprintf(CRITICAL,"INVALID SVC Version\n");
		return;
	}
#endif /* SSD_ENABLE */

	sparse_header = (sparse_header_t *) data;
	if (sparse_header->magic == SPARSE_HEADER_MAGIC)
		cmd_flash_mmc_sparse_img(arg, data, sz);
	else
		cmd_flash_mmc_img(arg, data, sz);

	return;
}

void cmd_updatevol(const char *vol_name, void *data, unsigned sz)
{
	struct ptentry *sys_ptn;
	struct ptable *ptable;

	ptable = flash_get_ptable();
	if (ptable == NULL) {
		fastboot_fail("partition table doesn't exist");
		return;
	}

	sys_ptn = ptable_find(ptable, "system");
	if (sys_ptn == NULL) {
		fastboot_fail("system partition not found");
		return;
	}

	sz = ROUND_TO_PAGE(sz, page_mask);
	if (update_ubi_vol(sys_ptn, vol_name, data, sz))
		fastboot_fail("update_ubi_vol failed");
	else
		fastboot_okay("");
}

void cmd_flash_nand(const char *arg, void *data, unsigned sz)
{
	struct ptentry *ptn;
	struct ptable *ptable;
	unsigned extra = 0;
	uint64_t partition_size = 0;
	unsigned bytes_to_round_page = 0;
	unsigned rounded_size = 0;

	if((uintptr_t)data > (UINT_MAX - sz)) {
		fastboot_fail("Cannot flash: image header corrupt");
                return;
        }

	ptable = flash_get_ptable();
	if (ptable == NULL) {
		fastboot_fail("partition table doesn't exist");
		return;
	}

	ptn = ptable_find(ptable, arg);
	if (ptn == NULL) {
		dprintf(INFO, "unknown partition name (%s). Trying updatevol\n",
				arg);
		cmd_updatevol(arg, data, sz);
		return;
	}

	if (!strcmp(ptn->name, "boot") || !strcmp(ptn->name, "recovery")) {
		if((sz > BOOT_MAGIC_SIZE) && (!memcmp((void *)data, BOOT_MAGIC, BOOT_MAGIC_SIZE))) {
			dprintf(INFO, "Verified the BOOT_MAGIC in image header  \n");
		} else {
			fastboot_fail("Image is not a boot image");
			return;
		}
	}

	if (!strcmp(ptn->name, "system")
		|| !strcmp(ptn->name, "userdata")
		|| !strcmp(ptn->name, "persist")
		|| !strcmp(ptn->name, "recoveryfs")
		|| !strcmp(ptn->name, "modem"))
		extra = 1;
	else {
		rounded_size = ROUNDUP(sz, page_size);
		bytes_to_round_page = rounded_size - sz;
		if (bytes_to_round_page) {
			if (((uintptr_t)data + sz ) > (UINT_MAX - bytes_to_round_page)) {
				fastboot_fail("Integer overflow detected");
				return;
			}
			if (((uintptr_t)data + sz + bytes_to_round_page) >
				((uintptr_t)target_get_scratch_address() + target_get_max_flash_size())) {
				fastboot_fail("Buffer size is not aligned to page_size");
				return;
			}
			else {
				memset(data + sz, 0, bytes_to_round_page);
				sz = rounded_size;
			}
		}
	}

	/*Checking partition_size for the possible integer overflow */
	partition_size = validate_partition_size(ptn);

	if (sz > partition_size) {
		fastboot_fail("Image size too large");
		return;
	}

	dprintf(INFO, "writing %d bytes to '%s'\n", sz, ptn->name);
	if ((sz > UBI_EC_HDR_SIZE) &&
		(!memcmp((void *)data, UBI_MAGIC, UBI_MAGIC_SIZE))) {
		if (flash_ubi_img(ptn, data, sz)) {
			fastboot_fail("flash write failure");
			return;
		}
	} else {
		if (flash_write(ptn, extra, data, sz)) {
			fastboot_fail("flash write failure");
			return;
		}
	}
	dprintf(INFO, "partition '%s' updated\n", ptn->name);
	fastboot_okay("");
}


static inline uint64_t validate_partition_size(struct ptentry *ptn)
{
	if (ptn->length && flash_num_pages_per_blk() && page_size) {
		if ((ptn->length < ( UINT_MAX / flash_num_pages_per_blk())) && ((ptn->length * flash_num_pages_per_blk()) < ( UINT_MAX / page_size))) {
			return ptn->length * flash_num_pages_per_blk() * page_size;
		}
        }
	return 0;
}


void cmd_flash(const char *arg, void *data, unsigned sz)
{
	if(target_is_emmc_boot())
		cmd_flash_mmc(arg, data, sz);
	else
		cmd_flash_nand(arg, data, sz);
}

void cmd_continue(const char *arg, void *data, unsigned sz)
{
	fastboot_okay("");
	fastboot_stop();

	if (target_is_emmc_boot())
	{
#if FBCON_DISPLAY_MSG
		/* Exit keys' detection thread firstly */
		exit_menu_keys_detection();
#endif
		boot_linux_from_mmc();
	}
	else
	{
		boot_linux_from_flash();
	}
}

void cmd_reboot(const char *arg, void *data, unsigned sz)
{
	dprintf(INFO, "rebooting the device\n");
	fastboot_okay("");
	reboot_device(0);
}

void cmd_set_active(const char *arg, void *data, unsigned sz)
{
	char *p, *sp = NULL;
	unsigned i,current_active_slot;
	const char *current_slot_suffix;

	if (!partition_multislot_is_supported())
	{
		fastboot_fail("Command not supported");
		return;
	}

	if (arg)
	{
		p = strtok_r((char *)arg, ":", &sp);
		if (p)
		{
			current_active_slot = partition_find_active_slot();

			/* Check if trying to make curent slot active */
			current_slot_suffix = SUFFIX_SLOT(current_active_slot);
			current_slot_suffix = strtok_r((char *)current_slot_suffix,
							(char *)suffix_delimiter, &sp);

			if (current_slot_suffix &&
				!strncmp(p, current_slot_suffix, strlen(current_slot_suffix)))
			{
				fastboot_okay("Slot already set active");
				return;
			}
			else
			{
				for (i = 0; i < AB_SUPPORTED_SLOTS; i++)
				{
					current_slot_suffix = SUFFIX_SLOT(i);
					current_slot_suffix = strtok_r((char *)current_slot_suffix,
									(char *)suffix_delimiter, &sp);
					if (current_slot_suffix &&
						!strncmp(p, current_slot_suffix, strlen(current_slot_suffix)))
					{
						partition_switch_slots(current_active_slot, i);
						publish_getvar_multislot_vars();
						fastboot_okay("");
						return;
					}
				}
			}
		}
	}
	fastboot_fail("Invalid slot suffix.");
	return;
}

void cmd_reboot_bootloader(const char *arg, void *data, unsigned sz)
{
	dprintf(INFO, "rebooting the device\n");
	fastboot_okay("");
	reboot_device(FASTBOOT_MODE);
}

void cmd_oem_enable_charger_screen(const char *arg, void *data, unsigned size)
{
	dprintf(INFO, "Enabling charger screen check\n");
	device.charger_screen_enabled = 1;
	write_device_info(&device);
	fastboot_okay("");
}

void cmd_oem_disable_charger_screen(const char *arg, void *data, unsigned size)
{
	dprintf(INFO, "Disabling charger screen check\n");
	device.charger_screen_enabled = 0;
	write_device_info(&device);
	fastboot_okay("");
}

void cmd_oem_off_mode_charger(const char *arg, void *data, unsigned size)
{
	char *p = NULL;
	const char *delim = " \t\n\r";
	char *sp;

	if (arg) {
		p = strtok_r((char *)arg, delim, &sp);
		if (p) {
			if (!strncmp(p, "0", 1)) {
				device.charger_screen_enabled = 0;
			} else if (!strncmp(p, "1", 1)) {
				device.charger_screen_enabled = 1;
			}
		}
	}

	/* update charger_screen_enabled value for getvar
	 * command
	 */
	snprintf(charger_screen_enabled, MAX_RSP_SIZE, "%d",
		device.charger_screen_enabled);

	write_device_info(&device);
	fastboot_okay("");
}

void cmd_oem_select_display_panel(const char *arg, void *data, unsigned size)
{
	dprintf(INFO, "Selecting display panel %s\n", arg);
	if (arg)
		strlcpy(device.display_panel, arg,
			sizeof(device.display_panel));
	write_device_info(&device);
	fastboot_okay("");
}

void cmd_oem_devinfo(const char *arg, void *data, unsigned sz)
{
	char response[MAX_RSP_SIZE];
	snprintf(response, sizeof(response), "\tCharger screen enabled: %s", (device.charger_screen_enabled ? "true" : "false"));
	fastboot_info(response);
	snprintf(response, sizeof(response), "\tDisplay panel: %s", (device.display_panel));
	fastboot_info(response);
	fastboot_okay("");
}

void fastboot_info_buffer(const char *str)
{
	char *workstr = strdup(str);
	char *pch = strtok(workstr, "\n\r");
	while (pch != NULL)
	{
		char *ptr = pch;
		while (ptr != NULL)
		{
			fastboot_info(ptr);
			if (strlen(ptr) > MAX_RSP_SIZE - 5)
				ptr += MAX_RSP_SIZE - 5;
			else
				ptr = NULL;
		}

		pch = strtok(NULL, "\n\r");
	}
	free(workstr);
}

void cmd_oem_bootlog(const char *arg, void *data, unsigned sz)
{
	const char *bbss_log = bbry_hwi_get_entry("bbss_log", 0);
	if (bbss_log)
		fastboot_info_buffer(bbss_log);

	const char *sbl_boot_log_buffer = bbry_hwi_get_entry("sbl_boot_log_buffer", 0);
	if (sbl_boot_log_buffer)
		fastboot_info_buffer(sbl_boot_log_buffer);

#if WITH_DEBUG_LOG_BUF
	fastboot_info_buffer(lk_log_getbuf());
#else
	fastboot_info("logbuf disabled");
#endif

	fastboot_okay("");
}

void cmd_preflash(const char *arg, void *data, unsigned sz)
{
	fastboot_okay("");
}

static uint8_t logo_header[LOGO_IMG_HEADER_SIZE];

int splash_screen_check_header(logo_img_header *header)
{
	if (memcmp(header->magic, LOGO_IMG_MAGIC, 8))
		return -1;
	if (header->width == 0 || header->height == 0)
		return -1;
	return 0;
}

int splash_screen_flash()
{
	struct ptentry *ptn;
	struct ptable *ptable;
	struct logo_img_header *header;
	struct fbcon_config *fb_display = NULL;

	ptable = flash_get_ptable();
	if (ptable == NULL) {
		dprintf(CRITICAL, "ERROR: Partition table not found\n");
		return -1;
	}

	ptn = ptable_find(ptable, "splash");
	if (ptn == NULL) {
		dprintf(CRITICAL, "ERROR: splash Partition not found\n");
		return -1;
	}
	if (flash_read(ptn, 0, (void *)logo_header, LOGO_IMG_HEADER_SIZE)) {
		dprintf(CRITICAL, "ERROR: Cannot read boot image header\n");
		return -1;
	}

	header = (struct logo_img_header *)logo_header;
	if (splash_screen_check_header(header)) {
		dprintf(CRITICAL, "ERROR: Boot image header invalid\n");
		return -1;
	}

	fb_display = fbcon_display();
	if (fb_display) {
		if (header->type && (header->blocks != 0) &&
				(UINT_MAX >= header->blocks * 512) &&
				((header->blocks * 512) <=  (fb_display->width *
				fb_display->height * (fb_display->bpp / 8)))) {
					/* RLE24 compressed data */
			uint8_t *base = (uint8_t *) fb_display->base + LOGO_IMG_OFFSET;

			/* if the logo is full-screen size, remove "fbcon_clear()" */
			if ((header->width != fb_display->width)
						|| (header->height != fb_display->height))
					fbcon_clear();

			if (flash_read(ptn + LOGO_IMG_HEADER_SIZE, 0,
				(uint32_t *)base,
				(header->blocks * 512))) {
				dprintf(CRITICAL, "ERROR: Cannot read splash image from partition\n");
				return -1;
			}
			fbcon_extract_to_screen(header, base);
			return 0;
		}

		if ((header->width > fb_display->width) || (header->height > fb_display->height)) {
			dprintf(CRITICAL, "Logo config greater than fb config. Fall back default logo\n");
			return -1;
		}

		uint8_t *base = (uint8_t *) fb_display->base;
		uint32_t fb_size = ROUNDUP(fb_display->width *
					fb_display->height *
					(fb_display->bpp / 8), 4096);
		uint32_t splash_size = ((((header->width * header->height *
					fb_display->bpp/8) + 511) >> 9) << 9);

		if (splash_size > fb_size) {
			dprintf(CRITICAL, "ERROR: Splash image size invalid\n");
			return -1;
		}

		if (flash_read(ptn + LOGO_IMG_HEADER_SIZE, 0,
			(uint32_t *)base,
			((((header->width * header->height * fb_display->bpp/8) + 511) >> 9) << 9))) {
			fbcon_clear();
			dprintf(CRITICAL, "ERROR: Cannot read splash image from partition\n");
			return -1;
		}
	}

	return 0;
}

int splash_screen_mmc()
{
	int index = INVALID_PTN;
	unsigned long long ptn = 0;
	struct fbcon_config *fb_display = NULL;
	struct logo_img_header *header;
	uint32_t blocksize, realsize, readsize;
	uint8_t *base;

	index = partition_get_index("splash");
	if (index == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition table not found\n");
		return -1;
	}

	ptn = partition_get_offset(index);
	if (ptn == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition invalid\n");
		return -1;
	}

	mmc_set_lun(partition_get_lun(index));

	blocksize = mmc_get_device_blocksize();
	if (blocksize == 0) {
		dprintf(CRITICAL, "ERROR:splash Partition invalid blocksize\n");
		return -1;
	}

	fb_display = fbcon_display();
	if (!fb_display)
	{
		dprintf(CRITICAL, "ERROR: fb config is not allocated\n");
		return -1;
	}

	base = (uint8_t *) fb_display->base;

	if (mmc_read(ptn, (uint32_t *)(base + LOGO_IMG_OFFSET), blocksize)) {
		dprintf(CRITICAL, "ERROR: Cannot read splash image header\n");
		return -1;
	}

	header = (struct logo_img_header *)(base + LOGO_IMG_OFFSET);
	if (splash_screen_check_header(header)) {
		dprintf(CRITICAL, "ERROR: Splash image header invalid\n");
		return -1;
	}

	if (fb_display) {
		if (header->type && (header->blocks != 0) &&
			(UINT_MAX >= header->blocks * 512 + LOGO_IMG_HEADER_SIZE) &&
			((header->blocks * 512) <=  (fb_display->width *
			fb_display->height * (fb_display->bpp / 8)))) {
			/* 1 RLE24 compressed data */
			base += LOGO_IMG_OFFSET;

			realsize =  header->blocks * 512;
			readsize =  ROUNDUP((realsize + LOGO_IMG_HEADER_SIZE), blocksize) - blocksize;

			/* if the logo is not full-screen size, clean screen */
			if ((header->width != fb_display->width)
						|| (header->height != fb_display->height))
				fbcon_clear();

			uint32_t fb_size = ROUNDUP(fb_display->width *
					fb_display->height *
					(fb_display->bpp / 8), 4096);

			if (readsize > fb_size) {
				dprintf(CRITICAL, "ERROR: Splash image size invalid\n");
				return -1;
			}

			if (mmc_read(ptn + blocksize, (uint32_t *)(base + blocksize), readsize)) {
				dprintf(CRITICAL, "ERROR: Cannot read splash image from partition\n");
				return -1;
			}

			fbcon_extract_to_screen(header, (base + LOGO_IMG_HEADER_SIZE));
		} else { /* 2 Raw BGR data */

			if ((header->width > fb_display->width) || (header->height > fb_display->height)) {
				dprintf(CRITICAL, "Logo config greater than fb config. Fall back default logo\n");
				return -1;
			}

			realsize =  header->width * header->height * fb_display->bpp / 8;
			readsize =  ROUNDUP((realsize + LOGO_IMG_HEADER_SIZE), blocksize) - blocksize;

			if (blocksize == LOGO_IMG_HEADER_SIZE) { /* read the content directly */
				if (mmc_read((ptn + LOGO_IMG_HEADER_SIZE), (uint32_t *)base, readsize)) {
					fbcon_clear();
					dprintf(CRITICAL, "ERROR: Cannot read splash image from partition\n");
					return -1;
				}
			} else {
				if (mmc_read(ptn + blocksize ,
						(uint32_t *)(base + LOGO_IMG_OFFSET + blocksize), readsize)) {
					dprintf(CRITICAL, "ERROR: Cannot read splash image from partition\n");
					return -1;
				}
				memmove(base, (base + LOGO_IMG_OFFSET + LOGO_IMG_HEADER_SIZE), realsize);
			}
		}
	}

	return 0;
}

int fetch_image_from_partition()
{
	if (target_is_emmc_boot()) {
		return splash_screen_mmc();
	} else {
		return splash_screen_flash();
	}
}

/* Get the size from partiton name */
static void get_partition_size(const char *arg, char *response)
{
	uint64_t ptn = 0;
	uint64_t size;
	int index = INVALID_PTN;

	index = partition_get_index(arg);

	if (index == INVALID_PTN)
	{
		dprintf(CRITICAL, "Invalid partition index\n");
		return;
	}

	ptn = partition_get_offset(index);

	if(!ptn)
	{
		dprintf(CRITICAL, "Invalid partition name %s\n", arg);
		return;
	}

	size = partition_get_size(index);

	snprintf(response, MAX_RSP_SIZE, "\t 0x%llx", size);
	return;
}

/*
 * Publish the partition type & size info
 * fastboot getvar will publish the required information.
 * fastboot getvar partition_size:<partition_name>: partition size in hex
 * fastboot getvar partition_type:<partition_name>: partition type (ext/fat)
 */
static void publish_getvar_partition_info(struct getvar_partition_info *info, uint8_t num_parts)
{
	uint8_t i;

	for (i = 0; i < num_parts; i++) {
		get_partition_size(info[i].part_name, info[i].size_response);

		if (strlcat(info[i].getvar_size, info[i].part_name, MAX_GET_VAR_NAME_SIZE) >= MAX_GET_VAR_NAME_SIZE)
		{
			dprintf(CRITICAL, "partition size name truncated\n");
			return;
		}
		if (strlcat(info[i].getvar_type, info[i].part_name, MAX_GET_VAR_NAME_SIZE) >= MAX_GET_VAR_NAME_SIZE)
		{
			dprintf(CRITICAL, "partition type name truncated\n");
			return;
		}

		/* publish partition size & type info */
		fastboot_publish((const char *) info[i].getvar_size, (const char *) info[i].size_response);
		fastboot_publish((const char *) info[i].getvar_type, (const char *) info[i].type_response);
	}
}

void publish_getvar_multislot_vars()
{
	int i,count;
	static bool published = false;
	static char slot_count[MAX_RSP_SIZE];
	static struct ab_slot_info slot_info[AB_SUPPORTED_SLOTS];
	static char active_slot_suffix[MAX_RSP_SIZE];
	static char has_slot_pname[NUM_PARTITIONS][MAX_GET_VAR_NAME_SIZE];
	static char has_slot_reply[NUM_PARTITIONS][MAX_RSP_SIZE];
	const char *tmp;
	char tmpbuff[MAX_GET_VAR_NAME_SIZE];
	signed active_slt;

	if (!published)
	{
		/* Update slot meta info */
		count = partition_fill_partition_meta(has_slot_pname, has_slot_reply,
							partition_get_partition_count());
		for(i=0; i<count; i++)
		{
			memset(tmpbuff, 0, MAX_GET_VAR_NAME_SIZE);
			snprintf(tmpbuff, MAX_GET_VAR_NAME_SIZE,"has-slot:%s",
								has_slot_pname[i]);
			strlcpy(has_slot_pname[i], tmpbuff, MAX_GET_VAR_NAME_SIZE);
			fastboot_publish(has_slot_pname[i], has_slot_reply[i]);
		}

		for (i=0; i<AB_SUPPORTED_SLOTS; i++)
		{
			tmp = SUFFIX_SLOT(i);
			snprintf(slot_info[i].slot_is_unbootable, sizeof(slot_info[i].slot_is_unbootable),
										"slot-unbootable:%s", tmp);
			snprintf(slot_info[i].slot_is_active, sizeof(slot_info[i].slot_is_active),
										"slot-active:%s", tmp);
			snprintf(slot_info[i].slot_is_succesful, sizeof(slot_info[i].slot_is_succesful),
										"slot-success:%s", tmp);
			snprintf(slot_info[i].slot_retry_count, sizeof(slot_info[i].slot_retry_count),
										"slot-retry-count:%s", tmp);
			fastboot_publish(slot_info[i].slot_is_unbootable,
							slot_info[i].slot_is_unbootable_rsp);
			fastboot_publish(slot_info[i].slot_is_active,
							slot_info[i].slot_is_active_rsp);
			fastboot_publish(slot_info[i].slot_is_succesful,
							slot_info[i].slot_is_succesful_rsp);
			fastboot_publish(slot_info[i].slot_retry_count,
							slot_info[i].slot_retry_count_rsp);
		}
		fastboot_publish("current-slot", active_slot_suffix);
		snprintf(slot_count, sizeof(slot_count),"%d", AB_SUPPORTED_SLOTS);
		fastboot_publish("slot-count", slot_count);
		published = true;
	}

	active_slt = partition_find_active_slot();
	if (active_slt != INVALID)
		snprintf(active_slot_suffix, sizeof(active_slot_suffix), "%s",
			SUFFIX_SLOT(active_slt));
	else
		strlcpy(active_slot_suffix, "INVALID", sizeof(active_slot_suffix));

	/* Update partition meta information */
	partition_fill_slot_meta(slot_info);
	return;
}

void get_product_name(unsigned char *buf)
{
	snprintf((char*)buf, MAX_RSP_SIZE, "Blackberry Passport");
	return;
}

/* register commands and variables for fastboot */
void aboot_fastboot_register_commands(void)
{
	int i;

	struct fastboot_cmd_desc cmd_list[] = {
		{"flash:", cmd_flash},
		{"erase:", cmd_erase},
		{"boot", cmd_boot},
		{"continue", cmd_continue},
		{"reboot", cmd_reboot},
		{"reboot-bootloader", cmd_reboot_bootloader},
		{"oem device-info", cmd_oem_devinfo},
		{"oem bootlog", cmd_oem_bootlog},
		{"preflash", cmd_preflash},
		{"oem enable-charger-screen", cmd_oem_enable_charger_screen},
		{"oem disable-charger-screen", cmd_oem_disable_charger_screen},
		{"oem off-mode-charge", cmd_oem_off_mode_charger},
		{"oem select-display-panel", cmd_oem_select_display_panel},
		{"set_active",cmd_set_active},
	};

	int fastboot_cmds_count = sizeof(cmd_list)/sizeof(cmd_list[0]);
	for (i = 0; i < fastboot_cmds_count; i++)
		fastboot_register(cmd_list[i].name,cmd_list[i].cb);

	/* publish variables and their values */
	fastboot_publish("product",  TARGET(BOARD));
	fastboot_publish("kernel",   "lk");
	fastboot_publish("serialno", sn_buf);

	snprintf(hwid_str, 20, "0x%x", bbry_get_hwid());
	snprintf(rev_str, 20, "%d", bbry_get_rev());
	fastboot_publish("hwid", hwid_str);
	fastboot_publish("rev", rev_str);

	char *product = bbry_hwi_get_entry("product", 0);
	if (product)
		fastboot_publish("product", product);
	else
		fastboot_publish("product", TARGET(BOARD));

	char *variant = bbry_hwi_get_entry("variant", 0);
	if (variant)
		fastboot_publish("variant", variant);

	char *bbss_insecure = bbry_hwi_get_entry("bbss_insecure", 0);
	if (bbss_insecure)
		fastboot_publish("bbss_insecure", bbss_insecure);

	char *bbss_wp_type = bbry_hwi_get_entry("bbss_wp_type", 0);
	if (bbss_wp_type)
		fastboot_publish("bbss_wp_type", bbss_wp_type);

	char *primary_bc_ver = bbry_hwi_get_entry("primary_bc_ver", 0);
	if (primary_bc_ver)
		fastboot_publish("primary_bc_ver", primary_bc_ver);

	char *backup_bc_ver = bbry_hwi_get_entry("backup_bc_ver", 0);
	if (backup_bc_ver)
		fastboot_publish("backup_bc_ver", backup_bc_ver);

	char *backup_bootchain = bbry_hwi_get_entry("backup_bootchain", 0);
	if (backup_bootchain)
		fastboot_publish("backup_bootchain", backup_bootchain);

	/*
	 * partition info is supported only for emmc partitions
	 * Calling this for NAND prints some error messages which
	 * is harmless but misleading. Avoid calling this for NAND
	 * devices.
	 */
	if (target_is_emmc_boot())
		publish_getvar_partition_info(part_info, ARRAY_SIZE(part_info));

	if (partition_multislot_is_supported())
		publish_getvar_multislot_vars();

	/* Max download size supported */
	snprintf(max_download_size, MAX_RSP_SIZE, "\t0x%x",
			target_get_max_flash_size());
	fastboot_publish("max-download-size", (const char *) max_download_size);
	/* Is the charger screen check enabled */
	snprintf(charger_screen_enabled, MAX_RSP_SIZE, "%d",
			device.charger_screen_enabled);
	fastboot_publish("charger-screen-enabled",
			(const char *) charger_screen_enabled);
	fastboot_publish("off-mode-charge", (const char *) charger_screen_enabled);
	snprintf(panel_display_mode, MAX_RSP_SIZE, "%s",
			device.display_panel);
	fastboot_publish("display-panel",
			(const char *) panel_display_mode);
	fastboot_publish("secure", is_secure_boot_enable()? "yes":"no");
#if CHECK_BAT_VOLTAGE
	update_battery_status();
	fastboot_publish("battery-voltage", (const char *) battery_voltage);
	fastboot_publish("battery-soc-ok", (const char *) battery_soc_ok);
#endif
}

void aboot_init(const struct app_descriptor *app)
{
	unsigned reboot_mode = 0;
	int boot_err_type = 0;
	int boot_slot = INVALID;

	/* Initialise wdog to catch early lk crashes */
#if WDOG_SUPPORT
	msm_wdog_init();
#endif

	/* Setup page size information for nv storage */
	if (target_is_emmc_boot())
	{
		page_size = mmc_page_size();
		page_mask = page_size - 1;
		mmc_blocksize = mmc_get_device_blocksize();
		mmc_blocksize_mask = mmc_blocksize - 1;
	}
	else
	{
		page_size = flash_page_size();
		page_mask = page_size - 1;
	}
	ASSERT((MEMBASE + MEMSIZE) > MEMBASE);

	read_device_info(&device);

	/* Detect multi-slot support */
	if (partition_multislot_is_supported())
	{
		boot_slot = partition_find_active_slot();
		if (boot_slot == INVALID)
		{
			boot_into_fastboot = true;
			dprintf(INFO, "Active Slot: (INVALID)\n");
		}
		else
		{
			/* Setting the state of system to boot active slot */
			partition_mark_active_slot(boot_slot);
			dprintf(INFO, "Active Slot: (%s)\n", SUFFIX_SLOT(boot_slot));
		}
	}

	/* Display splash screen if enabled */
#if DISPLAY_SPLASH_SCREEN
#if NO_ALARM_DISPLAY
	if (!check_alarm_boot()) {
#endif
		dprintf(SPEW, "Display Init: Start\n");
#if DISPLAY_HDMI_PRIMARY
	if (!strlen(device.display_panel))
		strlcpy(device.display_panel, DISPLAY_PANEL_HDMI,
			sizeof(device.display_panel));
#endif
#if ENABLE_WBC
		/* Wait if the display shutdown is in progress */
		while(pm_app_display_shutdown_in_prgs());
		if (!pm_appsbl_display_init_done())
			target_display_init(device.display_panel);
		else
			display_image_on_screen();
#else
		target_display_init(device.display_panel);
#endif
		dprintf(SPEW, "Display Init: Done\n");
#if NO_ALARM_DISPLAY
	}
#endif
#endif

	target_serialno((unsigned char *) sn_buf);
	dprintf(SPEW,"serial number: %s\n",sn_buf);

	memset(display_panel_buf, '\0', MAX_PANEL_BUF_SIZE);

	/*
	 * Check power off reason if user force reset,
	 * if yes phone will do normal boot.
	 */
	if (is_user_force_reset())
		goto normal_boot;

	/* Check if we should do something other than booting up */
	if (keys_get_state(KEY_VOLUMEUP) && keys_get_state(KEY_VOLUMEDOWN))
	{
		dprintf(ALWAYS,"dload mode key sequence detected\n");
		reboot_device(EMERGENCY_DLOAD);
		dprintf(CRITICAL,"Failed to reboot into dload mode\n");

		boot_into_fastboot = true;
	}
	if (!boot_into_fastboot)
	{
		if (keys_get_state(KEY_HOME) || keys_get_state(KEY_VOLUMEUP))
			boot_into_recovery = 1;
		if (!boot_into_recovery &&
			(keys_get_state(KEY_BACK) || keys_get_state(KEY_VOLUMEDOWN)))
			boot_into_fastboot = true;
	}
	#if NO_KEYPAD_DRIVER
	if (fastboot_trigger())
		boot_into_fastboot = true;
	#endif

#if USE_PON_REBOOT_REG
	reboot_mode = check_hard_reboot_mode();
#else
	reboot_mode = check_reboot_mode();
#endif
	if (reboot_mode == RECOVERY_MODE)
	{
		boot_into_recovery = 1;
	}
	else if(reboot_mode == FASTBOOT_MODE)
	{
		boot_into_fastboot = true;
	}
	else if(reboot_mode == ALARM_BOOT)
	{
		boot_reason_alarm = true;
	}

	if (is_backup_bootchain())
	{
		dprintf(ALWAYS, "Backup bootchain\n");
		boot_into_fastboot = 1;
	}

normal_boot:
	if (!boot_into_fastboot)
	{
		if (target_is_emmc_boot())
		{
			if(emmc_recovery_init())
				dprintf(ALWAYS,"error in emmc_recovery_init\n");

retry_boot:
			/* Trying to boot active partition */
			if (partition_multislot_is_supported())
			{
				boot_slot = partition_find_boot_slot();
				partition_mark_active_slot(boot_slot);
				if (boot_slot == INVALID)
					goto fastboot;
			}

			boot_err_type = boot_linux_from_mmc();
			switch (boot_err_type)
			{
				case ERR_INVALID_PAGE_SIZE:
				case ERR_DT_PARSE:
				case ERR_ABOOT_ADDR_OVERLAP:
					if(partition_multislot_is_supported())
						goto retry_boot;
					else
						break;
				case ERR_INVALID_BOOT_MAGIC:
				default:
					break;
				/* going to fastboot menu */
			}
			bbry_blink_code(0x21);
		}
		else
		{
			recovery_init();
			boot_linux_from_flash();
		}
		dprintf(CRITICAL, "ERROR: Could not do normal boot. Reverting "
			"to fastboot mode.\n");
	}

fastboot:
	/* We are here means regular boot did not happen. Start fastboot. */
	if (is_backup_bootchain())
		qpnp_led_set(0, 0, 0x80);
	else
		qpnp_led_set(0x1E, 0, 0x80);

	/* register aboot specific fastboot commands */
	aboot_fastboot_register_commands();

	/* dump partition table for debug info */
	partition_dump();

	/* initialize and start fastboot */
	fastboot_init(target_get_scratch_address(), target_get_max_flash_size());
#if FBCON_DISPLAY_MSG
	display_fastboot_menu();
#endif
}

uint32_t get_page_size()
{
	return page_size;
}


APP_START(aboot)
	.init = aboot_init,
APP_END
