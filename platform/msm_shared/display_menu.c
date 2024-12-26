/* Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*	notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*	copyright notice, this list of conditions and the following
*	disclaimer in the documentation and/or other materials provided
*	with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*	contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
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
#include <reg.h>
#include <stdlib.h>
#include <openssl/evp.h>
#include <dev/fbcon.h>
#include <kernel/thread.h>
#include <display_menu.h>
#include <menu_keys_detect.h>
#include <string.h>
#include <platform.h>
#include <smem.h>
#include <target.h>
#include <sys/types.h>
#include <../../../app/aboot/devinfo.h>

static bool is_thread_start = false;
static struct select_msg_info msg_info;

static char *fastboot_option_menu[] = {
		[0] = "START\n",
		[1] = "Restart bootloader\n",
		[2] = "Recovery mode\n",
		[3] = "Power off\n"};

static int big_factor = 2;
static int common_factor = 1;

static void wait_for_exit()
{
	struct select_msg_info *select_msg;
	select_msg = &msg_info;

	mutex_acquire(&select_msg->msg_lock);
	while(!select_msg->info.rel_exit == true) {
		mutex_release(&select_msg->msg_lock);
		thread_sleep(10);
		mutex_acquire(&select_msg->msg_lock);
	}
	mutex_release(&select_msg->msg_lock);

	is_thread_start = false;
	fbcon_clear();
	display_image_on_screen();
}

void wait_for_users_action()
{
	/* Waiting for exit menu keys detection if there is no any usr action
	 * otherwise it will do the action base on the keys detection thread
	 */
	wait_for_exit();
}

void exit_menu_keys_detection()
{
	struct select_msg_info *select_msg;
	select_msg = &msg_info;

	mutex_acquire(&select_msg->msg_lock);
	select_msg->info.is_exit = true;
	mutex_release(&select_msg->msg_lock);

	wait_for_exit();
}

static void set_message_factor()
{
	uint32_t tmp_factor = 0;
	uint32_t max_x_count = 40;
	uint32_t max_x = fbcon_get_max_x();

	max_x = fbcon_get_max_x();
	tmp_factor = max_x/max_x_count;

	if(tmp_factor <= 1) {
		big_factor = 2;
		common_factor = 1;
	} else {
		big_factor = tmp_factor*2;
		common_factor = tmp_factor;
	}
}

static void display_fbcon_menu_message(char *str, unsigned type,
	unsigned scale_factor)
{
	while(*str != 0) {
		fbcon_putc_factor(*str++, type, scale_factor);
	}
}

static char *str_align_right(char *str, int factor)
{
	uint32_t max_x = 0;
	int diff = 0;
	int i = 0;
	char *str_target = NULL;

	max_x = fbcon_get_max_x();
	if (!str_target && max_x) {
		str_target = malloc(max_x);
	}

	if (str_target) {
		memset(str_target, 0, max_x);
		if ( max_x/factor > strlen(str)) {
			if (factor == 1)
				diff = max_x/factor - strlen(str) - 1;
			else
				diff = max_x/factor - strlen(str);
			for (i = 0; i < diff; i++) {
				strlcat(str_target, " ", max_x);
			}
			strlcat(str_target, str, max_x);
			return str_target;
		} else {
			free(str_target);
			return str;
		}
	}
	return str;
}

/* msg_lock need to be holded when call this function. */
void display_fastboot_menu_renew(struct select_msg_info *fastboot_msg_info)
{
	int len;
	int msg_type = FBCON_COMMON_MSG;
	char msg_buf[64];
	char msg[128];

	/* The fastboot menu is switched base on the option index
	 * So it's need to store the index for the menu switching
	 */
	uint32_t option_index = fastboot_msg_info->info.option_index;

	fbcon_clear();
	memset(&fastboot_msg_info->info, 0, sizeof(struct menu_info));

	len = ARRAY_SIZE(fastboot_option_menu);
	switch(option_index) {
		case 0:
			msg_type = FBCON_GREEN_MSG;
			break;
		case 1:
		case 2:
			msg_type = FBCON_RED_MSG;
			break;
		case 3:
		case 4:
			msg_type = FBCON_COMMON_MSG;
			break;
	}
	fbcon_draw_line(msg_type);
	display_fbcon_menu_message(fastboot_option_menu[option_index],
		msg_type, big_factor);
	fbcon_draw_line(msg_type);
	display_fbcon_menu_message("\n\nPress volume key to select, and "\
		"press power key to select\n\n", FBCON_COMMON_MSG, common_factor);

	display_fbcon_menu_message("FASTBOOT MODE\n", FBCON_RED_MSG, common_factor);

	get_product_name((unsigned char *) msg_buf);
	snprintf(msg, sizeof(msg), "Name: - %s\n", msg_buf);
	display_fbcon_menu_message(msg, FBCON_COMMON_MSG, common_factor);

	char *product = bbry_hwi_get_entry("product", 0);
	if (product) {
		snprintf(msg, sizeof(msg), "Codename: %s\n", product);
		display_fbcon_menu_message(msg, FBCON_COMMON_MSG, common_factor);
	}

	char *variant = bbry_hwi_get_entry("variant", 0);
	if (variant) {
		snprintf(msg, sizeof(msg), "Variant: %s\n", variant);
		display_fbcon_menu_message(msg, FBCON_COMMON_MSG, common_factor);
	}
	
	
	snprintf(msg, sizeof(msg), "HWID: 0x%x\n", bbry_get_hwid());
	display_fbcon_menu_message(msg, FBCON_COMMON_MSG, common_factor);
	
	
	const char *pcb_rev = bbry_hwi_get_entry("pcb_rev", 0);
	if (variant) {
		snprintf(msg, sizeof(msg), "PCB revision: %s\n", pcb_rev);
		display_fbcon_menu_message(msg, FBCON_COMMON_MSG, common_factor);
	}
	
	
	const char *pop_rev = bbry_hwi_get_entry("pop_rev", 0);
	if (variant) {
		snprintf(msg, sizeof(msg), "POP revision: %s\n", pop_rev);
		display_fbcon_menu_message(msg, FBCON_COMMON_MSG, common_factor);
	}

	memset(msg_buf, 0, sizeof(msg_buf));
	target_serialno((unsigned char *) msg_buf);
	snprintf(msg, sizeof(msg), "Serial number: %s\n", msg_buf);
	display_fbcon_menu_message(msg, FBCON_COMMON_MSG, common_factor);

	fastboot_msg_info->info.msg_type = DISPLAY_MENU_FASTBOOT;
	fastboot_msg_info->info.option_num = len;
	fastboot_msg_info->info.option_index = option_index;
}

void msg_lock_init()
{
	static bool is_msg_lock_init = false;
	struct select_msg_info *msg_lock_info;
	msg_lock_info = &msg_info;

	if (!is_msg_lock_init) {
		mutex_init(&msg_lock_info->msg_lock);
		is_msg_lock_init = true;
	}
}

static void display_menu_thread_start(struct select_msg_info *msg_info)
{
	thread_t *thr;

	if (!is_thread_start) {
		thr = thread_create("selectkeydetect", &select_msg_keys_detect,
			(void*)msg_info, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE);
		if (!thr) {
			dprintf(CRITICAL, "ERROR: creat device status detect thread failed!!\n");
			return;
		}
		thread_resume(thr);
		is_thread_start = true;
	}
}

void display_fastboot_menu()
{
	struct select_msg_info *fastboot_menu_msg_info;
	fastboot_menu_msg_info = &msg_info;

	set_message_factor();

	msg_lock_init();
	mutex_acquire(&fastboot_menu_msg_info->msg_lock);

	/* There are 4 pages for fastboot menu:
	 * Page: Start/Fastboot/Recovery/Poweroff
	 * The menu is switched base on the option index
	 * Initialize the option index and last_msg_type
	 */
	fastboot_menu_msg_info->info.option_index = 0;
	fastboot_menu_msg_info->last_msg_type =
		fastboot_menu_msg_info->info.msg_type;

	display_fastboot_menu_renew(fastboot_menu_msg_info);
	mutex_release(&fastboot_menu_msg_info->msg_lock);

	dprintf(INFO, "creating fastboot menu keys detect thread\n");
	display_menu_thread_start(fastboot_menu_msg_info);
}
