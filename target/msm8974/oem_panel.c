/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
#include <err.h>
#include <smem.h>
#include <msm_panel.h>
#include <board.h>
#include <mipi_dsi.h>
#include <string.h>
#include <target/display.h>

#include "include/panel.h"
#include "panel_display.h"

/*---------------------------------------------------------------------------*/
/* GCDB Panel Database                                                       */
/*---------------------------------------------------------------------------*/
#include "include/panel_panorama_1440p_cmd.h"

int oem_panel_rotation()
{
	/* OEM can keep there panel spefic on instructions in this
	function */
	return NO_ERROR;
}


int oem_panel_on()
{
	/* OEM can keep there panel spefic on instructions in this
	function */
	return NO_ERROR;
}

int oem_panel_off()
{
	/* OEM can keep there panel spefic off instructions in this
	function */
	return NO_ERROR;
}

uint32_t oem_panel_max_auto_detect_panels()
{
	return 0;
}

static uint32_t auto_pan_loop = 0;

int oem_panel_select(const char *panel_name, struct panel_struct *panelstruct,
			struct msm_panel_info *pinfo,
			struct mdss_dsi_phy_ctrl *phy_db)
{
	panelstruct->paneldata    = &panorama_1440p_cmd_panel_data;
	panelstruct->panelres     = &panorama_1440p_cmd_panel_res;
	panelstruct->color        = &panorama_1440p_cmd_color;
	panelstruct->videopanel   = &panorama_1440p_cmd_video_panel;
	panelstruct->commandpanel = &panorama_1440p_cmd_command_panel;
	panelstruct->state        = &panorama_1440p_cmd_state;
	panelstruct->laneconfig   = &panorama_1440p_cmd_lane_config;
	panelstruct->paneltiminginfo
		= &panorama_1440p_cmd_timing_info;
	panelstruct->panelresetseq
				 = &panorama_1440p_cmd_panel_reset_seq;

	if (strcmp(bbry_get_product(), "wolverine") == 0)
	{
		if (strcmp(bbry_get_variant(), "wichita") == 0)
		{
			panelstruct->backlightinfo = &panorama_1440p_cmd_backlight_samanta_v2;
		}
		else
		{
			int board_rev = bbry_get_rev();
			if (board_rev <= 3) {
				panelstruct->backlightinfo = &panorama_1440p_cmd_backlight_samanta_v1;
			} else if (board_rev == 4) {
				int device_variant = bbry_get_device_variant();
				if (device_variant == -1)
					panelstruct->backlightinfo = &panorama_1440p_cmd_backlight_samanta_v2;
				else if (device_variant != 41)
					panelstruct->backlightinfo = &panorama_1440p_cmd_backlight_samanta_v1;
				else
					panelstruct->backlightinfo = &panorama_1440p_cmd_backlight_wled;
			} else
				panelstruct->backlightinfo = &panorama_1440p_cmd_backlight_wled;
		}
	}
	else
		panelstruct->backlightinfo = &panorama_1440p_cmd_backlight_wled;

	pinfo->mipi.panel_on_cmds
		= panorama_1440p_cmd_on_command;
	pinfo->mipi.num_of_panel_on_cmds
		= PANORAMA_1440P_CMD_ON_COMMAND;
	pinfo->mipi.panel_off_cmds
		= panorama_1440p_cmd_off_command;
	pinfo->mipi.num_of_panel_off_cmds
		= PANORAMA_1440P_CMD_OFF_COMMAND;
	memcpy(phy_db->timing,
		panorama_1440p_cmd_timings, TIMING_SIZE);
	pinfo->mipi.signature = PANORAMA_1440P_CMD_SIGNATURE;

	return PANEL_TYPE_DSI;
}
