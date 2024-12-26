/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

/*---------------------------------------------------------------------------
 * This file is autogenerated file using gcdb parser. Please do not edit it.
 * Update input XML file to add a new entry or update variable in this file
 * VERSION = "1.0"
 *---------------------------------------------------------------------------*/

#ifndef _PANEL_PANORAMA_1440P_CMD_H_

#define _PANEL_PANORAMA_1440P_CMD_H_
/*---------------------------------------------------------------------------*/
/* HEADER files                                                              */
/*---------------------------------------------------------------------------*/
#include "panel.h"

/*---------------------------------------------------------------------------*/
/* Panel configuration                                                       */
/*---------------------------------------------------------------------------*/

static struct panel_config panorama_1440p_cmd_panel_data = {
  "qcom,mdss_dsi_panorama_1440p_cmd", "dsi:0:", "qcom,mdss-dsi-panel",
  10, 1, "DISPLAY_1", 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel resolution                                                          */
/*---------------------------------------------------------------------------*/
static struct panel_resolution panorama_1440p_cmd_panel_res = {
  1440, 1440, 4, 30, 125, 0, 16, 16, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel Color Information                                                   */
/*---------------------------------------------------------------------------*/
static struct color_info panorama_1440p_cmd_color = {
  24, 0, 0xff, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel Command information                                                 */
/*---------------------------------------------------------------------------*/
static char panorama_1440p_cmd_on_cmd0[] = {
 2, 0, 0x39, 0xC0, 0x53, 0x2C};


static char panorama_1440p_cmd_on_cmd1[] = {
 5, 0, 0x39, 0xC0, 0x2A, 0, 0, 5, 0x9F};


static char panorama_1440p_cmd_on_cmd2[] = {
 5, 0, 0x39, 0xC0, 0x2B, 0, 0, 5, 0x9F};


static char panorama_1440p_cmd_on_cmd3[] = {
 2, 0, 0x39, 0xC0, 0x35, 0};


static char panorama_1440p_cmd_on_cmd4[] = {
  2, 0, 0x39, 0xC0, 0x36, 0};


static char panorama_1440p_cmd_on_cmd5[] = {
2, 0, 0x39, 0xC0, 0x3A, 0x77
 };


static char panorama_1440p_cmd_on_cmd6[] = {
  2, 0, 0x39, 0xC0, 0x51, 0xFF};


static char panorama_1440p_cmd_on_cmd7[] = {
 2, 0, 0x39, 0xC0, 0x55, 0};


static char panorama_1440p_cmd_on_cmd8[] = {
 2, 0, 0x39, 0xC0, 0x5E, 0};


static char panorama_1440p_cmd_on_cmd9[] = {
0x11, 0x00, 0x05, 0x80  };


static char panorama_1440p_cmd_on_cmd10[] = {
0x29, 0x00, 0x05, 0x80 };




static struct mipi_dsi_cmd panorama_1440p_cmd_on_command[] = {
{ 0x6 , panorama_1440p_cmd_on_cmd0 , 10},
{ 0x9 , panorama_1440p_cmd_on_cmd1 , 10},
{ 0x9 , panorama_1440p_cmd_on_cmd2 , 10},
{ 0x6 , panorama_1440p_cmd_on_cmd3 , 10},
{ 0x6 , panorama_1440p_cmd_on_cmd4 , 10},
{ 0x6 , panorama_1440p_cmd_on_cmd5 , 10},
{ 0x6 , panorama_1440p_cmd_on_cmd6 , 10},
{ 0x6 , panorama_1440p_cmd_on_cmd7 , 10},
{ 0x6 , panorama_1440p_cmd_on_cmd8 , 10},
{ 0x4 , panorama_1440p_cmd_on_cmd9 , 120},
{ 0x4 , panorama_1440p_cmd_on_cmd10 , 10},
};
#define PANORAMA_1440P_CMD_ON_COMMAND 11


static char panorama_1440p_cmdoff_cmd0[] = {
0x28, 0x00, 0x05, 0x80 };


static char panorama_1440p_cmdoff_cmd1[] = {
0x10, 0x00, 0x05, 0x80 };




static struct mipi_dsi_cmd panorama_1440p_cmd_off_command[] = {
{ 0x4 , panorama_1440p_cmdoff_cmd0},
{ 0x4 , panorama_1440p_cmdoff_cmd1}
};
#define PANORAMA_1440P_CMD_OFF_COMMAND 2


static struct command_state panorama_1440p_cmd_state = {
  0, 1
};

/*---------------------------------------------------------------------------*/
/* Command mode panel information                                            */
/*---------------------------------------------------------------------------*/

static struct commandpanel_info panorama_1440p_cmd_command_panel = {
  1, 1, 1, 0, 0, 0x2c, 0, 0, 0, 1, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Video mode panel information                                              */
/*---------------------------------------------------------------------------*/

static struct videopanel_info panorama_1440p_cmd_video_panel = {
  0, 0, 0, 0, 1, 1, 1, 0, 0x9
};

/*---------------------------------------------------------------------------*/
/* Lane Configuration                                                        */
/*---------------------------------------------------------------------------*/

static struct lane_configuration panorama_1440p_cmd_lane_config = {
  4, 0, 1, 1, 1, 1, 0
};


/*---------------------------------------------------------------------------*/
/* Panel Timing                                                              */
/*---------------------------------------------------------------------------*/
const uint32_t panorama_1440p_cmd_timings[] = {
  0xe2, 0x36, 0x24, 0x00, 0x66, 0x68, 0x2a, 0x38,  0x2a, 0x03, 0x04, 0x00
};



static struct mipi_dsi_cmd panorama_1440p_cmd_rotation[] = {

};
#define PANORAMA_1440P_CMD_ROTATION 0


static struct panel_timing panorama_1440p_cmd_timing_info = {
  0x0, 0x04, 0x02, 0x2a
};

static struct panel_reset_sequence panorama_1440p_cmd_panel_reset_seq = {
{ 1, 0, 1, }, { 20, 2, 20, }, 2
};

/*---------------------------------------------------------------------------*/
/* Backlight Settings                                                        */
/*---------------------------------------------------------------------------*/

static struct backlight panorama_1440p_cmd_backlight_wled = {
  BL_WLED, 1, 4095, 100, 1, "PMIC_8941"
};

static struct backlight panorama_1440p_cmd_backlight_samanta_v1 = {
  BL_SAMANTA_V1, 1, 4095, 100, 1, "SAMANTA_V1"
};

static struct backlight panorama_1440p_cmd_backlight_samanta_v2 = {
  BL_SAMANTA_V2, 1, 4095, 1, 1, "SAMANTA_V2"
};
#define PANORAMA_1440P_CMD_SIGNATURE 0x210000


#endif /*_PANEL_PANORAMA_1440P_CMD_H_*/
