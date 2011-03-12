/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "msm_fb.h"
//#include "../../../../drivers/video/msm/mddihost_eve.h"
#include "mddihosti.h"
#include "mddi_toshiba.h"

extern int mddi_ss_driveric_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel);

static u32 mddi_ss_driveric_panel_detect(void)
{
  printk(KERN_INFO "TODO: add swift panel detection code");
	return 1;
}

static int __init mddi_ss_driveric_hvga_init(void)
{
	int ret;
	struct msm_panel_info pinfo;
	u32 panel;

	printk(KERN_INFO "Swift MDDI Init\n");

	panel = mddi_ss_driveric_panel_detect();

	pinfo.xres = 320;
	pinfo.yres = 480;
	MSM_FB_SINGLE_MODE_PANEL(&pinfo);
	pinfo.type = MDDI_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 16;
	pinfo.lcd.vsync_enable = FALSE;
	pinfo.lcd.refx100 = 31250 * 100 / 480;
                        //(mddi_innotek_rows_per_second * 100) /
                        //mddi_innotek_rows_per_refresh;;

	pinfo.lcd.v_back_porch = 14;
	pinfo.lcd.v_front_porch = 6;
	pinfo.lcd.v_pulse_width = 4;
	pinfo.lcd.hw_vsync_mode = FALSE;
	pinfo.lcd.vsync_notifier_period = (1 * HZ);

	pinfo.bl_max = 32;
	pinfo.bl_min = 1;

	pinfo.clk_rate = 122880000;
	pinfo.clk_min =  120000000;
	pinfo.clk_max =  130000000;
	pinfo.fb_num = 2;

	ret = mddi_ss_driveric_register(&pinfo, 0, panel);
	if (ret) {
		printk(KERN_ERR "%s: failed to register device!\n", __func__);
		return ret;
	}
	return ret;
}

module_init(mddi_ss_driveric_hvga_init);
