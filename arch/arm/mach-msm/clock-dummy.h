/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_DUMMY_H
#define __ARCH_ARM_MACH_MSM_CLOCK_DUMMY_H

extern struct clk_ops clk_ops_dummy;
#define CLK_DUMMY(clk_name, clk_id, clk_dev, clk_flags) {	\
	.con_id = clk_name, \
	.dev_id = clk_dev, \
	.clk = &(struct clk){ \
		.dbg_name = #clk_id, \
		.flags = clk_flags, \
		.ops = &clk_ops_dummy, \
	}, \
	}
#endif
