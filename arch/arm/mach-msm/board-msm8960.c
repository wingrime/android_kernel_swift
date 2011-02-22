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
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>

#include "timer.h"

#include "devices.h"

static void __init msm8960_map_io(void)
{
	msm_map_msm8960_io();
}

void __iomem *gic_cpu_base_addr;

static void __init msm8960_init_irq(void)
{
	unsigned int i;
	gic_dist_init(0, MSM_QGIC_DIST_BASE, GIC_PPI_START);
	gic_cpu_base_addr = (void *)MSM_QGIC_CPU_BASE;
	gic_cpu_init(0, MSM_QGIC_CPU_BASE);

	/* Edge trigger PPIs except AVS_SVICINT and AVS_SVICINTSWDONE */
	writel(0xFFFFD7FF, MSM_QGIC_DIST_BASE + GIC_DIST_CONFIG + 4);

	writel(0x0000FFFF, MSM_QGIC_DIST_BASE + GIC_DIST_ENABLE_SET);

	/* FIXME: Not installing AVS_SVICINT and AVS_SVICINTSWDONE yet
	 * as they are configured as level, which does not play nice with
	 * handle_percpu_irq.
	 */
	for (i = GIC_PPI_START; i < GIC_SPI_START; i++) {
		if (i != AVS_SVICINT && i != AVS_SVICINTSWDONE)
			set_irq_handler(i, handle_percpu_irq);
	}
}

static struct mmc_platform_data msm8960_sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};

static void __init msm8960_init_mmc(void)
{
	/* SDCC1 : eMMC card connected */
	msm_add_sdcc(1, &msm8960_sdc1_data);
}

static struct platform_device *sim_devices[] __initdata = {
	&msm8960_device_uart_gsbi2,
};

static struct platform_device *rumi3_devices[] __initdata = {
	&msm8960_device_uart_gsbi5,
};

static void __init msm8960_sim_init(void)
{
	msm_clock_init(msm_clocks_8960, msm_num_clocks_8960);
	platform_add_devices(sim_devices, ARRAY_SIZE(sim_devices));
	msm8960_init_mmc();
}

static void __init msm8960_rumi3_init(void)
{
	msm_clock_init(msm_clocks_8960, msm_num_clocks_8960);
	platform_add_devices(rumi3_devices, ARRAY_SIZE(rumi3_devices));
}

MACHINE_START(MSM8960_SIM, "QCT MSM8960 SIMULATOR")
	.map_io = msm8960_map_io,
	.init_irq = msm8960_init_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_sim_init,
MACHINE_END

MACHINE_START(MSM8960_RUMI3, "QCT MSM8960 RUMI3")
	.map_io = msm8960_map_io,
	.init_irq = msm8960_init_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_rumi3_init,
MACHINE_END
