/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <mach/irqs.h>
#include <mach/dma.h>
#include <asm/mach/mmc.h>
#include <asm/clkdev.h>
#include <linux/msm_kgsl.h>
#include <linux/msm_rotator.h>
#include <mach/msm_hsusb.h>
#include "footswitch.h"
#include "clock.h"
#include "clock-8x60.h"
#include "clock-rpm.h"
#include "clock-voter.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/clkdev.h>
#include <mach/usbdiag.h>
#include <mach/usb_gadget_fserial.h>
#include <mach/msm_serial_hs_lite.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_tsif.h>
#include <mach/scm-io.h>
#ifdef CONFIG_MSM_DSPS
#include <mach/msm_dsps.h>
#endif
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/mdm.h>
#include <mach/rpm.h>
#include "rpm_stats.h"
#include "mpm.h"

/* Address of GSBI blocks */
#define MSM_GSBI1_PHYS	0x16000000
#define MSM_GSBI2_PHYS	0x16100000
#define MSM_GSBI3_PHYS	0x16200000
#define MSM_GSBI4_PHYS	0x16300000
#define MSM_GSBI5_PHYS	0x16400000
#define MSM_GSBI6_PHYS	0x16500000
#define MSM_GSBI7_PHYS	0x16600000
#define MSM_GSBI8_PHYS	0x19800000
#define MSM_GSBI9_PHYS	0x19900000
#define MSM_GSBI10_PHYS	0x19A00000
#define MSM_GSBI11_PHYS	0x19B00000
#define MSM_GSBI12_PHYS	0x19C00000

/* GSBI QUPe devices */
#define MSM_GSBI1_QUP_PHYS	0x16080000
#define MSM_GSBI2_QUP_PHYS	0x16180000
#define MSM_GSBI3_QUP_PHYS	0x16280000
#define MSM_GSBI4_QUP_PHYS	0x16380000
#define MSM_GSBI5_QUP_PHYS	0x16480000
#define MSM_GSBI6_QUP_PHYS	0x16580000
#define MSM_GSBI7_QUP_PHYS	0x16680000
#define MSM_GSBI8_QUP_PHYS	0x19880000
#define MSM_GSBI9_QUP_PHYS	0x19980000
#define MSM_GSBI10_QUP_PHYS	0x19A80000
#define MSM_GSBI11_QUP_PHYS	0x19B80000
#define MSM_GSBI12_QUP_PHYS	0x19C80000

/* GSBI UART devices */
#define MSM_UART1DM_PHYS    (MSM_GSBI6_PHYS + 0x40000)
#define INT_UART1DM_IRQ     GSBI6_UARTDM_IRQ
#define INT_UART2DM_IRQ     GSBI12_UARTDM_IRQ
#define MSM_UART2DM_PHYS    0x19C40000
#define MSM_UART3DM_PHYS    (MSM_GSBI3_PHYS + 0x40000)
#define INT_UART3DM_IRQ     GSBI3_UARTDM_IRQ
#define TCSR_BASE_PHYS      0x16b00000

/* PRNG device */
#define MSM_PRNG_PHYS		0x16C00000
#define MSM_UART9DM_PHYS    (MSM_GSBI9_PHYS + 0x40000)
#define INT_UART9DM_IRQ     GSBI9_UARTDM_IRQ

static void charm_ap2mdm_kpdpwr_on(void)
{
	gpio_direction_output(AP2MDM_PMIC_RESET_N, 0);
	if (machine_is_msm8x60_charm_surf())
		gpio_direction_output(AP2MDM_KPDPWR_N, 0);
	else
		gpio_direction_output(AP2MDM_KPDPWR_N, 1);
}

static void charm_ap2mdm_kpdpwr_off(void)
{
	int i;

	gpio_direction_output(AP2MDM_ERRFATAL, 1);

	for (i = 20; i > 0; i--) {
		if (gpio_get_value(MDM2AP_STATUS) == 0)
			break;
		msleep(100);
	}
	gpio_direction_output(AP2MDM_ERRFATAL, 0);

	if (i == 0) {
		pr_err("%s: MDM2AP_STATUS never went low. Doing a hard reset \
			of the charm modem.\n", __func__);
		gpio_direction_output(AP2MDM_PMIC_RESET_N, 1);
		/*
		* Currently, there is a debounce timer on the charm PMIC. It is
		* necessary to hold the AP2MDM_PMIC_RESET low for ~3.5 seconds
		* for the reset to fully take place. Sleep here to ensure the
		* reset has occured before the function exits.
		*/
		msleep(4000);
		gpio_direction_output(AP2MDM_PMIC_RESET_N, 0);
	}
}

static struct resource charm_resources[] = {
	/* MDM2AP_ERRFATAL */
	{
		.start	= MSM_GPIO_TO_INT(MDM2AP_ERRFATAL),
		.end	= MSM_GPIO_TO_INT(MDM2AP_ERRFATAL),
		.flags = IORESOURCE_IRQ,
	},
	/* MDM2AP_STATUS */
	{
		.start	= MSM_GPIO_TO_INT(MDM2AP_STATUS),
		.end	= MSM_GPIO_TO_INT(MDM2AP_STATUS),
		.flags = IORESOURCE_IRQ,
	}
};

static struct charm_platform_data mdm_platform_data = {
	.charm_modem_on		= charm_ap2mdm_kpdpwr_on,
	.charm_modem_off	= charm_ap2mdm_kpdpwr_off,
};

struct platform_device msm_charm_modem = {
	.name		= "charm_modem",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(charm_resources),
	.resource	= charm_resources,
	.dev		= {
		.platform_data = &mdm_platform_data,
	},
};

#ifdef CONFIG_MSM_DSPS
#define GSBI12_DEV (&msm_dsps_device.dev)
#else
#define GSBI12_DEV (&msm_gsbi12_qup_i2c_device.dev)
#endif

void __iomem *gic_cpu_base_addr;

void __init msm8x60_init_irq(void)
{
	unsigned int i;

	gic_dist_init(0, MSM_QGIC_DIST_BASE, GIC_PPI_START);
	gic_cpu_base_addr = (void *)MSM_QGIC_CPU_BASE;
	gic_cpu_init(0, MSM_QGIC_CPU_BASE);

	/* Edge trigger PPIs except AVS_SVICINT and AVS_SVICINTSWDONE */
	writel(0xFFFFD7FF, MSM_QGIC_DIST_BASE + GIC_DIST_CONFIG + 4);

	/* QGIC does not adhere to GIC spec by enabling STIs by default.
	 * Enable/clear is supposed to be RO for STIs, but is RW on QGIC.
	 */
	if (!machine_is_msm8x60_sim())
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

static struct resource msm_uart1_dm_resources[] = {
	{
		.start = MSM_UART1DM_PHYS,
		.end   = MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART1DM_IRQ,
		.end   = INT_UART1DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* GSBI6 is UARTDM1 */
		.start = MSM_GSBI6_PHYS,
		.end   = MSM_GSBI6_PHYS + 4 - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm1 = {
	.name = "msm_serial_hs",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev            = {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart3_dm_resources[] = {
	{
		.start = MSM_UART3DM_PHYS,
		.end   = MSM_UART3DM_PHYS + PAGE_SIZE - 1,
		.name  = "uartdm_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART3DM_IRQ,
		.end   = INT_UART3DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MSM_GSBI3_PHYS,
		.end   = MSM_GSBI3_PHYS + PAGE_SIZE - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_dm3 = {
	.name = "msm_serial_hsl",
	.id = 2,
	.num_resources = ARRAY_SIZE(msm_uart3_dm_resources),
	.resource = msm_uart3_dm_resources,
};

static struct resource msm_uart12_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.name  = "uartdm_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* GSBI 12 is UARTDM2 */
		.start = MSM_GSBI12_PHYS,
		.end   = MSM_GSBI12_PHYS + PAGE_SIZE - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_dm12 = {
	.name = "msm_serial_hsl",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart12_dm_resources),
	.resource = msm_uart12_dm_resources,
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct msm_serial_hslite_platform_data uart_gsbi9_pdata = {
	.config_gpio	= 1,
	.uart_tx_gpio	= 67,
	.uart_rx_gpio	= 66,
};

static struct resource msm_uart_gsbi9_resources[] = {
       {
		.start	= MSM_UART9DM_PHYS,
		.end	= MSM_UART9DM_PHYS + PAGE_SIZE - 1,
		.name	= "uartdm_resource",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_UART9DM_IRQ,
		.end	= INT_UART9DM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		/* GSBI 9 is UART_GSBI9 */
		.start	= MSM_GSBI9_PHYS,
		.end	= MSM_GSBI9_PHYS + PAGE_SIZE - 1,
		.name	= "gsbi_resource",
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device *msm_device_uart_gsbi9;
struct platform_device *msm_add_gsbi9_uart(void)
{
	int ret = -ENOMEM;
	struct platform_device *pdev;

	pdev = platform_device_alloc("msm_serial_hsl", 1);
	if (!pdev)
		goto fail;

	ret = platform_device_add_resources(pdev,
				msm_uart_gsbi9_resources,
				ARRAY_SIZE(msm_uart_gsbi9_resources));
	if (ret)
		goto fail;

	ret = platform_device_add_data(pdev, &uart_gsbi9_pdata,
						sizeof(uart_gsbi9_pdata));
	if (ret)
		goto fail;

	ret = platform_device_add(pdev);
	if (ret)
		goto fail;

	return pdev;
fail:
	pr_err("%s(): Error creating device\n", __func__);
	platform_device_put(pdev);
	return ERR_PTR(ret);
}
#endif

static struct resource gsbi3_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI3_QUP_PHYS,
		.end	= MSM_GSBI3_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI3_PHYS,
		.end	= MSM_GSBI3_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI3_QUP_IRQ,
		.end	= GSBI3_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi4_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI4_QUP_PHYS,
		.end	= MSM_GSBI4_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI4_PHYS,
		.end	= MSM_GSBI4_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI4_QUP_IRQ,
		.end	= GSBI4_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi7_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI7_QUP_PHYS,
		.end	= MSM_GSBI7_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI7_PHYS,
		.end	= MSM_GSBI7_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI7_QUP_IRQ,
		.end	= GSBI7_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi8_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI8_QUP_PHYS,
		.end	= MSM_GSBI8_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI8_PHYS,
		.end	= MSM_GSBI8_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI8_QUP_IRQ,
		.end	= GSBI8_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi9_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI9_QUP_PHYS,
		.end	= MSM_GSBI9_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI9_PHYS,
		.end	= MSM_GSBI9_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI9_QUP_IRQ,
		.end	= GSBI9_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi12_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI12_QUP_PHYS,
		.end	= MSM_GSBI12_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI12_PHYS,
		.end	= MSM_GSBI12_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI12_QUP_IRQ,
		.end	= GSBI12_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors grp3d_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp3d_nominal_low_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 1300000000U,
	},
};

static struct msm_bus_vectors grp3d_nominal_high_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 2008000000U,
	},
};

static struct msm_bus_vectors grp3d_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 2484000000U,
	},
};

static struct msm_bus_paths grp3d_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp3d_init_vectors),
		grp3d_init_vectors,
	},
	{
		ARRAY_SIZE(grp3d_nominal_low_vectors),
		grp3d_nominal_low_vectors,
	},
	{
		ARRAY_SIZE(grp3d_nominal_high_vectors),
		grp3d_nominal_high_vectors,
	},
	{
		ARRAY_SIZE(grp3d_max_vectors),
		grp3d_max_vectors,
	},
};

static struct msm_bus_scale_pdata grp3d_bus_scale_pdata = {
	grp3d_bus_scale_usecases,
	ARRAY_SIZE(grp3d_bus_scale_usecases),
	.name = "grp3d",
};

#ifdef CONFIG_MSM_KGSL_2D
static struct msm_bus_vectors grp2d0_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp2d0_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 1300000000U,
	},
};

static struct msm_bus_paths grp2d0_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp2d0_init_vectors),
		grp2d0_init_vectors,
	},
	{
		ARRAY_SIZE(grp2d0_max_vectors),
		grp2d0_max_vectors,
	},
};

static struct msm_bus_scale_pdata grp2d0_bus_scale_pdata = {
	grp2d0_bus_scale_usecases,
	ARRAY_SIZE(grp2d0_bus_scale_usecases),
	.name = "grp2d0",
};

static struct msm_bus_vectors grp2d1_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp2d1_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 1300000000U,
	},
};

static struct msm_bus_paths grp2d1_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp2d1_init_vectors),
		grp2d1_init_vectors,
	},
	{
		ARRAY_SIZE(grp2d1_max_vectors),
		grp2d1_max_vectors,
	},
};

static struct msm_bus_scale_pdata grp2d1_bus_scale_pdata = {
	grp2d1_bus_scale_usecases,
	ARRAY_SIZE(grp2d1_bus_scale_usecases),
	.name = "grp2d1",
};
#endif
#endif

#ifdef CONFIG_HW_RANDOM_MSM
static struct resource rng_resources = {
	.flags = IORESOURCE_MEM,
	.start = MSM_PRNG_PHYS,
	.end   = MSM_PRNG_PHYS + SZ_512 - 1,
};

struct platform_device msm_device_rng = {
	.name          = "msm_rng",
	.id            = 0,
	.num_resources = 1,
	.resource      = &rng_resources,
};
#endif

static struct resource kgsl_3d0_resources[] = {
	{
		.name = KGSL_3D0_REG_MEMORY,
		.start = 0x04300000, /* GFX3D address */
		.end = 0x0431ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = KGSL_3D0_IRQ,
		.start = GFX3D_IRQ,
		.end = GFX3D_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_3d0_pdata = {
	.pwr_data = {
		.pwrlevel = {
			{
				.gpu_freq = 266667000,
				.bus_freq = 3,
			},
			{
				.gpu_freq = 228571000,
				.bus_freq = 2,
			},
			{
				.gpu_freq = 200000000,
				.bus_freq = 1,
			},
			{
				.gpu_freq = 27000000,
				.bus_freq = 0,
			},
		},
		.init_level = 0,
		.num_levels = 4,
		.set_grp_async = NULL,
		.idle_timeout = HZ/5,
#ifdef CONFIG_MSM_BUS_SCALING
		.nap_allowed = true,
		.idle_pass = true,
#endif
		.pwrrail_first = true,
	},
	.clk = {
		.name = {
			.clk = "gfx3d_clk",
			.pclk = "gfx3d_pclk",
		},
#ifdef CONFIG_MSM_BUS_SCALING
		.bus_scale_table = &grp3d_bus_scale_pdata,
#endif
	},
	.imem_clk_name = {
		.clk = NULL,
		.pclk = "imem_pclk",
	},
};

struct platform_device msm_kgsl_3d0 = {
	.name = "kgsl-3d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_3d0_resources),
	.resource = kgsl_3d0_resources,
	.dev = {
		.platform_data = &kgsl_3d0_pdata,
	},
};

#ifdef CONFIG_MSM_KGSL_2D
static struct resource kgsl_2d0_resources[] = {
	{
		.name = KGSL_2D0_REG_MEMORY,
		.start = 0x04100000, /* Z180 base address */
		.end = 0x04100FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = KGSL_2D0_IRQ,
		.start = GFX2D0_IRQ,
		.end = GFX2D0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_2d0_pdata = {
	.pwr_data = {
		.pwrlevel = {
			{
				.gpu_freq = 200000000,
				.bus_freq = 1,
			},
			{
				.gpu_freq = 200000000,
				.bus_freq = 0,
			},
		},
		.init_level = 0,
		.num_levels = 2,
		.set_grp_async = NULL,
		.idle_timeout = HZ/10,
#ifdef CONFIG_MSM_BUS_SCALING
		.nap_allowed = true,
#endif
		.pwrrail_first = true,
	},
	.clk = {
		.name = {
			/* note: 2d clocks disabled on v1 */
			.clk = "gfx2d0_clk",
			.pclk = "gfx2d0_pclk",
		},
#ifdef CONFIG_MSM_BUS_SCALING
		.bus_scale_table = &grp2d0_bus_scale_pdata,
#endif
	},
};

struct platform_device msm_kgsl_2d0 = {
	.name = "kgsl-2d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_2d0_resources),
	.resource = kgsl_2d0_resources,
	.dev = {
		.platform_data = &kgsl_2d0_pdata,
	},
};

static struct resource kgsl_2d1_resources[] = {
	{
		.name = KGSL_2D1_REG_MEMORY,
		.start = 0x04200000, /* Z180 device 1 base address */
		.end =   0x04200FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = KGSL_2D1_IRQ,
		.start = GFX2D1_IRQ,
		.end = GFX2D1_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_2d1_pdata = {
	.pwr_data = {
		.pwrlevel = {
			{
				.gpu_freq = 200000000,
				.bus_freq = 1,
			},
			{
				.gpu_freq = 200000000,
				.bus_freq = 0,
			},
		},
		.init_level = 0,
		.num_levels = 2,
		.set_grp_async = NULL,
		.idle_timeout = HZ/10,
#ifdef CONFIG_MSM_BUS_SCALING
		.nap_allowed = true,
#endif
		.pwrrail_first = true,
	},
	.clk = {
		.name = {
			.clk = "gfx2d1_clk",
			.pclk = "gfx2d1_pclk",
		},
#ifdef CONFIG_MSM_BUS_SCALING
		.bus_scale_table = &grp2d1_bus_scale_pdata,
#endif
	},
};

struct platform_device msm_kgsl_2d1 = {
	.name = "kgsl-2d1",
	.id = 1,
	.num_resources = ARRAY_SIZE(kgsl_2d1_resources),
	.resource = kgsl_2d1_resources,
	.dev = {
		.platform_data = &kgsl_2d1_pdata,
	},
};

/*
 * this a software workaround for not having two distinct board
 * files for 8660v1 and 8660v2. 8660v1 has a faulty 2d clock, and
 * this workaround detects the cpu version to tell if the kernel is on a
 * 8660v1, and should disable the 2d core. it is called from the board file
 */
void __init msm8x60_check_2d_hardware(void)
{
	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1) &&
	    (SOCINFO_VERSION_MINOR(socinfo_get_version()) == 0)) {
		printk(KERN_WARNING "kgsl: 2D cores disabled on 8660v1\n");
		kgsl_2d0_pdata.clk.name.clk = NULL;
		kgsl_2d1_pdata.clk.name.clk = NULL;
	}
}
#endif

/* Use GSBI3 QUP for /dev/i2c-0 */
struct platform_device msm_gsbi3_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI3_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi3_qup_i2c_resources),
	.resource	= gsbi3_qup_i2c_resources,
};

/* Use GSBI4 QUP for /dev/i2c-1 */
struct platform_device msm_gsbi4_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI4_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi4_qup_i2c_resources),
	.resource	= gsbi4_qup_i2c_resources,
};

/* Use GSBI8 QUP for /dev/i2c-3 */
struct platform_device msm_gsbi8_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI8_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi8_qup_i2c_resources),
	.resource	= gsbi8_qup_i2c_resources,
};

/* Use GSBI9 QUP for /dev/i2c-2 */
struct platform_device msm_gsbi9_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI9_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi9_qup_i2c_resources),
	.resource	= gsbi9_qup_i2c_resources,
};

/* Use GSBI7 QUP for /dev/i2c-4 (Marimba) */
struct platform_device msm_gsbi7_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI7_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi7_qup_i2c_resources),
	.resource	= gsbi7_qup_i2c_resources,
};

/* Use GSBI12 QUP for /dev/i2c-5 (Sensors) */
struct platform_device msm_gsbi12_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI12_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi12_qup_i2c_resources),
	.resource	= gsbi12_qup_i2c_resources,
};

#ifdef CONFIG_I2C_SSBI
/* 8058 PMIC SSBI on /dev/i2c-6 */
#define MSM_SSBI1_PMIC1C_PHYS	0x00500000
static struct resource msm_ssbi1_resources[] = {
	{
		.name   = "ssbi_base",
		.start	= MSM_SSBI1_PMIC1C_PHYS,
		.end	= MSM_SSBI1_PMIC1C_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi1 = {
	.name		= "i2c_ssbi",
	.id		= MSM_SSBI1_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(msm_ssbi1_resources),
	.resource	= msm_ssbi1_resources,
};

/* 8901 PMIC SSBI on /dev/i2c-7 */
#define MSM_SSBI2_PMIC2B_PHYS	0x00C00000
static struct resource msm_ssbi2_resources[] = {
	{
		.name   = "ssbi_base",
		.start	= MSM_SSBI2_PMIC2B_PHYS,
		.end	= MSM_SSBI2_PMIC2B_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi2 = {
	.name		= "i2c_ssbi",
	.id		= MSM_SSBI2_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(msm_ssbi2_resources),
	.resource	= msm_ssbi2_resources,
};

/* CODEC SSBI on /dev/i2c-8 */
#define MSM_SSBI3_PHYS  0x18700000
static struct resource msm_ssbi3_resources[] = {
	{
		.name   = "ssbi_base",
		.start  = MSM_SSBI3_PHYS,
		.end    = MSM_SSBI3_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi3 = {
	.name		= "i2c_ssbi",
	.id		= MSM_SSBI3_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(msm_ssbi3_resources),
	.resource	= msm_ssbi3_resources,
};
#endif /* CONFIG_I2C_SSBI */

static struct resource gsbi1_qup_spi_resources[] = {
	{
		.name	= "spi_base",
		.start	= MSM_GSBI1_QUP_PHYS,
		.end	= MSM_GSBI1_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_base",
		.start	= MSM_GSBI1_PHYS,
		.end	= MSM_GSBI1_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "spi_irq_in",
		.start	= GSBI1_QUP_IRQ,
		.end	= GSBI1_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spidm_channels",
		.start  = 5,
		.end    = 6,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.start  = 8,
		.end    = 7,
		.flags  = IORESOURCE_DMA,
	},
};

/* Use GSBI1 QUP for SPI-0 */
struct platform_device msm_gsbi1_qup_spi_device = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(gsbi1_qup_spi_resources),
	.resource	= gsbi1_qup_spi_resources,
};


static struct resource gsbi10_qup_spi_resources[] = {
	{
		.name	= "spi_base",
		.start	= MSM_GSBI10_QUP_PHYS,
		.end	= MSM_GSBI10_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_base",
		.start	= MSM_GSBI10_PHYS,
		.end	= MSM_GSBI10_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "spi_irq_in",
		.start	= GSBI10_QUP_IRQ,
		.end	= GSBI10_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/* Use GSBI10 QUP for SPI-1 */
struct platform_device msm_gsbi10_qup_spi_device = {
	.name		= "spi_qsd",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(gsbi10_qup_spi_resources),
	.resource	= gsbi10_qup_spi_resources,
};
#define MSM_SDC1_BASE         0x12400000
#define MSM_SDC2_BASE         0x12140000
#define MSM_SDC3_BASE         0x12180000
#define MSM_SDC4_BASE         0x121C0000
#define MSM_SDC5_BASE         0x12200000

static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_BASE,
		.end	= MSM_SDC1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC1_IRQ_0,
		.end	= SDC1_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC1_CHAN,
		.end	= DMOV_SDC1_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC1_CRCI,
		.end	= DMOV_SDC1_CRCI,
		.flags	= IORESOURCE_DMA,
	}
};

static struct resource resources_sdc2[] = {
	{
		.start	= MSM_SDC2_BASE,
		.end	= MSM_SDC2_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC2_IRQ_0,
		.end	= SDC2_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC2_CHAN,
		.end	= DMOV_SDC2_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC2_CRCI,
		.end	= DMOV_SDC2_CRCI,
		.flags	= IORESOURCE_DMA,
	}
};

static struct resource resources_sdc3[] = {
	{
		.start	= MSM_SDC3_BASE,
		.end	= MSM_SDC3_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC3_IRQ_0,
		.end	= SDC3_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC3_CHAN,
		.end	= DMOV_SDC3_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC3_CRCI,
		.end	= DMOV_SDC3_CRCI,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc4[] = {
	{
		.start	= MSM_SDC4_BASE,
		.end	= MSM_SDC4_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC4_IRQ_0,
		.end	= SDC4_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC4_CHAN,
		.end	= DMOV_SDC4_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC4_CRCI,
		.end	= DMOV_SDC4_CRCI,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc5[] = {
	{
		.start	= MSM_SDC5_BASE,
		.end	= MSM_SDC5_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC5_IRQ_0,
		.end	= SDC5_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC5_CHAN,
		.end	= DMOV_SDC5_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC5_CRCI,
		.end	= DMOV_SDC5_CRCI,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc2 = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_sdc2),
	.resource	= resources_sdc2,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc3 = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(resources_sdc3),
	.resource	= resources_sdc3,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc4 = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_sdc4),
	.resource	= resources_sdc4,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc5 = {
	.name		= "msm_sdcc",
	.id		= 5,
	.num_resources	= ARRAY_SIZE(resources_sdc5),
	.resource	= resources_sdc5,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
	&msm_device_sdc5,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 1 || controller > 5)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

#define MIPI_DSI_HW_BASE	0x04700000
#define ROTATOR_HW_BASE		0x04E00000
#define TVENC_HW_BASE		0x04F00000
#define MDP_HW_BASE		0x05100000

static struct resource msm_mipi_dsi_resources[] = {
	{
		.name   = "mipi_dsi",
		.start  = MIPI_DSI_HW_BASE,
		.end    = MIPI_DSI_HW_BASE + 0x000F0000 - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct platform_device msm_mipi_dsi_device = {
	.name   = "mipi_dsi",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mipi_dsi_resources),
	.resource       = msm_mipi_dsi_resources,
};

static struct resource msm_mdp_resources[] = {
	{
		.name   = "mdp",
		.start  = MDP_HW_BASE,
		.end    = MDP_HW_BASE + 0x000F0000 - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct platform_device msm_mdp_device = {
	.name   = "mdp",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mdp_resources),
	.resource       = msm_mdp_resources,
};
#ifdef CONFIG_MSM_ROTATOR
static struct resource resources_msm_rotator[] = {
	{
		.start	= 0x04E00000,
		.end	= 0x04F00000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= ROT_IRQ,
		.end	= ROT_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_rot_clocks rotator_clocks[] = {
	{
		.clk_name = "rot_clk",
		.clk_type = ROTATOR_AXI_CLK,
		.clk_rate = 160 * 1000 * 1000,
	},
	{
		.clk_name = "rotator_pclk",
		.clk_type = ROTATOR_PCLK,
		.clk_rate = 0,
	},
};

static struct msm_rotator_platform_data rotator_pdata = {
	.number_of_clocks = ARRAY_SIZE(rotator_clocks),
	.hardware_version_number = 0x01010307,
	.rotator_clks = rotator_clocks,
	.regulator_name = "fs_rot",
};

struct platform_device msm_rotator_device = {
	.name		= "msm_rotator",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(resources_msm_rotator),
	.resource       = resources_msm_rotator,
	.dev		= {
		.platform_data = &rotator_pdata,
	},
};
#endif


/* Sensors DSPS platform data */
#ifdef CONFIG_MSM_DSPS

#define PPSS_REG_PHYS_BASE	0x12080000

#define MHZ (1000*1000)

static struct dsps_clk_info dsps_clks[] = {
	{
		.name = "ppss_pclk",
		.rate =	0, /* no rate just on/off */
	},
	{
		.name = "pmem_clk",
		.rate =	0, /* no rate just on/off */
	},
	{
		.name = "gsbi_qup_clk",
		.rate =	24 * MHZ, /* See clk_tbl_gsbi_qup[] */
	},
	{
		.name = "dfab_dsps_clk",
		.rate =	64 * MHZ, /* Same rate as USB. */
	}
};

static struct dsps_regulator_info dsps_regs[] = {
	{
		.name = "8058_l5",
		.volt = 2850000, /* in uV */
	},
	{
		.name = "8058_s3",
		.volt = 1800000, /* in uV */
	}
};

/*
 * Note: GPIOs field is	intialized in run-time at the function
 * msm8x60_init_dsps().
 */

struct msm_dsps_platform_data msm_dsps_pdata = {
	.clks = dsps_clks,
	.clks_num = ARRAY_SIZE(dsps_clks),
	.gpios = NULL,
	.gpios_num = 0,
	.regs = dsps_regs,
	.regs_num = ARRAY_SIZE(dsps_regs),
	.signature = DSPS_SIGNATURE,
};

static struct resource msm_dsps_resources[] = {
	{
		.start = PPSS_REG_PHYS_BASE,
		.end   = PPSS_REG_PHYS_BASE + SZ_8K - 1,
		.name  = "ppss_reg",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_dsps_device = {
	.name          = "msm_dsps",
	.id            = 0,
	.num_resources = ARRAY_SIZE(msm_dsps_resources),
	.resource      = msm_dsps_resources,
	.dev.platform_data = &msm_dsps_pdata,
};

#endif /* CONFIG_MSM_DSPS */

#ifdef CONFIG_FB_MSM_TVOUT
static struct resource msm_tvenc_resources[] = {
	{
		.name   = "tvenc",
		.start  = TVENC_HW_BASE,
		.end    = TVENC_HW_BASE + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct resource tvout_device_resources[] = {
	{
		.name  = "tvout_device_irq",
		.start = TV_ENC_IRQ,
		.end   = TV_ENC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};
#endif
static void __init msm_register_device(struct platform_device *pdev, void *data)
{
	int ret;

	pdev->dev.platform_data = data;

	ret = platform_device_register(pdev);
	if (ret)
		dev_err(&pdev->dev,
			  "%s: platform_device_register() failed = %d\n",
			  __func__, ret);
}

static struct platform_device msm_lcdc_device = {
	.name   = "lcdc",
	.id     = 0,
};

#ifdef CONFIG_FB_MSM_TVOUT
static struct platform_device msm_tvenc_device = {
	.name   = "tvenc",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_tvenc_resources),
	.resource       = msm_tvenc_resources,
};

static struct platform_device msm_tvout_device = {
	.name = "tvout_device",
	.id = 0,
	.num_resources = ARRAY_SIZE(tvout_device_resources),
	.resource = tvout_device_resources,
};
#endif

#ifdef CONFIG_MSM_BUS_SCALING
static struct platform_device msm_dtv_device = {
	.name   = "dtv",
	.id     = 0,
};
#endif

void __init msm_fb_register_device(char *name, void *data)
{
	if (!strncmp(name, "mdp", 3))
		msm_register_device(&msm_mdp_device, data);
	else if (!strncmp(name, "lcdc", 4))
		msm_register_device(&msm_lcdc_device, data);
	else if (!strncmp(name, "mipi_dsi", 8))
		msm_register_device(&msm_mipi_dsi_device, data);
#ifdef CONFIG_FB_MSM_TVOUT
	else if (!strncmp(name, "tvenc", 5))
		msm_register_device(&msm_tvenc_device, data);
	else if (!strncmp(name, "tvout_device", 12))
		msm_register_device(&msm_tvout_device, data);
#endif
#ifdef CONFIG_MSM_BUS_SCALING
	else if (!strncmp(name, "dtv", 3))
		msm_register_device(&msm_dtv_device, data);
#endif
	else
		printk(KERN_ERR "%s: unknown device! %s\n", __func__, name);
}

static struct resource resources_otg[] = {
	{
		.start	= 0x12500000,
		.end	= 0x12500000 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= USB1_HS_IRQ,
		.end	= USB1_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_otg),
	.resource	= resources_otg,
};

static u64 dma_mask = 0xffffffffULL;
struct platform_device msm_device_gadget_peripheral = {
	.name		= "msm_hsusb",
	.id		= -1,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};
#ifdef CONFIG_USB_EHCI_MSM_72K
static struct resource resources_hsusb_host[] = {
	{
		.start	= 0x12500000,
		.end	= 0x12500000 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= USB1_HS_IRQ,
		.end	= USB1_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb_host = {
	.name		= "msm_hsusb_host",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_hsusb_host),
	.resource	= resources_hsusb_host,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

static struct platform_device *msm_host_devices[] = {
	&msm_device_hsusb_host,
};

int msm_add_host(unsigned int host, struct msm_usb_host_platform_data *plat)
{
	struct platform_device	*pdev;

	pdev = msm_host_devices[host];
	if (!pdev)
		return -ENODEV;
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}
#endif

#ifdef CONFIG_USB_ANDROID_DIAG
#define PID_MAGIC_ID		0x71432909
#define SERIAL_NUM_MAGIC_ID	0x61945374
#define SERIAL_NUMBER_LENGTH	127
#define DLOAD_USB_BASE_ADD	0x2A05F0C8

struct magic_num_struct {
	uint32_t pid;
	uint32_t serial_num;
};

struct dload_struct {
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
	uint16_t	reserved4;
	uint16_t	pid;
	char		serial_number[SERIAL_NUMBER_LENGTH];
	uint16_t	reserved5;
	struct magic_num_struct
			magic_struct;
};

static int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum)
{
	struct dload_struct __iomem *dload = 0;

	dload = ioremap(DLOAD_USB_BASE_ADD, sizeof(*dload));
	if (!dload) {
		pr_err("%s: cannot remap I/O memory region: %08x\n",
					__func__, DLOAD_USB_BASE_ADD);
		return -ENXIO;
	}

	pr_debug("%s: dload:%p pid:%x serial_num:%s\n",
				__func__, dload, pid, snum);
	/* update pid */
	dload->magic_struct.pid = PID_MAGIC_ID;
	dload->pid = pid;

	/* update serial number */
	dload->magic_struct.serial_num = 0;
	if (!snum)
		return 0;

	dload->magic_struct.serial_num = SERIAL_NUM_MAGIC_ID;
	strncpy(dload->serial_number, snum, SERIAL_NUMBER_LENGTH);
	dload->serial_number[SERIAL_NUMBER_LENGTH - 1] = '\0';

	iounmap(dload);

	return 0;
}

struct usb_diag_platform_data usb_diag_pdata = {
	.ch_name = DIAG_LEGACY,
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

struct platform_device usb_diag_device = {
	.name	= "usb_diag",
	.id	= 0,
	.dev	= {
		.platform_data = &usb_diag_pdata,
	},
};

struct usb_diag_platform_data usb_diag_mdm_pdata = {
	.ch_name = DIAG_MDM,
};

struct platform_device usb_diag_mdm_device = {
	.name	= "usb_diag",
	.id	= 1,
	.dev	= {
		.platform_data = &usb_diag_mdm_pdata,
	},
};
#endif

#define MSM_TSIF0_PHYS       (0x18200000)
#define MSM_TSIF1_PHYS       (0x18201000)
#define MSM_TSIF_SIZE        (0x200)
#define TCSR_ADM_0_A_CRCI_MUX_SEL 0x0070

#define TSIF_0_CLK       GPIO_CFG(93, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_EN        GPIO_CFG(94, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_DATA      GPIO_CFG(95, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_SYNC      GPIO_CFG(96, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_CLK       GPIO_CFG(97, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_EN        GPIO_CFG(98, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_DATA      GPIO_CFG(99, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_SYNC      GPIO_CFG(100, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif0_gpios[] = {
	{ .gpio_cfg = TSIF_0_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_0_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_0_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_0_SYNC, .label =  "tsif_sync", },
};

static const struct msm_gpio tsif1_gpios[] = {
	{ .gpio_cfg = TSIF_1_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_1_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_1_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_1_SYNC, .label =  "tsif_sync", },
};

static void tsif_release(struct device *dev)
{
}

static void tsif_init1(struct msm_tsif_platform_data *data)
{
	int val;

	/* configure mux to use correct tsif instance */
	val = secure_readl(MSM_TCSR_BASE + TCSR_ADM_0_A_CRCI_MUX_SEL);
	val |= 0x80000000;
	secure_writel(val, MSM_TCSR_BASE + TCSR_ADM_0_A_CRCI_MUX_SEL);
}

struct msm_tsif_platform_data tsif1_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif1_gpios),
	.gpios = tsif1_gpios,
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
	.init = tsif_init1
};

struct resource tsif1_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = TSIF2_IRQ,
		.end   = TSIF2_IRQ,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF1_PHYS,
		.end   = MSM_TSIF1_PHYS + MSM_TSIF_SIZE - 1,
	},
	[2] = {
		.flags = IORESOURCE_DMA,
		.start = DMOV_TSIF_CHAN,
		.end   = DMOV_TSIF_CRCI,
	},
};

static void tsif_init0(struct msm_tsif_platform_data *data)
{
	int val;

	/* configure mux to use correct tsif instance */
	val = secure_readl(MSM_TCSR_BASE + TCSR_ADM_0_A_CRCI_MUX_SEL);
	val &= 0x7FFFFFFF;
	secure_writel(val, MSM_TCSR_BASE + TCSR_ADM_0_A_CRCI_MUX_SEL);
}

struct msm_tsif_platform_data tsif0_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif0_gpios),
	.gpios = tsif0_gpios,
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
	.init = tsif_init0
};
struct resource tsif0_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = TSIF1_IRQ,
		.end   = TSIF1_IRQ,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF0_PHYS,
		.end   = MSM_TSIF0_PHYS + MSM_TSIF_SIZE - 1,
	},
	[2] = {
		.flags = IORESOURCE_DMA,
		.start = DMOV_TSIF_CHAN,
		.end   = DMOV_TSIF_CRCI,
	},
};

struct platform_device msm_device_tsif[2] = {
	{
		.name          = "msm_tsif",
		.id            = 0,
		.num_resources = ARRAY_SIZE(tsif0_resources),
		.resource      = tsif0_resources,
		.dev = {
			.release       = tsif_release,
			.platform_data = &tsif0_platform_data
		},
	},
	{
		.name          = "msm_tsif",
		.id            = 1,
		.num_resources = ARRAY_SIZE(tsif1_resources),
		.resource      = tsif1_resources,
		.dev = {
			.release       = tsif_release,
			.platform_data = &tsif1_platform_data
		},
	}
};

#ifdef CONFIG_USB_F_SERIAL
static struct usb_gadget_fserial_platform_data fserial_pdata = {
	.no_ports	= 2,
};

struct platform_device usb_gadget_fserial_device = {
	.name	= "usb_fserial",
	.id	= -1,
	.dev	= {
		.platform_data = &fserial_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID_ACM
static struct usb_gadget_facm_pdata facm_pdata = {
	.no_ports	= 2,
};

struct platform_device usb_gadget_facm_device = {
	.name	= "usb_facm",
	.id	= -1,
	.dev	= {
		.platform_data = &facm_pdata,
	},
};
#endif

struct platform_device msm_device_smd = {
	.name           = "msm_smd",
	.id             = -1,
};

struct resource msm_dmov_resource_adm0[] = {
	{
		.start = INT_ADM0_AARM,
		.end = (resource_size_t)MSM_DMOV_ADM0_BASE,
		.flags = IORESOURCE_IRQ,
	},
};

struct resource msm_dmov_resource_adm1[] = {
	{
		.start = INT_ADM1_AARM,
		.end = (resource_size_t)MSM_DMOV_ADM1_BASE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_dmov_adm0 = {
	.name	= "msm_dmov",
	.id	= 0,
	.resource = msm_dmov_resource_adm0,
	.num_resources = ARRAY_SIZE(msm_dmov_resource_adm0),
};

struct platform_device msm_device_dmov_adm1 = {
	.name	= "msm_dmov",
	.id	= 1,
	.resource = msm_dmov_resource_adm1,
	.num_resources = ARRAY_SIZE(msm_dmov_resource_adm1),
};

/* MSM Video core device */

#define MSM_VIDC_BASE_PHYS 0x04400000
#define MSM_VIDC_BASE_SIZE 0x00100000

static struct resource msm_device_vidc_resources[] = {
	{
		.start	= MSM_VIDC_BASE_PHYS,
		.end	= MSM_VIDC_BASE_PHYS + MSM_VIDC_BASE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VCODEC_IRQ,
		.end	= VCODEC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_vidc = {
	.name = "msm_vidc",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_device_vidc_resources),
	.resource = msm_device_vidc_resources,
};

#if defined(CONFIG_MSM_RPM_STATS_LOG)
static struct msm_rpmstats_platform_data msm_rpm_stat_pdata = {
	.phys_addr_base = 0x00107E04,
	.phys_size = SZ_8K,
};

struct platform_device msm_rpm_stat_device = {
	.name = "msm_rpm_stat",
	.id = -1,
	.dev = {
		.platform_data = &msm_rpm_stat_pdata,
	},
};
#endif

#ifdef CONFIG_MSM_MPM
static uint16_t msm_mpm_irqs_m2a[MSM_MPM_NR_MPM_IRQS] = {
	[1] = MSM_GPIO_TO_INT(61),
	[4] = MSM_GPIO_TO_INT(87),
	[5] = MSM_GPIO_TO_INT(88),
	[6] = MSM_GPIO_TO_INT(89),
	[7] = MSM_GPIO_TO_INT(90),
	[8] = MSM_GPIO_TO_INT(91),
	[9] = MSM_GPIO_TO_INT(34),
	[10] = MSM_GPIO_TO_INT(38),
	[11] = MSM_GPIO_TO_INT(42),
	[12] = MSM_GPIO_TO_INT(46),
	[13] = MSM_GPIO_TO_INT(50),
	[14] = MSM_GPIO_TO_INT(54),
	[15] = MSM_GPIO_TO_INT(58),
	[16] = MSM_GPIO_TO_INT(63),
	[17] = MSM_GPIO_TO_INT(160),
	[18] = MSM_GPIO_TO_INT(162),
	[19] = MSM_GPIO_TO_INT(144),
	[20] = MSM_GPIO_TO_INT(146),
	[25] = USB1_HS_IRQ,
	[26] = TV_ENC_IRQ,
	[27] = HDMI_IRQ,
	[29] = MSM_GPIO_TO_INT(123),
	[30] = MSM_GPIO_TO_INT(172),
	[31] = MSM_GPIO_TO_INT(99),
	[32] = MSM_GPIO_TO_INT(96),
	[33] = MSM_GPIO_TO_INT(67),
	[34] = MSM_GPIO_TO_INT(71),
	[35] = MSM_GPIO_TO_INT(105),
	[36] = MSM_GPIO_TO_INT(117),
	[37] = MSM_GPIO_TO_INT(29),
	[38] = MSM_GPIO_TO_INT(30),
	[39] = MSM_GPIO_TO_INT(31),
	[40] = MSM_GPIO_TO_INT(37),
	[41] = MSM_GPIO_TO_INT(40),
	[42] = MSM_GPIO_TO_INT(41),
	[43] = MSM_GPIO_TO_INT(45),
	[44] = MSM_GPIO_TO_INT(51),
	[45] = MSM_GPIO_TO_INT(52),
	[46] = MSM_GPIO_TO_INT(57),
	[47] = MSM_GPIO_TO_INT(73),
	[48] = MSM_GPIO_TO_INT(93),
	[49] = MSM_GPIO_TO_INT(94),
	[50] = MSM_GPIO_TO_INT(103),
	[51] = MSM_GPIO_TO_INT(104),
	[52] = MSM_GPIO_TO_INT(106),
	[53] = MSM_GPIO_TO_INT(115),
	[54] = MSM_GPIO_TO_INT(124),
	[55] = MSM_GPIO_TO_INT(125),
	[56] = MSM_GPIO_TO_INT(126),
	[57] = MSM_GPIO_TO_INT(127),
	[58] = MSM_GPIO_TO_INT(128),
	[59] = MSM_GPIO_TO_INT(129),
};

static uint16_t msm_mpm_bypassed_apps_irqs[] = {
	TLMM_MSM_SUMMARY_IRQ,
	RPM_SCSS_CPU0_GP_HIGH_IRQ,
	RPM_SCSS_CPU0_GP_MEDIUM_IRQ,
	RPM_SCSS_CPU0_GP_LOW_IRQ,
	RPM_SCSS_CPU0_WAKE_UP_IRQ,
	RPM_SCSS_CPU1_GP_HIGH_IRQ,
	RPM_SCSS_CPU1_GP_MEDIUM_IRQ,
	RPM_SCSS_CPU1_GP_LOW_IRQ,
	RPM_SCSS_CPU1_WAKE_UP_IRQ,
	MARM_SCSS_GP_IRQ_0,
	MARM_SCSS_GP_IRQ_1,
	MARM_SCSS_GP_IRQ_2,
	MARM_SCSS_GP_IRQ_3,
	MARM_SCSS_GP_IRQ_4,
	MARM_SCSS_GP_IRQ_5,
	MARM_SCSS_GP_IRQ_6,
	MARM_SCSS_GP_IRQ_7,
	MARM_SCSS_GP_IRQ_8,
	MARM_SCSS_GP_IRQ_9,
	LPASS_SCSS_GP_LOW_IRQ,
	LPASS_SCSS_GP_MEDIUM_IRQ,
	LPASS_SCSS_GP_HIGH_IRQ,
	SDC4_IRQ_0,
	SPS_MTI_31,
};

struct msm_mpm_device_data msm_mpm_dev_data = {
	.irqs_m2a = msm_mpm_irqs_m2a,
	.irqs_m2a_size = ARRAY_SIZE(msm_mpm_irqs_m2a),
	.bypassed_apps_irqs = msm_mpm_bypassed_apps_irqs,
	.bypassed_apps_irqs_size = ARRAY_SIZE(msm_mpm_bypassed_apps_irqs),
	.mpm_request_reg_base = MSM_RPM_BASE + 0x9d8,
	.mpm_status_reg_base = MSM_RPM_BASE + 0xdf8,
	.mpm_apps_ipc_reg = MSM_GCC_BASE + 0x008,
	.mpm_apps_ipc_val =  BIT(1),
	.mpm_ipc_irq = RPM_SCSS_CPU0_GP_MEDIUM_IRQ,

};
#endif


#ifdef CONFIG_MSM_BUS_SCALING
struct platform_device msm_bus_sys_fabric = {
	.name  = "msm_bus_fabric",
	.id    =  MSM_BUS_FAB_SYSTEM,
};
struct platform_device msm_bus_apps_fabric = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_APPSS,
};
struct platform_device msm_bus_mm_fabric = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_MMSS,
};
struct platform_device msm_bus_sys_fpb = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_SYSTEM_FPB,
};
struct platform_device msm_bus_cpss_fpb = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_CPSS_FPB,
};
#endif

#define FS(_id, _name) (&(struct platform_device){ \
	.name	= "footswitch-msm8x60", \
	.id	= (_id), \
	.dev	= { \
		.platform_data = &(struct regulator_init_data){ \
			.constraints = { \
				.valid_modes_mask = REGULATOR_MODE_NORMAL, \
				.valid_ops_mask   = REGULATOR_CHANGE_STATUS, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = \
				&(struct regulator_consumer_supply) \
				REGULATOR_SUPPLY((_name), NULL), \
		} \
	}, \
})
struct platform_device *msm_footswitch_devices[] = {
	FS(FS_IJPEG,	"fs_ijpeg"),
	FS(FS_MDP,	"fs_mdp"),
	FS(FS_ROT,	"fs_rot"),
	FS(FS_VED,	"fs_ved"),
	FS(FS_VFE,	"fs_vfe"),
	FS(FS_VPE,	"fs_vpe"),
	FS(FS_GFX3D,	"fs_gfx3d"),
	FS(FS_GFX2D0,	"fs_gfx2d0"),
	FS(FS_GFX2D1,	"fs_gfx2d1"),
};
unsigned msm_num_footswitch_devices = ARRAY_SIZE(msm_footswitch_devices);

struct clk_lookup msm_clocks_8x60[] = {
	CLK_RPM("afab_clk",		AFAB_CLK,		NULL, CLK_MIN),
	CLK_RPM("afab_a_clk",		AFAB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("cfpb_clk",		CFPB_CLK,		NULL, CLK_MIN),
	CLK_RPM("cfpb_a_clk",		CFPB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("dfab_clk",		DFAB_CLK,		NULL, CLK_MIN),
	CLK_RPM("dfab_a_clk",		DFAB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("ebi1_clk",		EBI1_CLK,		NULL, CLK_MIN),
	CLK_RPM("ebi1_a_clk",		EBI1_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("mmfab_clk",		MMFAB_CLK,		NULL, CLK_MIN),
	CLK_RPM("mmfab_a_clk",		MMFAB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("mmfpb_clk",		MMFPB_CLK,		NULL, CLK_MIN),
	CLK_RPM("mmfpb_a_clk",		MMFPB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("sfab_clk",		SFAB_CLK,		NULL, CLK_MIN),
	CLK_RPM("sfab_a_clk",		SFAB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("sfpb_clk",		SFPB_CLK,		NULL, CLK_MIN),
	CLK_RPM("sfpb_a_clk",		SFPB_A_CLK,		NULL, CLK_MIN),
	CLK_RPM("smi_clk",		SMI_CLK,		NULL, CLK_MIN),
	CLK_RPM("smi_a_clk",		SMI_A_CLK,		NULL, CLK_MIN),

	CLK_8X60("gsbi_uart_clk",	GSBI1_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI2_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI3_UART_CLK, "msm_serial_hsl.2",
			OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI4_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI5_UART_CLK,		NULL, OFF),
	CLK_8X60("uartdm_clk",	GSBI6_UART_CLK, "msm_serial_hs.0", OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI7_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI8_UART_CLK,		NULL, OFF),
	CLK_8X60("gsbi_uart_clk", GSBI9_UART_CLK, "msm_serial_hsl.1", OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI10_UART_CLK,	NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI11_UART_CLK,	NULL, OFF),
	CLK_8X60("gsbi_uart_clk",	GSBI12_UART_CLK, "msm_serial_hsl.0",
			OFF),
	CLK_8X60("spi_clk",		GSBI1_QUP_CLK, "spi_qsd.0", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI2_QUP_CLK,		NULL, OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI3_QUP_CLK, "qup_i2c.0", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI4_QUP_CLK, "qup_i2c.1", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI5_QUP_CLK,		NULL, OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI6_QUP_CLK,		NULL, OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI7_QUP_CLK, "qup_i2c.4", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI8_QUP_CLK, "qup_i2c.3", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI9_QUP_CLK, "qup_i2c.2", OFF),
	CLK_8X60("spi_clk",		GSBI10_QUP_CLK,	"spi_qsd.1", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI11_QUP_CLK,		NULL, OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI12_QUP_CLK,	"msm_dsps.0", OFF),
	CLK_8X60("gsbi_qup_clk",	GSBI12_QUP_CLK, "qup_i2c.5", OFF),
	CLK_8X60("pdm_clk",		PDM_CLK,		NULL, OFF),
	CLK_8X60("pmem_clk",		PMEM_CLK,		NULL, OFF),
	CLK_8X60("prng_clk",		PRNG_CLK,		NULL, OFF),
	CLK_8X60("sdc_clk",		SDC1_CLK, "msm_sdcc.1", OFF),
	CLK_8X60("sdc_clk",		SDC2_CLK, "msm_sdcc.2", OFF),
	CLK_8X60("sdc_clk",		SDC3_CLK, "msm_sdcc.3", OFF),
	CLK_8X60("sdc_clk",		SDC4_CLK, "msm_sdcc.4", OFF),
	CLK_8X60("sdc_clk",		SDC5_CLK, "msm_sdcc.5", OFF),
	CLK_8X60("tsif_ref_clk",	TSIF_REF_CLK,		NULL, OFF),
	CLK_8X60("tssc_clk",		TSSC_CLK,		NULL, OFF),
	CLK_8X60("usb_hs_clk",		USB_HS1_XCVR_CLK,	NULL, OFF),
	CLK_8X60("usb_phy_clk",		USB_PHY0_CLK,		NULL, OFF),
	CLK_8X60("usb_fs_clk",		USB_FS1_XCVR_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_sys_clk",	USB_FS1_SYS_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_src_clk",	USB_FS1_SRC_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_clk",		USB_FS2_XCVR_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_sys_clk",	USB_FS2_SYS_CLK,	NULL, OFF),
	CLK_8X60("usb_fs_src_clk",	USB_FS2_SRC_CLK,	NULL, OFF),
	CLK_8X60("ce_clk",		CE2_P_CLK,		NULL, OFF),
	CLK_8X60("spi_pclk",		GSBI1_P_CLK, "spi_qsd.0", OFF),
	CLK_8X60("gsbi_pclk",		GSBI2_P_CLK,		NULL, OFF),
	CLK_8X60("gsbi_pclk",		GSBI3_P_CLK, "msm_serial_hsl.2", 0),
	CLK_8X60("gsbi_pclk",		GSBI3_P_CLK, "qup_i2c.0", OFF),
	CLK_8X60("gsbi_pclk",		GSBI4_P_CLK, "qup_i2c.1", OFF),
	CLK_8X60("gsbi_pclk",		GSBI5_P_CLK,		NULL, OFF),
	CLK_8X60("uartdm_pclk",		GSBI6_P_CLK, "msm_serial_hs.0", OFF),
	CLK_8X60("gsbi_pclk",		GSBI7_P_CLK, "qup_i2c.4", OFF),
	CLK_8X60("gsbi_pclk",		GSBI8_P_CLK, "qup_i2c.3", OFF),
	CLK_8X60("gsbi_pclk",		GSBI9_P_CLK, "msm_serial_hsl.1", OFF),
	CLK_8X60("gsbi_pclk",		GSBI9_P_CLK, "qup_i2c.2", OFF),
	CLK_8X60("spi_pclk",		GSBI10_P_CLK, "spi_qsd.1", OFF),
	CLK_8X60("gsbi_pclk",		GSBI11_P_CLK,		NULL, OFF),
	CLK_8X60("gsbi_pclk",		GSBI12_P_CLK, "msm_dsps.0", 0),
	CLK_8X60("gsbi_pclk",		GSBI12_P_CLK, "msm_serial_hsl.0", 0),
	CLK_8X60("gsbi_pclk",		GSBI12_P_CLK, "qup_i2c.5", 0),
	CLK_8X60("ppss_pclk",		PPSS_P_CLK,		NULL, OFF),
	CLK_8X60("tsif_pclk",		TSIF_P_CLK,		NULL, OFF),
	CLK_8X60("usb_fs_pclk",		USB_FS1_P_CLK,		NULL, OFF),
	CLK_8X60("usb_fs_pclk",		USB_FS2_P_CLK,		NULL, OFF),
	CLK_8X60("usb_hs_pclk",		USB_HS1_P_CLK,		NULL, OFF),
	CLK_8X60("sdc_pclk",		SDC1_P_CLK, "msm_sdcc.1", OFF),
	CLK_8X60("sdc_pclk",		SDC2_P_CLK, "msm_sdcc.2", OFF),
	CLK_8X60("sdc_pclk",		SDC3_P_CLK, "msm_sdcc.3", OFF),
	CLK_8X60("sdc_pclk",		SDC4_P_CLK, "msm_sdcc.4", OFF),
	CLK_8X60("sdc_pclk",		SDC5_P_CLK, "msm_sdcc.5", OFF),
	CLK_8X60("adm_clk",		ADM0_CLK, "msm_dmov.0", OFF),
	CLK_8X60("adm_pclk",		ADM0_P_CLK, "msm_dmov.0", OFF),
	CLK_8X60("adm_clk",		ADM1_CLK, "msm_dmov.1", OFF),
	CLK_8X60("adm_pclk",		ADM1_P_CLK, "msm_dmov.1", OFF),
	CLK_8X60("modem_ahb1_pclk",	MODEM_AHB1_P_CLK,	NULL, OFF),
	CLK_8X60("modem_ahb2_pclk",	MODEM_AHB2_P_CLK,	NULL, OFF),
	CLK_8X60("pmic_arb_pclk",	PMIC_ARB0_P_CLK,	NULL, OFF),
	CLK_8X60("pmic_arb_pclk",	PMIC_ARB1_P_CLK,	NULL, OFF),
	CLK_8X60("pmic_ssbi2",		PMIC_SSBI2_CLK,		NULL, OFF),
	CLK_8X60("rpm_msg_ram_pclk",	RPM_MSG_RAM_P_CLK,	NULL, OFF),
	CLK_8X60("amp_clk",		AMP_CLK,		NULL, OFF),
	CLK_8X60("cam_clk",		CAM_CLK,		NULL, OFF),
	CLK_8X60("csi_clk",		CSI0_CLK,		NULL, OFF),
	CLK_8X60("csi_clk",		CSI1_CLK,	  "msm_camera_ov7692.0",
			OFF),
	CLK_8X60("csi_clk",		CSI1_CLK,	  "msm_camera_ov9726.0",
			OFF),
	CLK_8X60("csi_src_clk",		CSI_SRC_CLK,		NULL, OFF),
	CLK_8X60("dsi_byte_div_clk",	DSI_BYTE_CLK,		NULL, OFF),
	CLK_8X60("dsi_esc_clk",		DSI_ESC_CLK,		NULL, OFF),
	CLK_8X60("gfx2d0_clk",		GFX2D0_CLK,		NULL, OFF),
	CLK_8X60("gfx2d1_clk",		GFX2D1_CLK,		NULL, OFF),
	CLK_8X60("gfx3d_clk",		GFX3D_CLK,		NULL, OFF),
	CLK_8X60("ijpeg_clk",		IJPEG_CLK,		NULL, OFF),
	CLK_8X60("jpegd_clk",		JPEGD_CLK,		NULL, OFF),
	CLK_8X60("mdp_clk",		MDP_CLK,		NULL, OFF),
	CLK_8X60("mdp_vsync_clk",	MDP_VSYNC_CLK,		NULL, OFF),
	CLK_8X60("pixel_lcdc_clk",	PIXEL_LCDC_CLK,		NULL, OFF),
	CLK_8X60("pixel_mdp_clk",	PIXEL_MDP_CLK,		NULL, OFF),
	CLK_8X60("rot_clk",		ROT_CLK,		NULL, OFF),
	CLK_8X60("tv_enc_clk",		TV_ENC_CLK,		NULL, OFF),
	CLK_8X60("tv_dac_clk",		TV_DAC_CLK,		NULL, OFF),
	CLK_8X60("vcodec_clk",		VCODEC_CLK,		NULL, OFF),
	CLK_8X60("mdp_tv_clk",		MDP_TV_CLK,		NULL, OFF),
	CLK_8X60("hdmi_clk",		HDMI_TV_CLK,		NULL, OFF),
	CLK_8X60("tv_src_clk",		TV_SRC_CLK,		NULL, OFF),
	CLK_8X60("hdmi_app_clk",	HDMI_APP_CLK,		NULL, OFF),
	CLK_8X60("vpe_clk",		VPE_CLK,		NULL, OFF),
	CLK_8X60("csi_vfe_clk",		CSI0_VFE_CLK,		NULL, OFF),
	CLK_8X60("csi_vfe_clk",		CSI1_VFE_CLK,	  "msm_camera_ov7692.0",
			OFF),
	CLK_8X60("csi_vfe_clk",		CSI1_VFE_CLK,	  "msm_camera_ov9726.0",
			OFF),
	CLK_8X60("vfe_clk",		VFE_CLK,		NULL, OFF),
	CLK_8X60("smmu_jpegd_clk",	JPEGD_AXI_CLK,		NULL, OFF),
	CLK_8X60("smmu_vfe_clk",	VFE_AXI_CLK,		NULL, OFF),
	CLK_8X60("vfe_axi_clk",		VFE_AXI_CLK,		NULL, OFF),
	CLK_8X60("ijpeg_axi_clk",	IJPEG_AXI_CLK,		NULL, OFF),
	CLK_8X60("imem_axi_clk",	IMEM_AXI_CLK,		NULL, OFF),
	CLK_8X60("mdp_axi_clk",		MDP_AXI_CLK,		NULL, OFF),
	CLK_8X60("rot_axi_clk",		ROT_AXI_CLK,		NULL, OFF),
	CLK_8X60("vcodec_axi_clk",	VCODEC_AXI_CLK,		NULL, OFF),
	CLK_8X60("vpe_axi_clk",		VPE_AXI_CLK,		NULL, OFF),
	CLK_8X60("amp_pclk",		AMP_P_CLK,		NULL, OFF),
	CLK_8X60("csi_pclk",		CSI0_P_CLK,		NULL, OFF),
	CLK_8X60("csi_pclk",		CSI1_P_CLK,	  "msm_camera_ov7692.0",
			OFF),
	CLK_8X60("csi_pclk",		CSI1_P_CLK,	  "msm_camera_ov9726.0",
			OFF),
	CLK_8X60("dsi_m_pclk",		DSI_M_P_CLK,		NULL, OFF),
	CLK_8X60("dsi_s_pclk",		DSI_S_P_CLK,		NULL, OFF),
	CLK_8X60("gfx2d0_pclk",		GFX2D0_P_CLK,		NULL, OFF),
	CLK_8X60("gfx2d1_pclk",		GFX2D1_P_CLK,		NULL, OFF),
	CLK_8X60("gfx3d_pclk",		GFX3D_P_CLK,		NULL, OFF),
	CLK_8X60("hdmi_m_pclk",		HDMI_M_P_CLK,		NULL, OFF),
	CLK_8X60("hdmi_s_pclk",		HDMI_S_P_CLK,		NULL, OFF),
	CLK_8X60("ijpeg_pclk",		IJPEG_P_CLK,		NULL, OFF),
	CLK_8X60("jpegd_pclk",		JPEGD_P_CLK,		NULL, OFF),
	CLK_8X60("imem_pclk",		IMEM_P_CLK,		NULL, OFF),
	CLK_8X60("mdp_pclk",		MDP_P_CLK,		NULL, OFF),
	CLK_8X60("smmu_pclk",		SMMU_P_CLK,		NULL, OFF),
	CLK_8X60("rotator_pclk",	ROT_P_CLK,		NULL, OFF),
	CLK_8X60("tv_enc_pclk",		TV_ENC_P_CLK,		NULL, OFF),
	CLK_8X60("vcodec_pclk",		VCODEC_P_CLK,		NULL, OFF),
	CLK_8X60("vfe_pclk",		VFE_P_CLK,		NULL, OFF),
	CLK_8X60("vpe_pclk",		VPE_P_CLK,		NULL, OFF),
	CLK_8X60("mi2s_osr_clk",	MI2S_OSR_CLK,		NULL, OFF),
	CLK_8X60("mi2s_bit_clk",	MI2S_BIT_CLK,		NULL, OFF),
	CLK_8X60("i2s_mic_osr_clk",	CODEC_I2S_MIC_OSR_CLK,	NULL, OFF),
	CLK_8X60("i2s_mic_bit_clk",	CODEC_I2S_MIC_BIT_CLK,	NULL, OFF),
	CLK_8X60("i2s_mic_osr_clk",	SPARE_I2S_MIC_OSR_CLK,	NULL, OFF),
	CLK_8X60("i2s_mic_bit_clk",	SPARE_I2S_MIC_BIT_CLK,	NULL, OFF),
	CLK_8X60("i2s_spkr_osr_clk",	CODEC_I2S_SPKR_OSR_CLK,	NULL, OFF),
	CLK_8X60("i2s_spkr_bit_clk",	CODEC_I2S_SPKR_BIT_CLK,	NULL, OFF),
	CLK_8X60("i2s_spkr_osr_clk",	SPARE_I2S_SPKR_OSR_CLK,	NULL, OFF),
	CLK_8X60("i2s_spkr_bit_clk",	SPARE_I2S_SPKR_BIT_CLK,	NULL, OFF),
	CLK_8X60("pcm_clk",		PCM_CLK,		NULL, OFF),
	CLK_8X60("iommu_clk",           JPEGD_AXI_CLK, "msm_iommu.0", 0),
	CLK_8X60("iommu_clk",           IJPEG_AXI_CLK, "msm_iommu.5", 0),
	CLK_8X60("iommu_clk",           VFE_AXI_CLK, "msm_iommu.6", 0),
	CLK_8X60("iommu_clk",           VCODEC_AXI_CLK, "msm_iommu.7", 0),
	CLK_8X60("iommu_clk",           VCODEC_AXI_CLK, "msm_iommu.8", 0),
	CLK_8X60("iommu_clk",           GFX3D_CLK, "msm_iommu.9", 0),
	CLK_8X60("iommu_clk",           GFX2D0_CLK, "msm_iommu.10", 0),
	CLK_8X60("iommu_clk",           GFX2D1_CLK, "msm_iommu.11", 0),

	CLK_VOTER("dfab_dsps_clk",     DFAB_DSPS_CLK,
					"dfab_clk",    NULL, 0),
	CLK_VOTER("dfab_usb_hs_clk",   DFAB_USB_HS_CLK,
					"dfab_clk",    NULL, 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC1_CLK,
					"dfab_clk",    "msm_sdcc.1", 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC2_CLK,
					"dfab_clk",    "msm_sdcc.2", 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC3_CLK,
					"dfab_clk",    "msm_sdcc.3", 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC4_CLK,
					"dfab_clk",    "msm_sdcc.4", 0),
	CLK_VOTER("dfab_sdc_clk",      DFAB_SDC5_CLK,
					"dfab_clk",    "msm_sdcc.5", 0),
	CLK_VOTER("ebi1_msmbus_clk",   EBI_MSMBUS_CLK,
					"ebi1_clk",    NULL, 0),
	CLK_VOTER("ebi1_adm_clk",     EBI_ADM0_CLK,
					"ebi1_clk",    "msm_dmov.0", 0),
	CLK_VOTER("ebi1_adm_clk",     EBI_ADM1_CLK,
					"ebi1_clk",    "msm_dmov.1", 0),

	CLK_8X60("sc0_div2_mclk",  SC0_DIV2_M_CLK,  NULL, 0),
	CLK_8X60("sc1_div2_mclk",  SC1_DIV2_M_CLK,  NULL, 0),
	CLK_8X60("l2_div2_mclk",   L2_DIV2_M_CLK,   NULL, 0),
	CLK_8X60("afab_mclk",      AFAB_M_CLK,      NULL, 0),
	CLK_8X60("sfab_mclk",      SFAB_M_CLK,      NULL, 0),
	CLK_8X60("ebi1_2x_mclk",   EBI1_2X_M_CLK,   NULL, 0),
	CLK_8X60("cfpb0_mclk",     CFPB0_M_CLK,     NULL, 0),
	CLK_8X60("cfpb1_mclk",     CFPB1_M_CLK,     NULL, 0),
	CLK_8X60("cfpb2_mclk",     CFPB2_M_CLK,     NULL, 0),
	CLK_8X60("dfab_mclk",      DFAB_M_CLK,      NULL, 0),
	CLK_8X60("sfpb_mclk",      SFPB_M_CLK,      NULL, 0),
	CLK_8X60("mmfab_mclk",     MMFAB_M_CLK,     NULL, 0),
	CLK_8X60("smi_ddr2x_mclk", SMI_DDR2X_M_CLK, NULL, 0),
	CLK_8X60("mmfpb_mclk",     MMFPB_M_CLK,     NULL, 0),
};

unsigned msm_num_clocks_8x60 = ARRAY_SIZE(msm_clocks_8x60);

#ifdef CONFIG_MSM_RPM
struct msm_rpm_map_data rpm_map_data[] __initdata = {
	MSM_RPM_MAP(TRIGGER_TIMED_TO, TRIGGER_TIMED, 1),
	MSM_RPM_MAP(TRIGGER_TIMED_SCLK_COUNT, TRIGGER_TIMED, 1),
	MSM_RPM_MAP(TRIGGER_SET_FROM, TRIGGER_SET, 1),
	MSM_RPM_MAP(TRIGGER_SET_TO, TRIGGER_SET, 1),
	MSM_RPM_MAP(TRIGGER_SET_TRIGGER, TRIGGER_SET, 1),
	MSM_RPM_MAP(TRIGGER_CLEAR_FROM, TRIGGER_CLEAR, 1),
	MSM_RPM_MAP(TRIGGER_CLEAR_TO, TRIGGER_CLEAR, 1),
	MSM_RPM_MAP(TRIGGER_CLEAR_TRIGGER, TRIGGER_CLEAR, 1),

	MSM_RPM_MAP(CXO_CLK, CXO_CLK, 1),
	MSM_RPM_MAP(PXO_CLK, PXO_CLK, 1),
	MSM_RPM_MAP(PLL_4, PLL_4, 1),
	MSM_RPM_MAP(APPS_FABRIC_CLK, APPS_FABRIC_CLK, 1),
	MSM_RPM_MAP(SYSTEM_FABRIC_CLK, SYSTEM_FABRIC_CLK, 1),
	MSM_RPM_MAP(MM_FABRIC_CLK, MM_FABRIC_CLK, 1),
	MSM_RPM_MAP(DAYTONA_FABRIC_CLK, DAYTONA_FABRIC_CLK, 1),
	MSM_RPM_MAP(SFPB_CLK, SFPB_CLK, 1),
	MSM_RPM_MAP(CFPB_CLK, CFPB_CLK, 1),
	MSM_RPM_MAP(MMFPB_CLK, MMFPB_CLK, 1),
	MSM_RPM_MAP(SMI_CLK, SMI_CLK, 1),
	MSM_RPM_MAP(EBI1_CLK, EBI1_CLK, 1),

	MSM_RPM_MAP(APPS_L2_CACHE_CTL, APPS_L2_CACHE_CTL, 1),

	MSM_RPM_MAP(APPS_FABRIC_HALT_0, APPS_FABRIC_HALT, 2),
	MSM_RPM_MAP(APPS_FABRIC_CLOCK_MODE_0, APPS_FABRIC_CLOCK_MODE, 3),
	MSM_RPM_MAP(APPS_FABRIC_ARB_0, APPS_FABRIC_ARB, 6),

	MSM_RPM_MAP(SYSTEM_FABRIC_HALT_0, SYSTEM_FABRIC_HALT, 2),
	MSM_RPM_MAP(SYSTEM_FABRIC_CLOCK_MODE_0, SYSTEM_FABRIC_CLOCK_MODE, 3),
	MSM_RPM_MAP(SYSTEM_FABRIC_ARB_0, SYSTEM_FABRIC_ARB, 22),

	MSM_RPM_MAP(MM_FABRIC_HALT_0, MM_FABRIC_HALT, 2),
	MSM_RPM_MAP(MM_FABRIC_CLOCK_MODE_0, MM_FABRIC_CLOCK_MODE, 3),
	MSM_RPM_MAP(MM_FABRIC_ARB_0, MM_FABRIC_ARB, 23),

	MSM_RPM_MAP(SMPS0B_0, SMPS0B, 2),
	MSM_RPM_MAP(SMPS1B_0, SMPS1B, 2),
	MSM_RPM_MAP(SMPS2B_0, SMPS2B, 2),
	MSM_RPM_MAP(SMPS3B_0, SMPS3B, 2),
	MSM_RPM_MAP(SMPS4B_0, SMPS4B, 2),
	MSM_RPM_MAP(LDO0B_0, LDO0B, 2),
	MSM_RPM_MAP(LDO1B_0, LDO1B, 2),
	MSM_RPM_MAP(LDO2B_0, LDO2B, 2),
	MSM_RPM_MAP(LDO3B_0, LDO3B, 2),
	MSM_RPM_MAP(LDO4B_0, LDO4B, 2),
	MSM_RPM_MAP(LDO5B_0, LDO5B, 2),
	MSM_RPM_MAP(LDO6B_0, LDO6B, 2),
	MSM_RPM_MAP(LVS0B, LVS0B, 1),
	MSM_RPM_MAP(LVS1B, LVS1B, 1),
	MSM_RPM_MAP(LVS2B, LVS2B, 1),
	MSM_RPM_MAP(LVS3B, LVS3B, 1),
	MSM_RPM_MAP(MVS, MVS, 1),

	MSM_RPM_MAP(SMPS0_0, SMPS0, 2),
	MSM_RPM_MAP(SMPS1_0, SMPS1, 2),
	MSM_RPM_MAP(SMPS2_0, SMPS2, 2),
	MSM_RPM_MAP(SMPS3_0, SMPS3, 2),
	MSM_RPM_MAP(SMPS4_0, SMPS4, 2),
	MSM_RPM_MAP(LDO0_0, LDO0, 2),
	MSM_RPM_MAP(LDO1_0, LDO1, 2),
	MSM_RPM_MAP(LDO2_0, LDO2, 2),
	MSM_RPM_MAP(LDO3_0, LDO3, 2),
	MSM_RPM_MAP(LDO4_0, LDO4, 2),
	MSM_RPM_MAP(LDO5_0, LDO5, 2),
	MSM_RPM_MAP(LDO6_0, LDO6, 2),
	MSM_RPM_MAP(LDO7_0, LDO7, 2),
	MSM_RPM_MAP(LDO8_0, LDO8, 2),
	MSM_RPM_MAP(LDO9_0, LDO9, 2),
	MSM_RPM_MAP(LDO10_0, LDO10, 2),
	MSM_RPM_MAP(LDO11_0, LDO11, 2),
	MSM_RPM_MAP(LDO12_0, LDO12, 2),
	MSM_RPM_MAP(LDO13_0, LDO13, 2),
	MSM_RPM_MAP(LDO14_0, LDO14, 2),
	MSM_RPM_MAP(LDO15_0, LDO15, 2),
	MSM_RPM_MAP(LDO16_0, LDO16, 2),
	MSM_RPM_MAP(LDO17_0, LDO17, 2),
	MSM_RPM_MAP(LDO18_0, LDO18, 2),
	MSM_RPM_MAP(LDO19_0, LDO19, 2),
	MSM_RPM_MAP(LDO20_0, LDO20, 2),
	MSM_RPM_MAP(LDO21_0, LDO21, 2),
	MSM_RPM_MAP(LDO22_0, LDO22, 2),
	MSM_RPM_MAP(LDO23_0, LDO23, 2),
	MSM_RPM_MAP(LDO24_0, LDO24, 2),
	MSM_RPM_MAP(LDO25_0, LDO25, 2),
	MSM_RPM_MAP(LVS0, LVS0, 1),
	MSM_RPM_MAP(LVS1, LVS1, 1),
	MSM_RPM_MAP(NCP_0, NCP, 2),

	MSM_RPM_MAP(CXO_BUFFERS, CXO_BUFFERS, 1),
};
unsigned int rpm_map_data_size = ARRAY_SIZE(rpm_map_data);

#endif
