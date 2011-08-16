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
#include <linux/list.h>
#include <linux/platform_device.h>
#include <asm/clkdev.h>
#include <linux/msm_kgsl.h>
#include <mach/irqs-8960.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_sps.h>
#include <mach/rpm.h>

#include "clock.h"
#include "devices.h"
#include "devices-msm8x60.h"

#ifdef CONFIG_MSM_MPM
#include "mpm.h"
#endif

/* Address of GSBI blocks */
#define MSM_GSBI1_PHYS		0x16000000
#define MSM_GSBI2_PHYS		0x16100000
#define MSM_GSBI3_PHYS		0x16200000
#define MSM_GSBI4_PHYS		0x16300000
#define MSM_GSBI5_PHYS		0x16400000
#define MSM_GSBI6_PHYS		0x16500000
#define MSM_GSBI7_PHYS		0x16600000
#define MSM_GSBI8_PHYS		0x1A000000
#define MSM_GSBI9_PHYS		0x1A100000
#define MSM_GSBI10_PHYS		0x1A200000

#define MSM_UART2DM_PHYS	(MSM_GSBI2_PHYS + 0x40000)
#define MSM_UART5DM_PHYS	(MSM_GSBI5_PHYS + 0x40000)

/* GSBI QUP devices */
#define MSM_GSBI1_QUP_PHYS	(MSM_GSBI1_PHYS + 0x80000)
#define MSM_GSBI2_QUP_PHYS	(MSM_GSBI2_PHYS + 0x80000)
#define MSM_GSBI3_QUP_PHYS	(MSM_GSBI3_PHYS + 0x80000)
#define MSM_GSBI4_QUP_PHYS	(MSM_GSBI4_PHYS + 0x80000)
#define MSM_GSBI5_QUP_PHYS	(MSM_GSBI5_PHYS + 0x80000)
#define MSM_GSBI6_QUP_PHYS	(MSM_GSBI6_PHYS + 0x80000)
#define MSM_GSBI7_QUP_PHYS	(MSM_GSBI7_PHYS + 0x80000)
#define MSM_GSBI8_QUP_PHYS	(MSM_GSBI8_PHYS + 0x80000)
#define MSM_GSBI9_QUP_PHYS	(MSM_GSBI9_PHYS + 0x80000)
#define MSM_GSBI10_QUP_PHYS	(MSM_GSBI10_PHYS + 0x80000)
#define MSM_QUP_SIZE		SZ_4K

#define MSM_PMIC1_SSBI_CMD_PHYS	0x00500000
#define MSM_PMIC2_SSBI_CMD_PHYS	0x00C00000
#define MSM_PMIC_SSBI_SIZE	SZ_4K

static struct resource resources_otg[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
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
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct resource resources_hsusb[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= USB1_HS_IRQ,
		.end	= USB1_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_gadget_peripheral = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb),
	.resource	= resources_hsusb,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct resource resources_uart_gsbi2[] = {
	{
		.start	= GSBI2_UARTDM_IRQ,
		.end	= GSBI2_UARTDM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2DM_PHYS,
		.end	= MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.name	= "uartdm_resource",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MSM_GSBI2_PHYS,
		.end	= MSM_GSBI2_PHYS + PAGE_SIZE - 1,
		.name	= "gsbi_resource",
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm8960_device_uart_gsbi2 = {
	.name	= "msm_serial_hsl",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart_gsbi2),
	.resource	= resources_uart_gsbi2,
};

static struct resource resources_uart_gsbi5[] = {
	{
		.start	= GSBI5_UARTDM_IRQ,
		.end	= GSBI5_UARTDM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART5DM_PHYS,
		.end	= MSM_UART5DM_PHYS + PAGE_SIZE - 1,
		.name	= "uartdm_resource",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MSM_GSBI5_PHYS,
		.end	= MSM_GSBI5_PHYS + PAGE_SIZE - 1,
		.name	= "gsbi_resource",
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm8960_device_uart_gsbi5 = {
	.name	= "msm_serial_hsl",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart_gsbi5),
	.resource	= resources_uart_gsbi5,
};

#define MSM_WCNSS_PHYS	0x03000000
#define MSM_WCNSS_SIZE	0x01000000

static struct resource resources_wcnss_wlan[] = {
	{
		.start	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.end	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.name	= "wcnss_wlanrx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.end	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.name	= "wcnss_wlantx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_WCNSS_PHYS,
		.end	= MSM_WCNSS_PHYS + MSM_WCNSS_SIZE - 1,
		.name	= "wcnss_mmio",
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_wcnss_wlan = {
	.name		= "wcnss_wlan",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_wcnss_wlan),
	.resource	= resources_wcnss_wlan,
};

#define MSM_SDC1_BASE         0x12400000
#define MSM_SDC2_BASE         0x12140000
#define MSM_SDC3_BASE         0x12180000
#define MSM_SDC4_BASE         0x121C0000
#define MSM_SDC5_BASE         0x12200000

static struct resource resources_sdc1[] = {
	{
		.name	= "core_mem",
		.flags	= IORESOURCE_MEM,
		.start	= MSM_SDC1_BASE,
		.end	= MSM_SDC1_BASE + SZ_4K - 1,
	},
	{
		.name	= "core_irq",
		.flags	= IORESOURCE_IRQ,
		.start	= SDC1_IRQ_0,
		.end	= SDC1_IRQ_0
	}
};

static struct resource resources_sdc2[] = {
	{
		.name	= "core_mem",
		.flags	= IORESOURCE_MEM,
		.start	= MSM_SDC2_BASE,
		.end	= MSM_SDC2_BASE + SZ_4K - 1,
	},
	{
		.name	= "core_irq",
		.flags	= IORESOURCE_IRQ,
		.start	= SDC2_IRQ_0,
		.end	= SDC2_IRQ_0
	}
};

static struct resource resources_sdc3[] = {
	{
		.name	= "core_mem",
		.flags	= IORESOURCE_MEM,
		.start	= MSM_SDC3_BASE,
		.end	= MSM_SDC3_BASE + SZ_4K - 1,
	},
	{
		.name	= "core_irq",
		.flags	= IORESOURCE_IRQ,
		.start	= SDC3_IRQ_0,
		.end	= SDC3_IRQ_0
	}
};

static struct resource resources_sdc4[] = {
	{
		.name	= "core_mem",
		.flags	= IORESOURCE_MEM,
		.start	= MSM_SDC4_BASE,
		.end	= MSM_SDC4_BASE + SZ_4K - 1,
	},
	{
		.name	= "core_irq",
		.flags	= IORESOURCE_IRQ,
		.start	= SDC4_IRQ_0,
		.end	= SDC4_IRQ_0
	}
};

static struct resource resources_sdc5[] = {
	{
		.name	= "core_mem",
		.flags	= IORESOURCE_MEM,
		.start	= MSM_SDC5_BASE,
		.end	= MSM_SDC5_BASE + SZ_4K - 1,
	},
	{
		.name	= "core_irq",
		.flags	= IORESOURCE_IRQ,
		.start	= SDC5_IRQ_0,
		.end	= SDC5_IRQ_0
	}
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

struct platform_device msm_device_smd = {
	.name		= "msm_smd",
	.id		= -1,
};

struct resource msm_dmov_resource[] = {
	{
		.start = ADM_0_SCSS_0_IRQ,
		.end = (resource_size_t)MSM_DMOV_BASE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_dmov = {
	.name	= "msm_dmov",
	.id	= -1,
	.resource = msm_dmov_resource,
	.num_resources = ARRAY_SIZE(msm_dmov_resource),
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

static struct resource resources_qup_i2c_gsbi4[] = {
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI4_PHYS,
		.end	= MSM_GSBI4_PHYS + MSM_QUP_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI4_QUP_PHYS,
		.end	= MSM_GSBI4_QUP_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI4_QUP_IRQ,
		.end	= GSBI4_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm8960_device_qup_i2c_gsbi4 = {
	.name		= "qup_i2c",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_qup_i2c_gsbi4),
	.resource	= resources_qup_i2c_gsbi4,
};

static struct resource resources_ssbi_pm8921[] = {
	{
		.start  = MSM_PMIC1_SSBI_CMD_PHYS,
		.end    = MSM_PMIC1_SSBI_CMD_PHYS + MSM_PMIC_SSBI_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device msm8960_device_ssbi_pm8921 = {
	.name           = "msm_ssbi",
	.id             = 0,
	.resource       = resources_ssbi_pm8921,
	.num_resources  = ARRAY_SIZE(resources_ssbi_pm8921),
};

static struct resource resources_qup_spi_gsbi1[] = {
	{
		.name   = "spi_base",
		.start  = MSM_GSBI1_QUP_PHYS,
		.end    = MSM_GSBI1_QUP_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_base",
		.start  = MSM_GSBI1_PHYS,
		.end    = MSM_GSBI1_PHYS + 4 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_irq_in",
		.start  = GSBI1_QUP_IRQ,
		.end    = GSBI1_QUP_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device msm8960_device_qup_spi_gsbi1 = {
	.name	= "spi_qsd",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_qup_spi_gsbi1),
	.resource	= resources_qup_spi_gsbi1,
};

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
};
unsigned msm_num_footswitch_devices = ARRAY_SIZE(msm_footswitch_devices);

static struct resource resources_sps[] = {
	{
		.name	= "pipe_mem",
		.start	= 0x12800000,
		.end	= 0x12800000 + 0x4000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "bamdma_dma",
		.start	= 0x12240000,
		.end	= 0x12240000 + 0x1000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "bamdma_bam",
		.start	= 0x12244000,
		.end	= 0x12244000 + 0x4000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "bamdma_irq",
		.start	= SPS_BAM_DMA_IRQ,
		.end	= SPS_BAM_DMA_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct msm_sps_platform_data msm_sps_pdata = {
	.bamdma_restricted_pipes = 0x06,
};

struct platform_device msm_device_sps = {
	.name		= "msm_sps",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_sps),
	.resource	= resources_sps,
	.dev.platform_data = &msm_sps_pdata,
};

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
	RPM_APCC_CPU0_GP_HIGH_IRQ,
	RPM_APCC_CPU0_GP_MEDIUM_IRQ,
	RPM_APCC_CPU0_GP_LOW_IRQ,
	RPM_APCC_CPU0_WAKE_UP_IRQ,
	RPM_APCC_CPU1_GP_HIGH_IRQ,
	RPM_APCC_CPU1_GP_MEDIUM_IRQ,
	RPM_APCC_CPU1_GP_LOW_IRQ,
	RPM_APCC_CPU1_WAKE_UP_IRQ,
	MSS_TO_APPS_IRQ_0,
	MSS_TO_APPS_IRQ_1,
	MSS_TO_APPS_IRQ_2,
	MSS_TO_APPS_IRQ_3,
	MSS_TO_APPS_IRQ_4,
	MSS_TO_APPS_IRQ_5,
	MSS_TO_APPS_IRQ_6,
	MSS_TO_APPS_IRQ_7,
	MSS_TO_APPS_IRQ_8,
	MSS_TO_APPS_IRQ_9,
	LPASS_SCSS_GP_LOW_IRQ,
	LPASS_SCSS_GP_MEDIUM_IRQ,
	LPASS_SCSS_GP_HIGH_IRQ,
	SPS_MTI_31,
};

struct msm_mpm_device_data msm_mpm_dev_data = {
	.irqs_m2a = msm_mpm_irqs_m2a,
	.irqs_m2a_size = ARRAY_SIZE(msm_mpm_irqs_m2a),
	.bypassed_apps_irqs = msm_mpm_bypassed_apps_irqs,
	.bypassed_apps_irqs_size = ARRAY_SIZE(msm_mpm_bypassed_apps_irqs),
	.mpm_request_reg_base = MSM_RPM_BASE + 0x9d8,
	.mpm_status_reg_base = MSM_RPM_BASE + 0xdf8,
	.mpm_apps_ipc_reg = MSM_APCS_GCC_BASE + 0x008,
	.mpm_apps_ipc_val =  BIT(1),
	.mpm_ipc_irq = RPM_APCC_CPU0_GP_MEDIUM_IRQ,

};
#endif

struct clk_lookup msm_clocks_8960[] = {
	CLK_DUMMY("gsbi_uart_clk",	GSBI1_UART_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI2_UART_CLK,
						  "msm_serial_hsl.0", OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI3_UART_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI4_UART_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI5_UART_CLK,		NULL, OFF),
	CLK_DUMMY("uartdm_clk",		GSBI6_UART_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI7_UART_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI8_UART_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI9_UART_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI10_UART_CLK,	NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI11_UART_CLK,	NULL, OFF),
	CLK_DUMMY("gsbi_uart_clk",	GSBI12_UART_CLK,	NULL, OFF),
	CLK_DUMMY("spi_clk",		GSBI1_QUP_CLK, "spi_qsd.0", OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI2_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI3_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI4_QUP_CLK,
							"qup_i2c.4", OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI5_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI6_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI7_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI8_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI9_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI10_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI11_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_qup_clk",	GSBI12_QUP_CLK,		NULL, OFF),
	CLK_DUMMY("pdm_clk",		PDM_CLK,		NULL, OFF),
	CLK_DUMMY("pmem_clk",		PMEM_CLK,		NULL, OFF),
	CLK_DUMMY("prng_clk",		PRNG_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_clk",		SDC1_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_clk",		SDC2_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_clk",		SDC3_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_clk",		SDC4_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_clk",		SDC5_CLK,		NULL, OFF),
	CLK_DUMMY("tsif_ref_clk",	TSIF_REF_CLK,		NULL, OFF),
	CLK_DUMMY("tssc_clk",		TSSC_CLK,		NULL, OFF),
	CLK_DUMMY("usb_hs_clk",		USB_HS1_XCVR_CLK,	NULL, OFF),
	CLK_DUMMY("usb_phy_clk",	USB_PHY0_CLK,		NULL, OFF),
	CLK_DUMMY("usb_fs_src_clk",	USB_FS1_SRC_CLK,	NULL, OFF),
	CLK_DUMMY("usb_fs_clk",		USB_FS1_XCVR_CLK,	NULL, OFF),
	CLK_DUMMY("usb_fs_sys_clk",	USB_FS1_SYS_CLK,	NULL, OFF),
	CLK_DUMMY("usb_fs_src_clk",	USB_FS2_SRC_CLK,	NULL, OFF),
	CLK_DUMMY("usb_fs_clk",		USB_FS2_XCVR_CLK,	NULL, OFF),
	CLK_DUMMY("usb_fs_sys_clk",	USB_FS2_SYS_CLK,	NULL, OFF),
	CLK_DUMMY("ce_clk",		CE2_CLK,		NULL, OFF),
	CLK_DUMMY("spi_pclk",		GSBI1_P_CLK, "spi_qsd.0", OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI2_P_CLK,
						  "msm_serial_hsl.0", OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI3_P_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI4_P_CLK,
							"qup_i2c.4", OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI5_P_CLK,		NULL, OFF),
	CLK_DUMMY("uartdm_pclk",	GSBI6_P_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI7_P_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI8_P_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI9_P_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI10_P_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI11_P_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI12_P_CLK,		NULL, OFF),
	CLK_DUMMY("gsbi_pclk",		GSBI12_P_CLK,		NULL, OFF),
	CLK_DUMMY("ppss_pclk",		PPSS_P_CLK,		NULL, OFF),
	CLK_DUMMY("tsif_pclk",		TSIF_P_CLK,		NULL, OFF),
	CLK_DUMMY("usb_fs_pclk",	USB_FS1_P_CLK,		NULL, OFF),
	CLK_DUMMY("usb_fs_pclk",	USB_FS2_P_CLK,		NULL, OFF),
	CLK_DUMMY("usb_hs_pclk",	USB_HS1_P_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_pclk",		SDC1_P_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_pclk",		SDC2_P_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_pclk",		SDC3_P_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_pclk",		SDC4_P_CLK,		NULL, OFF),
	CLK_DUMMY("sdc_pclk",		SDC5_P_CLK,		NULL, OFF),
	CLK_DUMMY("adm_clk",		ADM0_CLK,		NULL, OFF),
	CLK_DUMMY("adm_pclk",		ADM0_P_CLK,		NULL, OFF),
	CLK_DUMMY("pmic_arb_pclk",	PMIC_ARB0_P_CLK,	NULL, OFF),
	CLK_DUMMY("pmic_arb_pclk",	PMIC_ARB1_P_CLK,	NULL, OFF),
	CLK_DUMMY("pmic_ssbi2",		PMIC_SSBI2_CLK,		NULL, OFF),
	CLK_DUMMY("rpm_msg_ram_pclk",	RPM_MSG_RAM_P_CLK,	NULL, OFF),
	CLK_DUMMY("amp_clk",		AMP_CLK,		NULL, OFF),
	CLK_DUMMY("cam_clk",		CAM0_CLK,		NULL, OFF),
	CLK_DUMMY("cam_clk",		CAM1_CLK,		NULL, OFF),
	CLK_DUMMY("csi_src_clk",	CSI0_SRC_CLK,		NULL, OFF),
	CLK_DUMMY("csi_src_clk",	CSI1_SRC_CLK,		NULL, OFF),
	CLK_DUMMY("csi_clk",		CSI0_CLK,		NULL, OFF),
	CLK_DUMMY("csi_clk",		CSI1_CLK,		NULL, OFF),
	CLK_DUMMY("csi_pix_clk",	CSI_PIX_CLK,		NULL, OFF),
	CLK_DUMMY("csi_rdi_clk",	CSI_RDI_CLK,		NULL, OFF),
	CLK_DUMMY("csiphy_timer_src_clk", CSIPHY_TIMER_SRC_CLK,	NULL, OFF),
	CLK_DUMMY("csi0phy_timer_clk",	CSIPHY0_TIMER_CLK,	NULL, OFF),
	CLK_DUMMY("csi1phy_timer_clk",	CSIPHY1_TIMER_CLK,	NULL, OFF),
	CLK_DUMMY("dsi_byte_div_clk",	DSI1_BYTE_CLK,		NULL, OFF),
	CLK_DUMMY("dsi_byte_div_clk",	DSI2_BYTE_CLK,		NULL, OFF),
	CLK_DUMMY("dsi_esc_clk",	DSI1_ESC_CLK,		NULL, OFF),
	CLK_DUMMY("dsi_esc_clk",	DSI2_ESC_CLK,		NULL, OFF),
	CLK_DUMMY("gfx2d0_clk",		GFX2D0_CLK,		NULL, OFF),
	CLK_DUMMY("gfx2d1_clk",		GFX2D1_CLK,		NULL, OFF),
	CLK_DUMMY("gfx3d_clk",		GFX3D_CLK,		NULL, OFF),
	CLK_DUMMY("ijpeg_clk",		IJPEG_CLK,		NULL, OFF),
	CLK_DUMMY("imem_clk",		IMEM_CLK,		NULL, OFF),
	CLK_DUMMY("jpegd_clk",		JPEGD_CLK,		NULL, OFF),
	CLK_DUMMY("mdp_clk",		MDP_CLK,		NULL, OFF),
	CLK_DUMMY("mdp_vsync_clk",	MDP_VSYNC_CLK,		NULL, OFF),
	CLK_DUMMY("lut_mdp",		LUT_MDP_CLK,		NULL, OFF),
	CLK_DUMMY("rot_clk",		ROT_CLK,		NULL, OFF),
	CLK_DUMMY("tv_src_clk",		TV_SRC_CLK,		NULL, OFF),
	CLK_DUMMY("tv_enc_clk",		TV_ENC_CLK,		NULL, OFF),
	CLK_DUMMY("tv_dac_clk",		TV_DAC_CLK,		NULL, OFF),
	CLK_DUMMY("vcodec_clk",		VCODEC_CLK,		NULL, OFF),
	CLK_DUMMY("mdp_tv_clk",		MDP_TV_CLK,		NULL, OFF),
	CLK_DUMMY("hdmi_clk",		HDMI_TV_CLK,		NULL, OFF),
	CLK_DUMMY("hdmi_app_clk",	HDMI_APP_CLK,		NULL, OFF),
	CLK_DUMMY("vpe_clk",		VPE_CLK,		NULL, OFF),
	CLK_DUMMY("vfe_clk",		VFE_CLK,		NULL, OFF),
	CLK_DUMMY("csi_vfe_clk",	CSI0_VFE_CLK,		NULL, OFF),
	CLK_DUMMY("vfe_axi_clk",	VFE_AXI_CLK,		NULL, OFF),
	CLK_DUMMY("ijpeg_axi_clk",	IJPEG_AXI_CLK,		NULL, OFF),
	CLK_DUMMY("mdp_axi_clk",	MDP_AXI_CLK,		NULL, OFF),
	CLK_DUMMY("rot_axi_clk",	ROT_AXI_CLK,		NULL, OFF),
	CLK_DUMMY("vcodec_axi_clk",	VCODEC_AXI_CLK,		NULL, OFF),
	CLK_DUMMY("vcodec_axi_a_clk",	VCODEC_AXI_A_CLK,	NULL, OFF),
	CLK_DUMMY("vcodec_axi_b_clk",	VCODEC_AXI_B_CLK,	NULL, OFF),
	CLK_DUMMY("vpe_axi_clk",	VPE_AXI_CLK,		NULL, OFF),
	CLK_DUMMY("amp_pclk",		AMP_P_CLK,		NULL, OFF),
	CLK_DUMMY("csi_pclk",		CSI0_P_CLK,		NULL, OFF),
	CLK_DUMMY("dsi_m_pclk",		DSI1_M_P_CLK,		NULL, OFF),
	CLK_DUMMY("dsi_s_pclk",		DSI1_S_P_CLK,		NULL, OFF),
	CLK_DUMMY("dsi_m_pclk",		DSI2_M_P_CLK,		NULL, OFF),
	CLK_DUMMY("dsi_s_pclk",		DSI2_S_P_CLK,		NULL, OFF),
	CLK_DUMMY("gfx2d0_pclk",	GFX2D0_P_CLK,		NULL, OFF),
	CLK_DUMMY("gfx2d1_pclk",	GFX2D1_P_CLK,		NULL, OFF),
	CLK_DUMMY("gfx3d_pclk",		GFX3D_P_CLK,		NULL, OFF),
	CLK_DUMMY("hdmi_m_pclk",	HDMI_M_P_CLK,		NULL, OFF),
	CLK_DUMMY("hdmi_s_pclk",	HDMI_S_P_CLK,		NULL, OFF),
	CLK_DUMMY("ijpeg_pclk",		IJPEG_P_CLK,		NULL, OFF),
	CLK_DUMMY("jpegd_pclk",		JPEGD_P_CLK,		NULL, OFF),
	CLK_DUMMY("imem_pclk",		IMEM_P_CLK,		NULL, OFF),
	CLK_DUMMY("mdp_pclk",		MDP_P_CLK,		NULL, OFF),
	CLK_DUMMY("smmu_pclk",		SMMU_P_CLK,		NULL, OFF),
	CLK_DUMMY("rotator_pclk",	ROT_P_CLK,		NULL, OFF),
	CLK_DUMMY("tv_enc_pclk",	TV_ENC_P_CLK,		NULL, OFF),
	CLK_DUMMY("vcodec_pclk",	VCODEC_P_CLK,		NULL, OFF),
	CLK_DUMMY("vfe_pclk",		VFE_P_CLK,		NULL, OFF),
	CLK_DUMMY("vpe_pclk",		VPE_P_CLK,		NULL, OFF),
	CLK_DUMMY("mi2s_osr_clk",	MI2S_OSR_CLK,		NULL, OFF),
	CLK_DUMMY("mi2s_bit_clk",	MI2S_BIT_CLK,		NULL, OFF),
	CLK_DUMMY("i2s_mic_osr_clk",	CODEC_I2S_MIC_OSR_CLK,	NULL, OFF),
	CLK_DUMMY("i2s_mic_bit_clk",	CODEC_I2S_MIC_BIT_CLK,	NULL, OFF),
	CLK_DUMMY("i2s_mic_osr_clk",	SPARE_I2S_MIC_OSR_CLK,	NULL, OFF),
	CLK_DUMMY("i2s_mic_bit_clk",	SPARE_I2S_MIC_BIT_CLK,	NULL, OFF),
	CLK_DUMMY("i2s_spkr_osr_clk",	CODEC_I2S_SPKR_OSR_CLK,	NULL, OFF),
	CLK_DUMMY("i2s_spkr_bit_clk",	CODEC_I2S_SPKR_BIT_CLK,	NULL, OFF),
	CLK_DUMMY("i2s_spkr_osr_clk",	SPARE_I2S_SPKR_OSR_CLK,	NULL, OFF),
	CLK_DUMMY("i2s_spkr_bit_clk",	SPARE_I2S_SPKR_BIT_CLK,	NULL, OFF),
	CLK_DUMMY("pcm_clk",		PCM_CLK,		NULL, OFF),
	CLK_DUMMY("iommu_clk",		JPEGD_AXI_CLK,		NULL, 0),
	CLK_DUMMY("iommu_clk",		VFE_AXI_CLK,		NULL, 0),
	CLK_DUMMY("iommu_clk",		VCODEC_AXI_CLK,	NULL, 0),
	CLK_DUMMY("iommu_clk",		GFX3D_CLK,	NULL, 0),
	CLK_DUMMY("iommu_clk",		GFX2D0_CLK,	NULL, 0),
	CLK_DUMMY("iommu_clk",		GFX2D1_CLK,	NULL, 0),

	CLK_DUMMY("dfab_dsps_clk",	DFAB_DSPS_CLK, NULL, 0),
	CLK_DUMMY("dfab_usb_hs_clk",	DFAB_USB_HS_CLK, NULL, 0),
	CLK_DUMMY("dfab_sdc_clk",	DFAB_SDC1_CLK, "msm_sdcc.1", 0),
	CLK_DUMMY("dfab_sdc_clk",	DFAB_SDC2_CLK, "msm_sdcc.2", 0),
	CLK_DUMMY("dfab_sdc_clk",	DFAB_SDC3_CLK, "msm_sdcc.3", 0),
	CLK_DUMMY("dfab_sdc_clk",	DFAB_SDC4_CLK, "msm_sdcc.4", 0),
	CLK_DUMMY("dfab_sdc_clk",	DFAB_SDC5_CLK, "msm_sdcc.5", 0),
	CLK_DUMMY("dfab_clk",		DFAB_CLK,		NULL, 0),
	CLK_DUMMY("dma_bam_pclk",	DMA_BAM_P_CLK,		NULL, 0),
};

unsigned msm_num_clocks_8960 = ARRAY_SIZE(msm_clocks_8960);

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors grp3d_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp3d_nominal_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 200800000U,
	},
};

static struct msm_bus_vectors grp3d_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 2096000000U,
	},
};

static struct msm_bus_paths grp3d_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp3d_init_vectors),
		grp3d_init_vectors,
	},
	{
		ARRAY_SIZE(grp3d_nominal_vectors),
		grp3d_nominal_vectors,
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
		.ib = 248000000,
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

struct msm_bus_scale_pdata grp2d0_bus_scale_pdata = {
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
		.ib = 248000000,
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

struct msm_bus_scale_pdata grp2d1_bus_scale_pdata = {
	grp2d1_bus_scale_usecases,
	ARRAY_SIZE(grp2d1_bus_scale_usecases),
	.name = "grp2d1",
};
#endif
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
				.bus_freq = 2,
			},
			{
				.gpu_freq = 228571000,
				.bus_freq = 1,
			},
			{
				.gpu_freq = 266667000,
				.bus_freq = 0,
			},
		},
		.init_level = 0,
		.num_levels = 3,
		.set_grp_async = NULL,
		.idle_timeout = HZ/5,
#ifdef CONFIG_MSM_BUS_SCALING
		.nap_allowed = true,
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
static struct resource kgsl_2d0_resources = {
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
				.gpu_freq = 228571000,
				.bus_freq = 1,
			},
			{
				.gpu_freq = 228571000,
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

static struct resource kgsl_2d1_resources = {
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
				.gpu_freq = 228571000,
				.bus_freq = 1,
			},
			{
				.gpu_freq = 228571000,
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
#endif

struct msm_rpm_map_data rpm_map_data[] __initdata = {
	MSM_RPM_MAP(TRIGGER_TIMED_TO, TRIGGER_TIMED, 1),
	MSM_RPM_MAP(TRIGGER_TIMED_SCLK_COUNT, TRIGGER_TIMED, 1),

	MSM_RPM_MAP(RPM_CTL, RPM_CTL, 1),

	MSM_RPM_MAP(CXO_CLK, CXO_CLK, 1),
	MSM_RPM_MAP(PXO_CLK, PXO_CLK, 1),
	MSM_RPM_MAP(APPS_FABRIC_CLK, APPS_FABRIC_CLK, 1),
	MSM_RPM_MAP(SYSTEM_FABRIC_CLK, SYSTEM_FABRIC_CLK, 1),
	MSM_RPM_MAP(MM_FABRIC_CLK, MM_FABRIC_CLK, 1),
	MSM_RPM_MAP(DAYTONA_FABRIC_CLK, DAYTONA_FABRIC_CLK, 1),
	MSM_RPM_MAP(SFPB_CLK, SFPB_CLK, 1),
	MSM_RPM_MAP(CFPB_CLK, CFPB_CLK, 1),
	MSM_RPM_MAP(MMFPB_CLK, MMFPB_CLK, 1),
	MSM_RPM_MAP(EBI1_CLK, EBI1_CLK, 1),

	MSM_RPM_MAP(APPS_FABRIC_CFG_HALT_0, APPS_FABRIC_CFG_HALT, 2),
	MSM_RPM_MAP(APPS_FABRIC_CFG_CLKMOD_0, APPS_FABRIC_CFG_CLKMOD, 3),
	MSM_RPM_MAP(APPS_FABRIC_CFG_IOCTL, APPS_FABRIC_CFG_IOCTL, 1),
	MSM_RPM_MAP(APPS_FABRIC_ARB_0, APPS_FABRIC_ARB, 12),

	MSM_RPM_MAP(SYS_FABRIC_CFG_HALT_0, SYS_FABRIC_CFG_HALT, 2),
	MSM_RPM_MAP(SYS_FABRIC_CFG_CLKMOD_0, SYS_FABRIC_CFG_CLKMOD, 3),
	MSM_RPM_MAP(SYS_FABRIC_CFG_IOCTL, SYS_FABRIC_CFG_IOCTL, 1),
	MSM_RPM_MAP(SYSTEM_FABRIC_ARB_0, SYSTEM_FABRIC_ARB, 27),

	MSM_RPM_MAP(MMSS_FABRIC_CFG_HALT_0, MMSS_FABRIC_CFG_HALT, 2),
	MSM_RPM_MAP(MMSS_FABRIC_CFG_CLKMOD_0, MMSS_FABRIC_CFG_CLKMOD, 3),
	MSM_RPM_MAP(MMSS_FABRIC_CFG_IOCTL, MMSS_FABRIC_CFG_IOCTL, 1),
	MSM_RPM_MAP(MM_FABRIC_ARB_0, MM_FABRIC_ARB, 23),

	MSM_RPM_MAP(PM8921_S1_0, PM8921_S1, 2),
	MSM_RPM_MAP(PM8921_S2_0, PM8921_S2, 2),
	MSM_RPM_MAP(PM8921_S3_0, PM8921_S3, 2),
	MSM_RPM_MAP(PM8921_S4_0, PM8921_S4, 2),
	MSM_RPM_MAP(PM8921_S5_0, PM8921_S5, 2),
	MSM_RPM_MAP(PM8921_S6_0, PM8921_S6, 2),
	MSM_RPM_MAP(PM8921_S7_0, PM8921_S7, 2),
	MSM_RPM_MAP(PM8921_S8_0, PM8921_S8, 2),
	MSM_RPM_MAP(PM8921_L1_0, PM8921_L1, 2),
	MSM_RPM_MAP(PM8921_L2_0, PM8921_L2, 2),
	MSM_RPM_MAP(PM8921_L3_0, PM8921_L3, 2),
	MSM_RPM_MAP(PM8921_L4_0, PM8921_L4, 2),
	MSM_RPM_MAP(PM8921_L5_0, PM8921_L5, 2),
	MSM_RPM_MAP(PM8921_L6_0, PM8921_L6, 2),
	MSM_RPM_MAP(PM8921_L7_0, PM8921_L7, 2),
	MSM_RPM_MAP(PM8921_L8_0, PM8921_L8, 2),
	MSM_RPM_MAP(PM8921_L9_0, PM8921_L9, 2),
	MSM_RPM_MAP(PM8921_L10_0, PM8921_L10, 2),
	MSM_RPM_MAP(PM8921_L11_0, PM8921_L11, 2),
	MSM_RPM_MAP(PM8921_L12_0, PM8921_L12, 2),
	MSM_RPM_MAP(PM8921_L13_0, PM8921_L13, 2),
	MSM_RPM_MAP(PM8921_L14_0, PM8921_L14, 2),
	MSM_RPM_MAP(PM8921_L15_0, PM8921_L15, 2),
	MSM_RPM_MAP(PM8921_L16_0, PM8921_L16, 2),
	MSM_RPM_MAP(PM8921_L17_0, PM8921_L17, 2),
	MSM_RPM_MAP(PM8921_L18_0, PM8921_L18, 2),
	MSM_RPM_MAP(PM8921_L19_0, PM8921_L19, 2),
	MSM_RPM_MAP(PM8921_L20_0, PM8921_L20, 2),
	MSM_RPM_MAP(PM8921_L21_0, PM8921_L21, 2),
	MSM_RPM_MAP(PM8921_L22_0, PM8921_L22, 2),
	MSM_RPM_MAP(PM8921_L23_0, PM8921_L23, 2),
	MSM_RPM_MAP(PM8921_L24_0, PM8921_L24, 2),
	MSM_RPM_MAP(PM8921_L25_0, PM8921_L25, 2),
	MSM_RPM_MAP(PM8921_L26_0, PM8921_L26, 2),
	MSM_RPM_MAP(PM8921_L27_0, PM8921_L27, 2),
	MSM_RPM_MAP(PM8921_L28_0, PM8921_L28, 2),
	MSM_RPM_MAP(PM8921_L29_0, PM8921_L29, 2),
	MSM_RPM_MAP(PM8921_CLK1_0, PM8921_CLK1, 2),
	MSM_RPM_MAP(PM8921_CLK2_0, PM8921_CLK2, 2),
	MSM_RPM_MAP(PM8921_LVS1, PM8921_LVS1, 1),
	MSM_RPM_MAP(PM8921_LVS2, PM8921_LVS2, 1),
	MSM_RPM_MAP(PM8921_LVS3, PM8921_LVS3, 1),
	MSM_RPM_MAP(PM8921_LVS4, PM8921_LVS4, 1),
	MSM_RPM_MAP(PM8921_LVS5, PM8921_LVS5, 1),
	MSM_RPM_MAP(PM8921_LVS6, PM8921_LVS6, 1),
	MSM_RPM_MAP(PM8921_LVS7, PM8921_LVS7, 1),
	MSM_RPM_MAP(NCP_0, NCP, 2),
	MSM_RPM_MAP(CXO_BUFFERS, CXO_BUFFERS, 1),
	MSM_RPM_MAP(USB_OTG_SWITCH, USB_OTG_SWITCH, 1),
	MSM_RPM_MAP(HDMI_SWITCH, HDMI_SWITCH, 1),

};
unsigned int rpm_map_data_size = ARRAY_SIZE(rpm_map_data);
