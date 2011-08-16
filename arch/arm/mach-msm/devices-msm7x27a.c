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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/msm_iomap.h>
#include <mach/board.h>
#include <mach/dma.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>
#include <mach/usbdiag.h>
#include <mach/usb_gadget_fserial.h>

#include "clock.h"
#include "clock-voter.h"
#include "clock-pcom.h"
#include "devices.h"
#include "devices-msm7x2xa.h"

/* Address of GSBI blocks */
#define MSM_GSBI0_PHYS		0xA1200000
#define MSM_GSBI1_PHYS		0xA1300000

/* GSBI QUPe devices */
#define MSM_GSBI0_QUP_PHYS	(MSM_GSBI0_PHYS + 0x80000)
#define MSM_GSBI1_QUP_PHYS	(MSM_GSBI1_PHYS + 0x80000)

static struct resource gsbi0_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI0_QUP_PHYS,
		.end	= MSM_GSBI0_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI0_PHYS,
		.end	= MSM_GSBI0_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

/* Use GSBI0 QUP for /dev/i2c-0 */
struct platform_device msm_gsbi0_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI0_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi0_qup_i2c_resources),
	.resource	= gsbi0_qup_i2c_resources,
};

static struct resource gsbi1_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI1_QUP_PHYS,
		.end	= MSM_GSBI1_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI1_PHYS,
		.end	= MSM_GSBI1_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= INT_ARM11_DMA,
		.end	= INT_ARM11_DMA,
		.flags	= IORESOURCE_IRQ,
	},
};

/* Use GSBI1 QUP for /dev/i2c-1 */
struct platform_device msm_gsbi1_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI1_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi1_qup_i2c_resources),
	.resource	= gsbi1_qup_i2c_resources,
};

#define MSM_HSUSB_PHYS        0xA0800000
static struct resource resources_hsusb_otg[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 dma_mask = 0xffffffffULL;
struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb_otg),
	.resource	= resources_hsusb_otg,
	.dev		= {
		.dma_mask		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

static struct resource resources_gadget_peripheral[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_gadget_peripheral = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_gadget_peripheral),
	.resource	= resources_gadget_peripheral,
	.dev		= {
		.dma_mask		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

static struct resource resources_hsusb_host[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb_host = {
	.name		= "msm_hsusb_host",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_hsusb_host),
	.resource	= resources_hsusb_host,
	.dev		= {
		.dma_mask		= &dma_mask,
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

struct usb_diag_platform_data usb_diag_pdata = {
	.ch_name = DIAG_LEGACY,
};

struct platform_device usb_diag_device = {
	.name	= "usb_diag",
	.id	= -1,
	.dev	= {
		.platform_data = &usb_diag_pdata,
	},
};
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
static struct resource msm_dmov_resource[] = {
	{
		.start	= INT_ADM_AARM,
		.end	= (resource_size_t)MSM_DMOV_BASE,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_dmov = {
	.name		= "msm_dmov",
	.id		= -1,
	.resource	= msm_dmov_resource,
	.num_resources	= ARRAY_SIZE(msm_dmov_resource),
};

struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.id	= -1,
};

static struct resource resources_uart1[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart1 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart1),
	.resource	= resources_uart1,
};

#define MSM_UART1DM_PHYS      0xA0200000
static struct resource msm_uart1_dm_resources[] = {
	{
		.start	= MSM_UART1DM_PHYS,
		.end	= MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_UART1DM_IRQ,
		.end	= INT_UART1DM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_UART1DM_RX,
		.end	= INT_UART1DM_RX,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DMOV_HSUART1_TX_CHAN,
		.end	= DMOV_HSUART1_RX_CHAN,
		.name	= "uartdm_channels",
		.flags	= IORESOURCE_DMA,
	},
	{
		.start	= DMOV_HSUART1_TX_CRCI,
		.end	= DMOV_HSUART1_RX_CRCI,
		.name	= "uartdm_crci",
		.flags	= IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);
struct platform_device msm_device_uart_dm1 = {
	.name	= "msm_serial_hs",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(msm_uart1_dm_resources),
	.resource	= msm_uart1_dm_resources,
	.dev	= {
		.dma_mask		= &msm_uart_dm1_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

#define MSM_NAND_PHYS		0xA0A00000
static struct resource resources_nand[] = {
	[0] = {
		.name   = "msm_nand_dmac",
		.start	= DMOV_NAND_CHAN,
		.end	= DMOV_NAND_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	[1] = {
		.name   = "msm_nand_phys",
		.start  = MSM_NAND_PHYS,
		.end    = MSM_NAND_PHYS + 0x7FF,
		.flags  = IORESOURCE_MEM,
	},
};

struct flash_platform_data msm_nand_data;

struct platform_device msm_device_nand = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_nand),
	.resource	= resources_nand,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

#define MSM_SDC1_BASE         0xA0400000
#define MSM_SDC2_BASE         0xA0500000
#define MSM_SDC3_BASE         0xA0600000
#define MSM_SDC4_BASE         0xA0700000
static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_BASE,
		.end	= MSM_SDC1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_1,
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
		.start	= INT_SDC2_0,
		.end	= INT_SDC2_1,
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
		.start	= INT_SDC3_0,
		.end	= INT_SDC3_1,
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
		.start	= INT_SDC4_0,
		.end	= INT_SDC4_1,
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

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 1 || controller > 4)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

struct clk_lookup msm_clocks_7x27a[] = {
	CLK_PCOM("adm_clk",	ADM_CLK,	NULL, 0),
	CLK_PCOM("adsp_clk",	ADSP_CLK,	NULL, 0),
	CLK_PCOM("csi_clk",		CSI0_CLK,	NULL, 0),
	CLK_PCOM("csi_pclk",		CSI0_P_CLK,	NULL, 0),
	CLK_PCOM("csi_vfe_clk",	CSI0_VFE_CLK,	NULL, 0),
	CLK_PCOM("csi_clk",		CSI1_CLK,	NULL, 0),
	CLK_PCOM("csi_pclk",		CSI1_P_CLK,	NULL, 0),
	CLK_PCOM("csi_vfe_clk",	CSI1_VFE_CLK,	NULL, 0),
	CLK_PCOM("dsi_byte_clk",	DSI_BYTE_CLK,	NULL, OFF),
	CLK_PCOM("dsi_clk",		DSI_CLK,	NULL, OFF),
	CLK_PCOM("dsi_esc_clk",	DSI_ESC_CLK,	NULL, OFF),
	CLK_PCOM("dsi_pixel_clk",	DSI_PIXEL_CLK,	NULL, OFF),
	CLK_PCOM("ebi1_clk",	EBI1_CLK,	NULL, CLK_MIN),
	CLK_PCOM("ebi2_clk",	EBI2_CLK,	NULL, 0),
	CLK_PCOM("ecodec_clk",	ECODEC_CLK,	NULL, 0),
	CLK_PCOM("gp_clk",	GP_CLK,		NULL, 0),
	CLK_PCOM("grp_clk",	GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("grp_pclk",	GRP_3D_P_CLK,	NULL, 0),
	CLK_PCOM("gsbi_qup_clk", GSBI1_QUP_CLK,	"qup_i2c.0", OFF),
	CLK_PCOM("gsbi_qup_clk", GSBI2_QUP_CLK,	"qup_i2c.1", OFF),
	CLK_PCOM("gsbi_qup_pclk", GSBI1_QUP_P_CLK,	"qup_i2c.0", OFF),
	CLK_PCOM("gsbi_qup_pclk", GSBI2_QUP_P_CLK,	"qup_i2c.1", OFF),
	CLK_PCOM("hdmi_clk",	HDMI_CLK,	NULL, 0),
	CLK_PCOM("i2c_clk",	I2C_CLK,	"msm_i2c.0", 0),
	CLK_PCOM("icodec_rx_clk",	ICODEC_RX_CLK,	NULL, 0),
	CLK_PCOM("icodec_tx_clk",	ICODEC_TX_CLK,	NULL, 0),
	CLK_PCOM("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLK_PCOM("mdc_clk",	MDC_CLK,	NULL, 0),
	CLK_PCOM("mddi_clk",	PMDH_CLK,	NULL, OFF | CLK_MINMAX),
	CLK_PCOM("mdp_clk",	MDP_CLK,	NULL, OFF),
	CLK_PCOM("mdp_lcdc_pclk_clk", MDP_LCDC_PCLK_CLK, NULL, 0),
	CLK_PCOM("mdp_lcdc_pad_pclk_clk", MDP_LCDC_PAD_PCLK_CLK, NULL, 0),
	CLK_PCOM("mdp_vsync_clk",	MDP_VSYNC_CLK,	NULL, OFF),
	CLK_PCOM("pbus_clk",	PBUS_CLK,	NULL, CLK_MIN),
	CLK_PCOM("pcm_clk",	PCM_CLK,	NULL, 0),
	CLK_PCOM("rotator_clk",	AXI_ROTATOR_CLK,	NULL, OFF),
	CLK_PCOM("sdac_clk",	SDAC_CLK,	NULL, OFF),
	CLK_PCOM("sdc_clk",	SDC1_CLK,	"msm_sdcc.1", OFF),
	CLK_PCOM("sdc_pclk",	SDC1_P_CLK,	"msm_sdcc.1", OFF),
	CLK_PCOM("sdc_clk",	SDC2_CLK,	"msm_sdcc.2", OFF),
	CLK_PCOM("sdc_pclk",	SDC2_P_CLK,	"msm_sdcc.2", OFF),
	CLK_PCOM("sdc_clk",	SDC3_CLK,	"msm_sdcc.3", OFF),
	CLK_PCOM("sdc_pclk",	SDC3_P_CLK,	"msm_sdcc.3", OFF),
	CLK_PCOM("sdc_clk",	SDC4_CLK,	"msm_sdcc.4", OFF),
	CLK_PCOM("sdc_pclk",	SDC4_P_CLK,	"msm_sdcc.4", OFF),
	CLK_PCOM("tsif_clk",	TSIF_CLK,	NULL, 0),
	CLK_PCOM("tsif_ref_clk", TSIF_REF_CLK,	NULL, 0),
	CLK_PCOM("tsif_pclk",	TSIF_P_CLK,	NULL, 0),
	CLK_PCOM("uart_clk",	UART1_CLK,	"msm_serial.0", OFF),
	CLK_PCOM("uart_clk",	UART2_CLK,	"msm_serial.1", 0),
	CLK_PCOM("uartdm_clk",	UART1DM_CLK,	"msm_serial_hs.0", OFF),
	CLK_PCOM("uartdm_clk",	UART2DM_CLK,	"msm_serial_hs.1", 0),
	CLK_PCOM("usb_hs_clk",	USB_HS_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs_pclk",	USB_HS_P_CLK,	NULL, OFF),
	CLK_PCOM("usb_otg_clk",	USB_OTG_CLK,	NULL, 0),
	CLK_PCOM("usb_phy_clk",	USB_PHY_CLK,	NULL, 0),
	CLK_PCOM("vdc_clk",	VDC_CLK,	NULL, OFF | CLK_MIN),
	CLK_PCOM("vfe_clk",	VFE_CLK,	NULL, OFF),
	CLK_PCOM("vfe_mdc_clk",	VFE_MDC_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs_core_clk",	USB_HS_CORE_CLK,	NULL, OFF),

	CLK_VOTER("ebi1_acpu_clk",	EBI_ACPU_CLK,	"ebi1_clk", NULL, 0),
	CLK_VOTER("ebi1_kgsl_clk",	EBI_KGSL_CLK,	"ebi1_clk", NULL, 0),
	CLK_VOTER("ebi1_lcdc_clk",	EBI_LCDC_CLK,	"ebi1_clk", NULL, 0),
	CLK_VOTER("ebi1_mddi_clk",	EBI_MDDI_CLK,	"ebi1_clk", NULL, 0),
	CLK_VOTER("ebi1_usb_clk",	EBI_USB_CLK,	"ebi1_clk", NULL, 0),
	CLK_VOTER("ebi1_vfe_clk",	EBI_VFE_CLK,	"ebi1_clk", NULL, 0),
};

unsigned msm_num_clocks_7x27a = ARRAY_SIZE(msm_clocks_7x27a);
