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
#include <linux/i2c.h>
#include <linux/usb/android_composite.h>
#include <linux/msm_ssbi.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/irqs.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach/mmc.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_spi.h>
#include <mach/msm_hsusb.h>
#include <mach/usbdiag.h>
#include <mach/socinfo.h>
#include <mach/usb_gadget_fserial.h>

#include "timer.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include "gpiomux.h"

struct msm_gpiomux_config msm8960_gpiomux_configs[NR_GPIO_IRQS] = {};

static int __init gpiomux_init(void)
{
	int rc;

	rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err(KERN_ERR "msm_gpiomux_init failed %d\n", rc);
		return rc;
	}

	msm_gpiomux_install(msm8960_gpiomux_configs,
			ARRAY_SIZE(msm8960_gpiomux_configs));

	return 0;
}

#define MSM_SHARED_RAM_PHYS 0x40000000

/* Macros assume PMIC GPIOs and MPPs start at 1 */
#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_MPP_BASE			(PM8921_GPIO_BASE + PM8921_NR_GPIOS)
#define PM8921_MPP_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_MPP_BASE)
#define PM8921_IRQ_BASE			(NR_MSM_IRQS + NR_GPIO_IRQS)
#define PM8921_MPP_IRQ_BASE		(PM8921_IRQ_BASE + NR_GPIO_IRQS)

static void __init msm8960_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
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

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm8960_sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
	.sdcc_v4_sup	= true
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm8960_sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.sdcc_v4_sup	= true
};
#endif

static void __init msm8960_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	/* SDC1 : eMMC card connected */
	msm_add_sdcc(1, &msm8960_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	/* SDC3: External card slot */
	msm_add_sdcc(3, &msm8960_sdc3_data);
#endif
}

static struct msm_spi_platform_data msm8960_qup_spi_gsbi1_pdata = {
	.max_clock_speed = 26000000,
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Qualcomm Incorporated",
	.product	= "Mass storage",
	.can_stall	= 1,
};

static struct msm_otg_platform_data msm_otg_pdata;

static struct usb_diag_platform_data usb_diag_pdata = {
	.ch_name = DIAG_LEGACY,
};

static struct usb_gadget_fserial_platform_data fserial_pdata = {
	.no_ports	= 2,
};
static char *usb_functions_default[] = {
	"diag",
	"modem",
	"nmea",
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
	"usb_mass_storage",
};

static char *usb_functions_all[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
	"usb_mass_storage",
};

struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x9018,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0x9017,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0x9018,
	.version	= 0x0100,
	.product_name		= "Qualcomm HSUSB Device",
	.manufacturer_name	= "Qualcomm Incorporated",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};

struct platform_device usb_diag_device = {
	.name	= "usb_diag",
	.id	= 0,
	.dev	= {
		.platform_data = &usb_diag_pdata,
	},
};

struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

struct platform_device usb_gadget_fserial_device = {
	.name	= "usb_fserial",
	.id	= -1,
	.dev	= {
		.platform_data = &fserial_pdata,
	},
};
struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
	.platform_data = &mass_storage_pdata,
	},
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
}

static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi4_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct platform_device *sim_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm8960_device_uart_gsbi2,
	&msm8960_device_ssbi_pm8921,
	&msm8960_device_qup_spi_gsbi1,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&usb_diag_device,
	&msm8960_device_qup_i2c_gsbi4,
	&msm_device_wcnss_wlan,
	&usb_mass_storage_device,
	&usb_gadget_fserial_device,
};

static struct platform_device *rumi3_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm8960_device_uart_gsbi5,
	&msm8960_device_ssbi_pm8921,
	&msm8960_device_qup_spi_gsbi1,
	&msm8960_device_qup_i2c_gsbi4,
	&msm_kgsl_3d0,
#ifdef CONFIG_MSM_KGSL_2D
	&msm_kgsl_2d0,
	&msm_kgsl_2d1,
#endif
	&msm_device_wcnss_wlan,
};

static void __init msm8960_i2c_init(void)
{
	msm8960_device_qup_i2c_gsbi4.dev.platform_data =
					&msm8960_i2c_qup_gsbi4_pdata;
}

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata __devinitdata = {
	.irq_base		= PM8921_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(104),
	.irq_trigger_flag	= IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata __devinitdata = {
	.gpio_base	= PM8921_GPIO_PM_TO_SYS(1),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata __devinitdata = {
	.mpp_base	= PM8921_MPP_PM_TO_SYS(1),
};

static struct pm8921_platform_data pm8921_platform_data __devinitdata = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
};

static struct msm_ssbi_platform_data msm8960_ssbi_pm8921_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name			= "pm8921-core",
		.platform_data		= &pm8921_platform_data,
	},
};

static void __init msm8960_sim_init(void)
{
	if (socinfo_init() < 0)
		pr_err("socinfo_init() failed!\n");
	msm_clock_init(msm_clocks_8960, msm_num_clocks_8960);
	msm8960_device_ssbi_pm8921.dev.platform_data =
				&msm8960_ssbi_pm8921_pdata;
	msm8960_device_qup_spi_gsbi1.dev.platform_data =
				&msm8960_qup_spi_gsbi1_pdata;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	gpiomux_init();
	msm8960_i2c_init();
	platform_add_devices(sim_devices, ARRAY_SIZE(sim_devices));
	msm8960_init_mmc();
}

static void __init msm8960_rumi3_init(void)
{
	if (socinfo_init() < 0)
		pr_err("socinfo_init() failed!\n");
	msm_clock_init(msm_clocks_8960, msm_num_clocks_8960);
	gpiomux_init();
	msm8960_device_ssbi_pm8921.dev.platform_data =
				&msm8960_ssbi_pm8921_pdata;
	msm8960_device_qup_spi_gsbi1.dev.platform_data =
				&msm8960_qup_spi_gsbi1_pdata;
	msm8960_i2c_init();
	platform_add_devices(rumi3_devices, ARRAY_SIZE(rumi3_devices));
	msm8960_init_mmc();
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
