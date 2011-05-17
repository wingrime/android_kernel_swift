/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * The MSM peripherals are spread all over across 768MB of physical
 * space, which makes just having a simple IO_ADDRESS macro to slide
 * them into the right virtual location rough.  Instead, we will
 * provide a master phys->virt mapping for peripherals here.
 *
 */

#ifndef __ASM_ARCH_MSM_IOMAP_8960_H
#define __ASM_ARCH_MSM_IOMAP_8960_H

/* Physical base address and size of peripherals.
 * Ordered by the virtual base addresses they will be mapped at.
 *
 * If you add or remove entries here, you'll want to edit the
 * msm_io_desc array in arch/arm/mach-msm/io.c to reflect your
 * changes.
 *
 */


#define MSM_QGIC_DIST_BASE	IOMEM(0xF0000000)
#define MSM_QGIC_DIST_PHYS	0x02000000
#define MSM_QGIC_DIST_SIZE	SZ_4K

#define MSM_QGIC_CPU_BASE	IOMEM(0xF0001000)
#define MSM_QGIC_CPU_PHYS	0x02002000
#define MSM_QGIC_CPU_SIZE	SZ_4K

#define MSM_TMR_BASE		IOMEM(0xF0100000)
#define MSM_TMR_PHYS		0x0200A000
#define MSM_TMR_SIZE		(SZ_4K)

#define MSM_TMR0_BASE		IOMEM(0xF0101000)
#define MSM_TMR0_PHYS		0x0208A000
#define MSM_TMR0_SIZE		(SZ_4K)

#define MSM_RPM_BASE		IOMEM(0xF0102000)
#define MSM_RPM_PHYS		0x00108000
#define MSM_RPM_SIZE		(SZ_4K)

#define MSM_SHARED_RAM_BASE	IOMEM(0xF0200000)
#define MSM_SHARED_RAM_SIZE	SZ_1M

#define MSM_DMOV_BASE		IOMEM(0xF0300000)
#define MSM_DMOV_PHYS		0x18300000
#define MSM_DMOV_SIZE		SZ_1M

#define MSM_APCS_GCC_BASE       IOMEM(0xFA006000)
#define MSM_APCS_GCC_PHYS       0x02011000
#define MSM_APCS_GCC_SIZE       SZ_4K

#define MSM_MMSS_CLK_CTL_BASE	IOMEM(0xFA014000)
#define MSM_MMSS_CLK_CTL_PHYS	0x04000000
#define MSM_MMSS_CLK_CTL_SIZE	SZ_4K

#define MSM_SIC_NON_SECURE_BASE IOMEM(0xFA600000)
#define MSM_SIC_NON_SECURE_PHYS 0x12100000
#define MSM_SIC_NON_SECURE_SIZE SZ_64K

#define MSM_GPT_BASE		(MSM_TMR_BASE + 0x4)
#define MSM_DGT_BASE		(MSM_TMR_BASE + 0x24)

#define MSM_HSUSB_PHYS		0x12500000
#define MSM_HSUSB_SIZE		SZ_4K

#define MSM_TLMM_BASE		IOMEM(0xF0400000)
#define MSM_TLMM_PHYS		0x00800000
#define MSM_TLMM_SIZE		SZ_16K

#endif
