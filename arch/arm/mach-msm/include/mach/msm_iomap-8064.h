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

#ifndef __ASM_ARCH_MSM_IOMAP_8064_H
#define __ASM_ARCH_MSM_IOMAP_8064_H

/* Physical base address and size of peripherals.
 * Ordered by the virtual base addresses they will be mapped at.
 *
 * If you add or remove entries here, you'll want to edit the
 * msm_io_desc array in arch/arm/mach-msm/io.c to reflect your
 * changes.
 *
 */

#define APQ8064_TMR_PHYS		0x0200A000
#define APQ8064_TMR_SIZE		SZ_4K

#define APQ8064_TMR0_PHYS		0x0208A000
#define APQ8064_TMR0_SIZE		SZ_4K

#define APQ8064_QGIC_DIST_PHYS		0x02000000
#define APQ8064_QGIC_DIST_SIZE		SZ_4K

#define APQ8064_QGIC_CPU_PHYS		0x02002000
#define APQ8064_QGIC_CPU_SIZE		SZ_4K

#define APQ8064_DMOV_PHYS		0x18300000
#define APQ8064_DMOV_SIZE		SZ_1M

#define APQ8064_TLMM_PHYS		0x00800000
#define APQ8064_TLMM_SIZE		SZ_16K

#define APQ8064_ACC0_PHYS		0x02088000
#define APQ8064_ACC0_SIZE		SZ_4K

#define APQ8064_ACC1_PHYS		0x02098000
#define APQ8064_ACC1_SIZE		SZ_4K

#define APQ8064_ACC2_PHYS		0x020A8000
#define APQ8064_ACC2_SIZE		SZ_4K

#define APQ8064_ACC3_PHYS		0x020B8000
#define APQ8064_ACC3_SIZE		SZ_4K

#define APQ8064_APCS_GCC_PHYS		0x02011000
#define APQ8064_APCS_GCC_SIZE		SZ_4K

#define APQ8064_CLK_CTL_PHYS		0x00900000
#define APQ8064_CLK_CTL_SIZE		SZ_16K

#define APQ8064_MMSS_CLK_CTL_PHYS	0x04000000
#define APQ8064_MMSS_CLK_CTL_SIZE	SZ_4K

#define APQ8064_LPASS_CLK_CTL_PHYS	0x28000000
#define APQ8064_LPASS_CLK_CTL_SIZE	SZ_4K

#define APQ8064_HFPLL_PHYS		0x00903000
#define APQ8064_HFPLL_SIZE		SZ_4K

#define APQ8064_IMEM_PHYS		0x2A03F000
#define APQ8064_IMEM_SIZE		SZ_4K

#endif
