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
 */

#ifndef _ARCH_ARM_MACH_MSM_QDSS_H_
#define _ARCH_ARM_MACH_MSM_QDSS_H_

/* Coresight management registers (0xF00-0xFCC) */
#define CS_ITCTRL		(0xF00)
#define CS_CLAIMSET		(0xFA0)
#define CS_CLAIMCLR		(0xFA4)
#define CS_LAR			(0xFB0)
#define CS_LSR			(0xFB4)
#define CS_AUTHSTATUS		(0xFB8)
#define CS_DEVID		(0xFC8)
#define CS_DEVTYPE		(0xFCC)
/* Peripheral id registers (0xFD0-0xFEC) */
#define CS_PIDR4		(0xFD0)
#define CS_PIDR5		(0xFD4)
#define CS_PIDR6		(0xFD8)
#define CS_PIDR7		(0xFDC)
#define CS_PIDR0		(0xFE0)
#define CS_PIDR1		(0xFE4)
#define CS_PIDR2		(0xFE8)
#define CS_PIDR3		(0xFEC)
/* Component id registers (0xFF0-0xFFC) */
#define CS_CIDR0		(0xFF0)
#define CS_CIDR1		(0xFF4)
#define CS_CIDR2		(0xFF8)
#define CS_CIDR3		(0xFFC)


#define TIMEOUT_US		100
#define MAGIC1			0xC5ACCE55
#define MAGIC2			0x0

#define BM(lsb, msb)		((BIT(msb) - BIT(lsb)) + BIT(msb))
#define BMVAL(val, lsb, msb)	((val & BM(lsb, msb)) >> lsb)
#define BVAL(val, n)		((val & BIT(n)) >> n)

/* TODO: clean this up */
void etb_enable(void);
void etb_disable(void);
void etb_dump(void);
void tpiu_disable(void);
void funnel_enable(uint8_t id, uint32_t port_mask);
void funnel_disable(uint8_t id, uint32_t port_mask);

#ifdef CONFIG_MSM_TRACE_ACROSS_PC
extern void etm_save_reg_check(void);
extern void etm_restore_reg_check(void);
#else
static inline void etm_save_reg_check(void) {}
static inline void etm_restore_reg_check(void) {}
#endif

#endif
