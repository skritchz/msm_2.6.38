/*
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
/*
 * Qualcomm PMIC 8018 driver header file
 *
 */

#ifndef __MFD_PM8018_H
#define __MFD_PM8018_H

#include <linux/device.h>
#include <linux/mfd/pm8xxx/irq.h>
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/mfd/pm8xxx/mpp.h>
#include <linux/mfd/pm8xxx/rtc.h>
#include <linux/input/pmic8xxx-pwrkey.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/regulator/pm8018-regulator.h>

#define PM8018_CORE_DEV_NAME "pm8018-core"

#define PM8018_NR_IRQS		256

#define PM8018_NR_GPIOS		6

#define PM8018_NR_MPPS		6

#define PM8018_GPIO_BLOCK_START	24
#define PM8018_MPP_BLOCK_START	16
#define PM8018_IRQ_BLOCK_BIT(block, bit) ((block) * 8 + (bit))

/* GPIOs and MPPs [1,N] */
#define PM8018_GPIO_IRQ(base, gpio)	((base) + \
		PM8018_IRQ_BLOCK_BIT(PM8018_GPIO_BLOCK_START, (gpio)-1))
#define PM8018_MPP_IRQ(base, mpp)	((base) + \
		PM8018_IRQ_BLOCK_BIT(PM8018_MPP_BLOCK_START, (mpp)-1))

/* PMIC Interrupts */
#define PM8018_RTC_ALARM_IRQ		PM8018_IRQ_BLOCK_BIT(4, 7)

#define PM8018_PWRKEY_REL_IRQ		PM8018_IRQ_BLOCK_BIT(6, 2)
#define PM8018_PWRKEY_PRESS_IRQ		PM8018_IRQ_BLOCK_BIT(6, 3)

struct pm8018_platform_data {
	struct pm8xxx_irq_platform_data		*irq_pdata;
	struct pm8xxx_gpio_platform_data	*gpio_pdata;
	struct pm8xxx_mpp_platform_data		*mpp_pdata;
	struct pm8xxx_rtc_platform_data         *rtc_pdata;
	struct pm8xxx_pwrkey_platform_data	*pwrkey_pdata;
	struct pm8xxx_misc_platform_data	*misc_pdata;
	struct pm8018_regulator_platform_data	*regulator_pdatas;
	int					num_regulators;
};

#endif
