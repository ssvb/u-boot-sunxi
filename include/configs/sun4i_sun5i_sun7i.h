/*
 * (C) Copyright 2012-2013 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * Configuration settings for the Allwinner A10 (sun4i) CPU
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIG_H
#define __CONFIG_H

#define SOC_IS_SUN4I()			(soc_is_sun4i())
#define SOC_IS_SUN5I()			(soc_is_sun5i())
#define SOC_IS_SUN7I()			(soc_is_sun7i())

#define CONFIG_SUN4I_SUN5I_SUN7I

/*
 * The device may have a PMIC chip, but we don't explicitly initialize it
 * in this generic configuration.  The CPU core voltage provided by AXP209
 * by default is 1.25V and can't support really high CPU clock speeds.
 * So we just assume that 384MHz is going to be safe (it is the reset
 * default for PLL1).
 */
#define CONFIG_CLK_FULL_SPEED		384000000

#define CONFIG_SYS_PROMPT		"sunxi# "

/*
 * The generic configuration does not initialize PSCI, so only a single
 * CPU core will be used on Allwinner A20.
 */

/*
 * Include common sunxi configuration where most the settings are
 */
#include <configs/sunxi-common.h>

#endif /* __CONFIG_H */
