/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/ipu.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/uio_driver.h>
#include <linux/iram_alloc.h>
#include <linux/fsl_devices.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

static const char * const gpio_names0[32] = {
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL,
	NULL, NULL, NULL,
	"iPod Reset", 						// 16
	"DO_TP1055", 						// 17
	NULL,
	"NULL", 						// 19
	"NULL", 						// 20
	"NULL", 					// 21
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
};

static const char * const gpio_names1[32] = {
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL,
	"DO_TP1053",						//  8
	NULL,
	"VIP IRQ",							// 10
	"Reverse",							// 11
	"NULL",						// 12
	"DO_VD_PWRDWN",						// 13
	NULL,
	"DO_TP1054",						// 15
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL
};

static const char * const gpio_names2[32] = {
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL,
	"GPS Wakeup",						// 19
	"Watchdog Interrupt",				// 20
	NULL, NULL,
	"Watchdog Disable",					// 23
	NULL,
	"GPS Boot0",						// 25
	"Reset", 							// 26
	"DO_VD_RESET",						// 27
	NULL, NULL, NULL, NULL,
};
static const char * const gpio_names3[32] = {
	NULL, NULL, NULL, NULL, NULL,
	"Debug LED",						//  5
	"NULL",					             //  6
	"Reset VIP",						//  7
	NULL, NULL,
	"ACC_Sense",						// 10
	NULL, NULL, NULL, NULL, NULL,
	"DO_BT_EN",							// 16
	NULL, NULL,
	"DO_TP1056",						// 19
	NULL, NULL, NULL, NULL, NULL,
	"DO_WL_EN", 						// 25
	"DI_WL_IRQ",						// 26
	"NFC IRQ", 							// 27
	NULL, NULL, NULL, NULL,
};
static const char * const gpio_names4[32] = {
	NULL, NULL,
	"LVDS Line Diagnostics",			//  2
	NULL, NULL, NULL,
	"DO_CODEC_RESET",					//  6
	"DO_TP1029",						//  7
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL,
	"NULL",					// 20
	"DO_TP1027",						// 21
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
};
static const char * const gpio_names5[32] = {
	"NULL",					//  0
	"GPS Reset",						//  1
	NULL, NULL,
	"LVDS_Line Fault",					//  4
	"DI_LVDS_INT",						//  5
	NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL,
	"DO_ENET_RST",						// 14
	NULL,
	"DI_VD_IRQ",						// 16
	NULL, NULL, NULL, NULL,
	"DO_TP1057",						// 21
	NULL, NULL, NULL, NULL, NULL, NULL,
	"DI_PMIC_INT",						// 28
	NULL, NULL,
	"NULL",						// 31
};
static const char * const  const gpio_names6[32] = {
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL,
};

#define GPIO_NAMES0		gpio_names0
#define GPIO_NAMES1		gpio_names1
#define GPIO_NAMES2		gpio_names2
#define GPIO_NAMES3		gpio_names3
#define GPIO_NAMES4		gpio_names4
#define GPIO_NAMES5		gpio_names5
#define GPIO_NAMES6		gpio_names6


static struct mxc_gpio_port mxc_gpio_ports[] = {
	{
		.chip.label = "gpio-0",
		.chip.names = GPIO_NAMES0,
		.base = IO_ADDRESS(GPIO1_BASE_ADDR),
		.irq = MXC_INT_GPIO1_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO1_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START
	},
	{
		.chip.label = "gpio-1",
		.chip.names = GPIO_NAMES1,
		.base = IO_ADDRESS(GPIO2_BASE_ADDR),
		.irq = MXC_INT_GPIO2_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO2_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 1
	},
	{
		.chip.label = "gpio-2",
		.chip.names = GPIO_NAMES2,
		.base = IO_ADDRESS(GPIO3_BASE_ADDR),
		.irq = MXC_INT_GPIO3_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO3_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 2
	},
	{
		.chip.label = "gpio-3",
		.chip.names = GPIO_NAMES3,
		.base = IO_ADDRESS(GPIO4_BASE_ADDR),
		.irq = MXC_INT_GPIO4_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO4_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 3
	},
	{
		.chip.label = "gpio-4",
		.chip.names = GPIO_NAMES4,
		.base = IO_ADDRESS(GPIO5_BASE_ADDR),
		.irq = MXC_INT_GPIO5_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO5_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 4
	},
	{
		.chip.label = "gpio-5",
		.chip.names = GPIO_NAMES5,
		.base = IO_ADDRESS(GPIO6_BASE_ADDR),
		.irq = MXC_INT_GPIO6_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO6_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 5
	},
	{
		.chip.label = "gpio-6",
		.chip.names = GPIO_NAMES6,
		.base = IO_ADDRESS(GPIO7_BASE_ADDR),
		.irq = MXC_INT_GPIO7_INT15_0_NUM,
		.irq_high = MXC_INT_GPIO7_INT31_16_NUM,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 6
	},
};

int mx6q_register_gpios(void)
{
	/* 7 ports for Mx6 */
	return mxc_gpio_init(mxc_gpio_ports, 7);
}
