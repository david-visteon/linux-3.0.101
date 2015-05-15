/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
 
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/wl12xx.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <mach/common.h>
#include <mach/iomux-mx6q.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "usb.h"

#include "board-mx6q_visteon_sx5.h"


/* ECSPI2 VIP SPI Port IRQ GPIO3_GPIO[30] configuration */
#define DI_ECSPI2_VIP_IRQ                 IMX_GPIO_NR(2, 10)

/* IO to control the reset state of the ADV7182 video decoder */
#define DO_VD_RESET                       IMX_GPIO_NR(3, 27)
/* IO to control the power down state of the ADV7182 video decoder */
#define DO_VD_PWRDWN                      IMX_GPIO_NR(2, 13)
/* IO to control the reset state of the tef6638 audio dsp */
#define DO_CODEC_RESET                    IMX_GPIO_NR(2, 13)

/**
 * The names in this array are not used to give an exported GPIO its name- that happens
 * inside devices.c. These names are only used for debugfs to display an alternate name
 * for an exported GPIO when viewing /sys/kernel/debug/gpio.
 */
static struct gpio_export board_gpio_exports[] = {
	    {
	        .name = "TP1054",
	        .altName = "DO_TP1054",
	        .id = IMX_GPIO_NR( 2, 15),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	   
	    {
	        .name = "Reverse",
	        .altName = "DI_VIP_SHIFTR",
	        .id = IMX_GPIO_NR( 2, 11),
	        .active_low = true,
	        .initial_level = false,
	        .dir_in = true,
	        .redirectable = true,
	    },
	    {
	        .name = "VIP IRQ",
	        .altName = "DI_VIP_IRQ",
	        .id = IMX_GPIO_NR( 2, 10),
	        .active_low = true,
	        .initial_level = true,
	        .dir_in = true,
	        .redirectable = true,
	    },
	    {
	        .name = "TP1053",
	        .altName = "DO_TP1053",
	        .id = IMX_GPIO_NR( 2,  8),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	    {
	        .name = "Reset",
	        .altName = "DO_SW_RESET",
	        .id = IMX_GPIO_NR( 3, 26),
	        .active_low = true,
	        .initial_level = true,
	        .dir_in = false,
	        .redirectable = true,
	    },
	    {
	        .name = "Watchdog Disable",
	        .altName = "DO_WDI_DISABLE",
	        .id = IMX_GPIO_NR( 3, 23),
	        .active_low = false,
	        .initial_level = true,
	        .dir_in = false,
	        .redirectable = true,
	    },
	    {
	        .name = "Watchdog Interrupt",
	        .altName = "DO_WDI",
	        .id = IMX_GPIO_NR( 3, 20),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	    {
	        .name = "GPS Wakeup",
	        .altName = "DO_GPS_WAKEUP",
	        .id = IMX_GPIO_NR( 3, 19),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	   
	    {
	        .name = "TP1056",
	        .altName = "DO_TP1056",
	        .id = IMX_GPIO_NR( 4, 19),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	    {
	        .name = "ACC_Sense",
	        .altName = "DI_ACC_SENSE",
	        .id = IMX_GPIO_NR( 4, 10),
	        .active_low = true,
	        .initial_level = true,
	        .dir_in = true,
	        .redirectable = true,
	    },
	    {
	        .name = "Reset VIP",
	        .altName = "DO_VIP_RESET",
	        .id = IMX_GPIO_NR( 4,  7),
	        .active_low = true,
	        .initial_level = true,
	        .dir_in = false,
	        .redirectable = true,
	    },
	   
	    {
	        .name = "Debug LED",
	        .altName = "DO_DEBUG_LED",
	        .id = IMX_GPIO_NR( 4,  5),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	    {
	        .name = "TP1027",
	        .altName = "DO_TP1027",
	        .id = IMX_GPIO_NR( 5, 21),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	    {
	        .name = "TP1029",
	        .altName = "DO_TP1029",
	        .id = IMX_GPIO_NR( 5,  7),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	   
	    {
	        .name = "TP1057",
	        .altName = "DO_TP1057",
	        .id = IMX_GPIO_NR( 6, 21),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
	    {
	        .name = "GPS Reset",
	        .altName = "DO_GPS_RESET",
	        .id = IMX_GPIO_NR( 6,  1),
	        .active_low = false,
	        .initial_level = false,
	        .dir_in = false,
	        .redirectable = true,
	    },
};


/* SX5-Board specific USB init of Universal Serial Bus Controller (USBOH3) OTG and USB Host 1 devices. */
static void __init imx6q_init_usb(void) {
	/* Set base address of USBOH3 (USB)  controller. */
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
}

#ifndef __BRING_UP__

static struct spi_board_info mx6q_spi2_board_info[] __initdata = {
	{
		/* The modalias must be the same as SPI device driver name */
		.modalias = "spidev",
		.max_speed_hz = 4000000,
		.bus_num = 1,
		.chip_select = 0,
		.irq = DI_ECSPI2_VIP_IRQ, /* GPIO2_GPIO[10] */
		.mode = 0,
		.controller_data = NULL,
	},
};

/* HDMI SPI */
static struct spi_board_info mx6q_spi4_board_info[] __initdata = {
	{
		/* The modalias must be the same as SPI device driver name */
		.modalias = "spidev",
		.max_speed_hz = 1000000,
		.bus_num = 3,
		.chip_select = 0,
		.irq = -EINVAL,
		.mode = 0,
		.controller_data = NULL,
	},
};

static void __init mx6q_spi_device_init(void) {
	//gpio_set_value(MX6Q_ARM2_ECSPI1_SS0, 1);
	gpio_set_value(MX6Q_ARM2_ECSPI2_SS0, 1);
	//gpio_set_value(MX6Q_ARM2_ECSPI4_SS0, 1);
	gpio_set_value(MX6Q_ARM2_ECSPI3_SS0,1);
	
	//spi_register_board_info(mx6q_cmu_spi1_board_info, ARRAY_SIZE(mx6q_cmu_spi1_board_info));
	spi_register_board_info(mx6q_spi2_board_info, ARRAY_SIZE(mx6q_spi2_board_info));
	spi_register_board_info(mx6q_spi4_board_info, ARRAY_SIZE(mx6q_spi4_board_info));
}
#endif


/* I2C1 addresses */
static struct i2c_board_info mx6q_i2c1_board_info[] __initdata = {
	{
		/* Texas Instruments AIC3104 audio CODEC. */
		I2C_BOARD_INFO("aic310x", 0x18),
	},
};

/* I2C2 addresses: Linux wants 7 bit, without the r/w bit, so (addr >> 1) */
static struct i2c_board_info mx6q_i2c2_board_info[] __initdata = {
	{
		/* Check addr of Analog Devices ADV7182 Video Decoder */
		I2C_BOARD_INFO("adv7180", 0x20),
		//.platform_data = (void *)&tvin_7180_data,
	},
};

/* I2C3 addresses: Linux wants 7 bit, without the r/w bit, so (addr >> 1) */
static struct i2c_board_info mx6q_i2c3_board_info[] __initdata = {
	{
		/* Apple authentication coprocessor v2.0B (AUTH-IC) */
		I2C_BOARD_INFO("auth-ic", 0x10),
	},
};

/*
 * SX5 board specific consumer regulator initialisation stuff.
 */
static struct regulator_consumer_supply mx6q_cmu_vmmc_consumers[] __initdata = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data mx6q_cmu_vmmc_init __initdata = {
	.num_consumer_supplies = ARRAY_SIZE(mx6q_cmu_vmmc_consumers),
	.consumer_supplies = mx6q_cmu_vmmc_consumers,
};

static struct fixed_voltage_config mx6q_cmu_vmmc_reg_config __initdata = {
	.supply_name ="vmmc",
	.microvolts = 3300000,
	.gpio = -1,
	.init_data = &mx6q_cmu_vmmc_init,
};

static struct platform_device mx6q_cmu_vmmc_reg_devices __initdata = {
	.name = "reg-fixed-voltage",
	.id = 0,
	.dev = {
		.platform_data = &mx6q_cmu_vmmc_reg_config,
	},
};

/*
 * network initialisation stuff. 
 */
#ifdef CONFIG_PHYLIB
/* PHY power down */
#define BMCR_PDOWN		0x0800
static int mx6q_cmu_fec_phy_init(struct phy_device *phydev) {
	/* Check PHY power state. */
	unsigned short val = phy_read(phydev, 0x0);
	if (val & BMCR_PDOWN){
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));
	}
	return 0;
}

/* Initialization data of SMSC LAN8720A 10/100 RMII ethernet device. */
static struct fec_platform_data mx6q_cmu_fec_data __initdata = {
	.init = mx6q_cmu_fec_phy_init,
	/* Reduced media independent interface ethernet PHY  */
	.phy = PHY_INTERFACE_MODE_RMII,
};
#endif

//set_power & power_off_card is not defined in new fs driver
/* Setup for for uSDHC2, IMX6 sx5 board WIFI 802.11 WLAN (50Mbps). */
static const struct esdhc_platform_data mx6q_sd2_data __initconst = {
	.cd_gpio = -EINVAL,
	.wp_gpio = -EINVAL,
	.cd_type = ESDHC_CD_PERMANENT,
	.always_present = 1,
	.keep_power_at_suspend = 0,
	.support_18v = 1,
	.support_8bit = 0,
	.set_power = wl12xx_wlan_set_power,
	.power_off_card = 1,
};

/* Setup for for uSDHC3, IMX6 sx5 board Managed NAND device. (Micron 8bit, 16 GByte eMMC). */
static const struct esdhc_platform_data mx6q_sd3_data __initconst = {
	.cd_gpio = -EINVAL,
	.wp_gpio = -EINVAL,
	.cd_type = ESDHC_CD_PERMANENT,
	.always_present = 1,
	.keep_power_at_suspend = 0,
	.support_18v = 1,
	.support_8bit = 1,
	.delay_line = 0,
};

/* LVDS Display Bridge (LDB) initial data structure */
static struct fsl_mxc_ldb_platform_data mx6q_ldb_data __initdata = {
	.ipu_id = 0,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SIN0,
};

/*
 * IPU1_DI0, display interface that feeds the LDB0 Gigabit Multimedia Serial Link (GMSL) serializer.
 * IPU1_DI1, display interface that feeds the LDB1 Flat Panel Display Link (FPD Link II) serializer.
 * IPU2_DI0, display interface that feeds the High Definition Media Interface (HDMI) output port.
 * IPU2_DI1, display interface that feeds the MIPI-DSI output port.
 * IPU2_CSI1 is the camera sensor interface that captures video from one of two input sources(Video Decoder, HDMI Receiver)
 */
static struct fsl_mxc_capture_platform_data mx6q_capture_data[] __initdata = {
	{
		.csi = 1,
		.ipu = 1,
		.mclk_source = 0,
		.is_mipi = 0,
	}
};

static struct mxc_audio_platform_data mx6_audio_dsp_data;
static struct mxc_audio_platform_data mx6_SCO_audio_data;

static int mx6_init_BT_audio_clk(const char *clk, int round_rate)
{
	struct clk *ssi_clk, *new_parent;
	int rate;

	ssi_clk = clk_get_sys(clk, NULL);
	if (IS_ERR(ssi_clk)) {
		pr_err("Can't get clock for %s.\n",clk);
		return -EINVAL;
	}

	new_parent = clk_get(NULL, "pll4");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(ssi_clk, new_parent);
		clk_put(new_parent);
	} else {
		pr_err("Can't get parent clock pll4.\n");
		return -EINVAL;
	}

	rate = clk_round_rate(ssi_clk, round_rate);
	pr_debug("%s clock: %d\n", clk, rate);
	clk_set_rate(ssi_clk, rate);

	return rate;
}


static int mx6_SCO_audio_init(void)
{
	int rate;
	
	/* Initialize the BT SCO SSI2 to a 12.288 MHz sysclk from PLL4 */
	rate = mx6_init_BT_audio_clk("imx-ssi.1", 12288000);
	if (rate < 0) {
		return rate;
	}
	mx6_SCO_audio_data.sysclk = rate;
	
	return 0;
}

static struct clk *clko2 = NULL;

static int mx6_tef663x_init(void)
{
	/* Do nothing at the moment */
	return 0;
}

static int mx6_tef663x_clock_enable(int enable)
{
#if 0
	//no necessary for tef663x?
#endif
	return 0;
}

static struct imx_ssi_platform_data mx6_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct imx_ssi_platform_data mx6_SCO_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

/* need to reconfig */
static struct mxc_audio_platform_data mx6_audio_dsp_data = {
	.src_port = 1,  
	.ext_port = 4,
	.init = mx6_tef663x_init,
	.clock_enable = mx6_tef663x_clock_enable,
	.rst_gpio = DO_CODEC_RESET,
};

static struct mxc_audio_platform_data mx6_SCO_audio_data = {
	.init = mx6_SCO_audio_init,
	.src_port = 2,
	.ext_port = 5,
};

static struct platform_device mx6_audio_dsp_device = {
	.name = "imx-tef663x",
};

static struct platform_device mx6_SCO_audio_device = {
	.name = "imx-wl1285q",
};

static struct platform_device mx6_SCO_audio_codec = {
	.name = "wl1285q-codec",
};

static int __init imx6q_init_audio(void)
{
	/* Configure CODEC reset gpio */
	gpio_request(DO_CODEC_RESET, "DO_CODEC_RESET");
	gpio_direction_output(DO_CODEC_RESET, 0);

	/* Configure the tef663x audio dsp */
	mxc_register_device(&mx6_audio_dsp_device,
			    &mx6_audio_dsp_data);
	imx6q_add_imx_ssi(0, &mx6_ssi_pdata);

	/* Configure the BT SCO audio channel */
	mxc_register_device(&mx6_SCO_audio_device,
			    &mx6_SCO_audio_data);
	mxc_register_device(&mx6_SCO_audio_codec,
			    &mx6_SCO_audio_data);
	imx6q_add_imx_ssi(1, &mx6_SCO_ssi_pdata);

	return 0;
}

/* export the list of GPIO required by user space */
static void board_gpio_export(void)
{
	struct gpio_export *export_gpio = &board_gpio_exports[0];
	int export_count = sizeof(board_gpio_exports)/sizeof(board_gpio_exports[0]);
	int status;

	pr_warning( "sx5 board_gpio_export: %d",  export_count);
	while( export_count-- > 0) {

		pr_debug( "Exporting '%s'",  export_gpio->name);
		status = gpio_request( export_gpio->id, export_gpio->altName);
		if( 0 != status)
		{
			pr_err( "Error '%d' requesting '%s'", status, export_gpio->name);
			continue;
		}

		status = gpio_export( export_gpio->id, export_gpio->redirectable);
		if( 0 != status)
		{
			pr_err( "Error '%d' exporting '%s'", status, export_gpio->name);
			gpio_free( export_gpio->id);
			continue;
		}

		pr_debug( "'%s' is active_low: %d",  export_gpio->name,  export_gpio->active_low);
        gpio_sysfs_set_active_low( export_gpio->id,  export_gpio->active_low);

		if( true == export_gpio->dir_in) {

			pr_debug( "'%s' is an input",  export_gpio->name);
			gpio_direction_input( export_gpio->id);
#if 0
			{
				unsigned char edge_sys_file[128];
				int fd = -1;
				mm_segment_t old_fs = get_fs();
				set_fs(KERNEL_DS);

				strcpy(edge_sys_file, "/sys/class/gpio/");
	        	strcat(edge_sys_file, export_gpio->name);
	        	strcat(edge_sys_file, "/edge");
	        	fd = sys_open( edge_sys_file, O_RDWR | O_NDELAY, 0644);
	        	if (fd >= 0) {
	        		char *edge_str = "none";

	        		if( GPIO_EDGE_BOTH == export_gpio->edge_irq) edge_str = "both";
	        		else if( GPIO_EDGE_RISING == export_gpio->edge_irq) edge_str = "rising";
	        		else if( GPIO_EDGE_RISING == export_gpio->edge_irq) edge_str = "falling";

	        		sys_write( fd, edge_str, strlen( edge_str));
	        		sys_close( fd);
	        	}
	        	else {
	        		pr_err( "Failed to open '%s': %d",  edge_sys_file, fd);
	        	}

				set_fs(old_fs);

			}
#endif
		}
		else {
			pr_debug( "'%s' is an output: %d",  export_gpio->name, export_gpio->initial_level);
			gpio_direction_output( export_gpio->id, export_gpio->initial_level);
		}

        export_gpio++;
	}
}

static void __init mx6q_fixup_sx5_board(struct machine_desc *desc, struct tag *tags, char **cmdline, struct meminfo *mi) {
	/* Do nothing */
}

static void __init mx6q_sx5_board_init(void) {

	/* Toggle GPIO4_28 ON, Only for debug purpose. */
	toggle_gpio4_28(1);

	/* Add the UART devices */
	mx6q_init_uart();

	/* Add DMA support */
	imx6q_add_dma();
	/* Add On-Chip OTP controller device. */
	imx6q_add_otp();
	/* Add Virtual IC Identification Module device. */
	imx6q_add_viim();

	/* Initialize the CPU regulator */
	gp_reg_id = mx6q_dvfscore_data.reg_id;
	soc_reg_id = mx6q_dvfscore_data.soc_id;
	pu_reg_id = mx6q_dvfscore_data.pu_id;
	imx6q_add_dvfs_core(&mx6q_dvfscore_data);
	//mx6_cpu_regulator_init();

	/* Add temperature sensor driver. */
	imx6q_add_anatop_thermal_imx(1, &mx6q_anatop_thermal_data);

#ifdef CONFIG_PHYLIB
	/* Add the Ethernet 10/100 Local Area Network Device. */
	imx6_init_fec(mx6q_cmu_fec_data);
#endif

	/* Add the Secure Real Time Clock (SRTC). */
	imx6q_add_imx_snvs_rtc();

	platform_device_register(&mx6q_cmu_vmmc_reg_devices);

	/* Add the Ultra Secure Digital Host Controller (uSDHC2) device for the Wireless Module. */
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_sd2_data);
	/* Add the Ultra Secure Digital Host Controller (uSDHC2) device for the Managed NAND. */
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sd3_data);

	/* Add I2C1, I2C2 and I2C3 devices and register all devices. */
	imx6q_add_imx_i2c(0, &mx6q_i2c1_data);
	imx6q_add_imx_i2c(1, &mx6q_i2c2_data);
	imx6q_add_imx_i2c(2, &mx6q_i2c3_data);
	i2c_register_board_info(0, mx6q_i2c1_board_info, ARRAY_SIZE(mx6q_i2c1_board_info));
	i2c_register_board_info(1, mx6q_i2c2_board_info, ARRAY_SIZE(mx6q_i2c2_board_info));
	i2c_register_board_info(2, mx6q_i2c3_board_info, ARRAY_SIZE(mx6q_i2c3_board_info));

	imx6q_add_ecspi(1, &mx6q_spi2_data);
	imx6q_add_ecspi(3, &mx6q_spi4_data);
	mx6q_spi_device_init();

	/* Add the USB device */
	imx6q_init_usb();
	/* Add the display and graphics */
	imx6q_add_ipuv3(0, &mx6q_ipu_data[0]);
	imx6q_add_ipuv3(1, &mx6q_ipu_data[1]);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_platform_data);
	imx6q_add_vpu();

	imx6q_add_ipuv3fb(0, &mx6q_frame_buffer_data[0]);

	/* Add LVDS Display Bridge (LDB) */
	imx6q_add_ldb(&mx6q_ldb_data);

	/* For MX6Q GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 */
	/* Do not set this register, otherwise FEC does not work */
	mxc_iomux_set_gpr_register(1, 19, 1, 1);
	mxc_iomux_set_gpr_register(1, 20, 1, 1);

	/* Add video capture and output devices */
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &mx6q_capture_data[0]);


	/* Add audio support */
	imx6q_init_audio();

	/* Initialized and powering on the Bluetooth module. */
	mx6q_bluetooth_init();

	/* Initialized and powering on the WLAN module. */
	mx6q_wl12xx_wlan_init();
//#endif

	/* Add CAAM device */
	imx6q_add_imx_caam();
	/* Export GPIOs needed by user space */
	board_gpio_export();

	/* Toggle GPIO4_28 OFF, Only for debug purpose. */
	toggle_gpio4_28(0);
}

static struct sys_timer mx6q_sx5_timer = {
   .init = mx6q_timer_init,
};

/*
 * initialize __mach_desc_MX6Q_VISTEON_SX5 data structure.
 */
MACHINE_START(MX6Q_VISTEON_SX5, "Visteon i.MX 6Quad Core Visteon SX5 Board")
	.boot_params  = MX6_PHYS_OFFSET + 0x100,
	.fixup        = mx6q_fixup_sx5_board,
	.map_io       = mx6_map_io,
	.init_irq     = mx6_init_irq,
	.init_machine = mx6q_sx5_board_init,
	.timer        = &mx6q_sx5_timer,
	.reserve      = mx6q_gpu_memory_reserve,
MACHINE_END

