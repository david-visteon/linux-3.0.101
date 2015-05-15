#ifndef __BOARD_MX6Q_COMMON_H
#define __BOARD_MX6Q_COMMON_H

#include <mach/imx_rfkill.h>
#include <mtd/mtd-abi.h>

/* SPI bus chip selects */
#if 0
/* DO_NOR_CS - ESCPI1_SSO */
#define MX6Q_ARM2_ECSPI1_SS0                   IMX_GPIO_NR(2, 30)
#endif

/* DO_VIP_CS - ESCPI2_SSO */
#define MX6Q_ARM2_ECSPI2_SS0                   IMX_GPIO_NR(2, 26)
/* HDMI_SPI_CS  - ESCPI3_SSO */
#define MX6Q_ARM2_ECSPI3_SS0                   IMX_GPIO_NR(4, 24)
#if 0
/* DO_NFC_CS - ESCPI4_SSO */
#define MX6Q_ARM2_ECSPI4_SS0                   IMX_GPIO_NR(3, 29)
#endif

/* IO to control the to activate Bluetooth interface. */
#define DO_BT_EN                               IMX_GPIO_NR(4, 16)
/* IO to control the to activate WLAN interface. */
#define DO_WL_EN                               IMX_GPIO_NR(4, 25)
/* General purpose input to capture WLAN interrupts. */
#define DI_WL_IRQ                              IMX_GPIO_NR(4, 26)
/* IO to control the to toggle GPIO4_28. */
#define SET_GPIO4_28                           IMX_GPIO_NR(4, 28)

/* Declare early_console_setup() prototype */
void __init early_console_setup(unsigned long base, struct clk *clk);

/*
 * JCI specific timer initialization stuff.
 */
#ifdef CONFIG_LOCAL_TIMERS
extern void __iomem *twd_base;
#endif

//#define   __BRING_UP__

static void __init mx6q_timer_init(void) {
	struct clk *uart_clk;

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.1", NULL);
	early_console_setup(UART2_BASE_ADDR, uart_clk);
}

/* IMX6 GPU initial data structure */
static struct viv_gpu_platform_data imx6q_gpu_platform_data __initdata = {
	.reserved_mem_size = SZ_256M,
};

/*
 * JCI specific function to reserve GPU memory..
 */
static void __init mx6q_gpu_memory_reserve(void) {
	phys_addr_t phys;

	if (imx6q_gpu_platform_data.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_platform_data.reserved_mem_size, SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_platform_data.reserved_mem_size);
		imx6q_gpu_platform_data.reserved_mem_base = phys;
	}
}

/*
 * GPIO toggle helper function.
 */
void __init toggle_gpio4_28(int value) {
	gpio_set_value(SET_GPIO4_28, value);
	printk(KERN_INFO "SET_GPIO4_28 to %d\n", value);
}

enum {
	GPIO_EDGE_NONE = 0x00,
	GPIO_EDGE_FALLING = (0x01 << 0),
	GPIO_EDGE_RISING = (0x01 << 1),
	GPIO_EDGE_BOTH = (GPIO_EDGE_FALLING | GPIO_EDGE_RISING)
};

struct gpio_export {
	uint16_t id;
	const char *name;
	const char *altName;
	bool active_low;
	bool initial_level;
	bool dir_in;
	uint8_t edge_irq;
	bool redirectable;
};

/*
 * The temperature sensor module implements a temperature sensor/conversion function based on a temperature-dependent voltage to time conversion.
 */
static const struct anatop_thermal_platform_data mx6q_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

/*
 * JCI specific UART1 - UART5 initialization.
 */
 #if 0
static const struct imxuart_platform_data mx6q_uart4_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS /* | IMXUART_SDMA */,
	.dma_req_rx = MX6Q_DMA_REQ_UART4_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART4_TX,
	.ctstl = 24,
	.rxtl = 16,
};
#endif

static void __init mx6q_init_uart(void) {
	imx6q_add_imx_uart(0, NULL);                /* Initialize UART1 - GMSL LVDS serializer port, connected to MAX9249 GSML serializer. */
	imx6q_add_imx_uart(1, NULL);                /* Initialize UART2 - RTOS Debug port. */
	imx6q_add_imx_uart(2, NULL);                /* Initialize UART3 - GPS NMEA port to communicates GPS coordinates */
	//imx6q_add_imx_uart(3, &mx6q_uart4_data);    /* Initialize UART4 - Bluetooth Host Controller interface (HCI) */
	imx6q_add_imx_uart(3, NULL);    /* Initialize UART4 - Bluetooth Host Controller interface (HCI) */
	//imx6q_add_imx_uart(4, NULL);                /* Initialize UART5 - RS485 Serial link to the vehicle radio / amplifier. */
}

/*
 * regulator settings.
 */
extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern void mx6_cpu_regulator_init(void);

static struct mxc_dvfs_platform_data mx6q_dvfscore_data __initdata = {
	.reg_id = "cpu_vddgp",
	.soc_id = "cpu_vddsoc",
	.pu_id = "cpu_vddvpu",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

/*
 * wl12xx WLAN initialisation stuff.
 */
static struct wl12xx_platform_data wl12xx_wlan_data __initdata = {
	.irq = -1,
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_26,
};

static void wl12xx_wlan_set_power(bool power_on) {
	static bool power_state;

	if (power_on == power_state) {
		return;
	}
	power_state = power_on;

	pr_info("Powering %s wl12xx", power_on ? "on" : "off");

	if (power_on) {
		gpio_set_value(DO_WL_EN, 1);
	}
	else {
		gpio_set_value(DO_WL_EN, 0);
	}
}

static int __init mx6q_wl12xx_wlan_init(void) {
	int result = gpio_request_one(DO_WL_EN, GPIOF_OUT_INIT_LOW, "DO_WL_EN");
	if (result) {
		pr_err("Could not request wl12xx wlan enable gpio: %d\n", result);
		return result;
	}

	result = gpio_request_one(DI_WL_IRQ, GPIOF_IN, "DO_WL_IRQ");
	if (result) {
		pr_err("Could not request wl12xx wlan irq gpio: %d\n", result);
		gpio_free(DO_WL_EN);
		return result;
	}

	wl12xx_wlan_data.irq = gpio_to_irq(DI_WL_IRQ);

	result = wl12xx_set_platform_data(&wl12xx_wlan_data);
	if (result) {
		pr_err("Could not set wl12xx wlan data: %d\n", result);
		gpio_free(DI_WL_IRQ);
		gpio_free(DO_WL_EN);
	}

	return result;
}

/*
 *  bluetooth initialisation stuff.
 */
static int bluetooth_enable(int status) {
	if (status) {
		gpio_set_value(DO_BT_EN, 1);
	} else {
		gpio_set_value(DO_BT_EN, 0);
	}

	return 0;
}

static int __init mx6q_bluetooth_clock_init(void) {
	struct clk *new_parent;
	int rate = 0;
	struct clk *clko = clk_get(NULL, "clko_clk");

	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}

	new_parent = clk_get(NULL, "ckil");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	/* Set Clock to 32,768 KHz, required of BT/WLAN modul */
	rate = clk_round_rate(clko, 32768);

	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_bt_rfkill_platform_data mx6q_bluetooth_rfkill_data __initdata = {
	.power_change = bluetooth_enable,
};

static struct platform_device mx6q_bluetooth_rfkill __initdata = {
	.name = "mx6_bt_rfkill",
};

static int __init mx6q_bluetooth_init(void) {
	int result = gpio_request_one(DO_BT_EN, GPIOF_OUT_INIT_HIGH, "DO_BT_EN");
	/* Get and set slow clock of BT/WLAN module (CLK_WL_32.768KHZ */
	mx6q_bluetooth_clock_init();

	if (0 == result) {
		if (0 != (result = mxc_register_device(&mx6q_bluetooth_rfkill, &mx6q_bluetooth_rfkill_data))) {
			pr_err("Could not register bluetooth rfkill: %d\n", result);
			gpio_free(DO_BT_EN);
		}
	}
	else {
		pr_err("Could not request bluetooth enable gpio: %d\n", result);
	}

	return result;
}

/*
 * SX5 board specific I2C initialisation stuff.
 */
/* I2C1 initialization data, 1V8 standard mode with 100 kHz transfer rate. */
static const struct imxi2c_platform_data mx6q_i2c1_data __initconst = {
	.bitrate = 104000,
};

/* I2C2 initialization data, 3V3 standard mode with 100 kHz transfer rate. */
static const struct imxi2c_platform_data mx6q_i2c2_data __initconst = {
	.bitrate = 104000,
};

/* I2C3 initialization data, 3V3 low speed mode with 50 kHz transfer rate. */
static const struct imxi2c_platform_data mx6q_i2c3_data __initconst = {
	.bitrate = 50000,
	//.post_addr_delay = 100,
};

/*
 * SX5 board specific SPI initialisation stuff.
 */
#if 0
static int mx6q_spi1_chip_select[] __initdata = {
	MX6Q_ARM2_ECSPI1_SS0,
};
#endif
static int mx6q_spi2_chip_select[] __initdata = {
    MX6Q_ARM2_ECSPI2_SS0,
};

static int mx6q_spi4_chip_select[] __initdata = {
    //MX6Q_ARM2_ECSPI4_SS0,
    MX6Q_ARM2_ECSPI3_SS0,
};

#if 0
/* ECSPI1 Serial NOR SPI Port */
static const struct spi_imx_master mx6q_spi1_data __initconst = {
	.chipselect = mx6q_spi1_chip_select,
	.num_chipselect = ARRAY_SIZE(mx6q_spi1_chip_select),
};
#endif

static const struct spi_imx_master mx6q_spi2_data __initconst = {
	.chipselect = mx6q_spi2_chip_select,
	.num_chipselect = ARRAY_SIZE(mx6q_spi2_chip_select),
};

static const struct spi_imx_master mx6q_spi4_data __initconst = {
	.chipselect = mx6q_spi4_chip_select,
	.num_chipselect = ARRAY_SIZE(mx6q_spi4_chip_select),
};

/* SPI-NOR flash partition table, size 8 MByte */
/* Partition layout is based on document "Infotainment Head Unit Platform Flash Memory Use" */

#if 0
#ifndef  __BRING_UP__
static struct mtd_partition mx6q_nor_flash_partitions[] __initdata = {
	{
		.name = "uboot",
		.offset = 0,
		.size = 0x00040000, 			/* 256 KByte */
	}, {
		.name = "kernel",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL, 			/*  KByte */
	}, 
};


static struct flash_platform_data mx6q_spi_flash_data __initdata = {
	.name = "m25p80",
	.parts = mx6q_nor_flash_partitions,
	.nr_parts = ARRAY_SIZE(mx6q_nor_flash_partitions),
	.type = "s25fl064p",
};
#endif
#endif

/*
 * Raw NAND flash partition table, max. sizes of all partitions limited to 1GB.
 * Partition layout is based on document "Infotainment Head Unit Platform Flash Memory Use"
 */
/* Set to 1 for UBI IBC single MTD */
#if 0
#ifndef  __BRING_UP__
#ifdef CONFIG_MX6Q_UBI_SINGLE_MTD
static struct mtd_partition mx6q_gpmi_flash_partitions[] = {
	{
		.name = "linux1",
		.offset = 0,
		.size = 0x02400000,                     /* 36 MByte */
	}, {
		.name = "rootfs1",
		.offset = MTDPART_OFS_APPEND,
		.size = 0x0C800000,                     /* 200 MByte */
	}, {
		.name = "jci1",
		.offset = MTDPART_OFS_APPEND,
		.size = 0x15400000,                     /* 340 MByte */
	}, {
		.name = "data",
		.offset = MTDPART_OFS_APPEND,
		.size = 0x1C000000,                     /* 448 MByte */
	},
};
#else
#ifndef CONFIG_MX6Q_RELFS
static struct mtd_partition mx6q_gpmi_flash_partitions[] __initdata = {
	{
		.name = "linux1",
		.offset = 0,
		.size = 0x00800000, 			/* 8 MByte */
	}, {
		.name = "linux2",
		.offset = MTDPART_OFS_APPEND,
		.size = 0x00800000, 			/* 8 MByte */
	}, {
		.name = "fs",
		.offset = MTDPART_OFS_APPEND,
		.size = 0x3F000000, 			/* 1008 MByte */
	},
};
#else
static struct mtd_partition mx6q_gpmi_flash_partitions[] __initdata = {
};
#endif
#endif

static int gpmi_nand_platform_init(void) {
#if 0
	/* Initialize the GPMI_NAND clock with 198 MHz */
	struct clk *gpmi = clk_get(NULL, "enfc_clk");
	clk_set_rate(gpmi, clk_round_rate(gpmi, 198000000));
#endif
	return 0;
}

static const struct gpmi_nand_platform_data mx6q_gpmi_nand_platform_data __initconst = {
	.platform_init = gpmi_nand_platform_init,
	.min_prop_delay_in_ns = 5,
	.max_prop_delay_in_ns = 9,
	.max_chip_count = 1,
	.partitions = mx6q_gpmi_flash_partitions,
	.partition_count = ARRAY_SIZE(mx6q_gpmi_flash_partitions),
	.enable_bbt = 1,
};
#endif
#endif

/*
 * IPU1_DI0, display interface that feeds the LDB0 Gigabit Multimedia Serial Link (GMSL) serializer.
 * IPU1_DI1, display interface that feeds the LDB1 Flat Panel Display Link (FPD Link II) serializer.
 * IPU2_DI0, display interface that feeds the High Definition Media Interface (HDMI) output port.
 * IPU2_DI1, display interface that feeds the MIPI-DSI output port.
 * IPU2_CSI1 is the camera sensor interface that captures video from one of two input sources(Video Decoder, HDMI Receiver)
 */
/* TODO: Check settings */
static struct imx_ipuv3_platform_data mx6q_ipu_data[] __initdata = {
	{
		.rev = 4,
		.csi_clk[0] = "ccm_clk0",
	}, {
		.rev = 4,
		.csi_clk[0] = "ccm_clk0",
	},
};

static struct ipuv3_fb_platform_data mx6q_frame_buffer_data[] __initdata = {
	/* FB0 */
	{
		.disp_dev = "ldb",
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		.mode_str = "LDB-WVGA",
		.default_bpp = 32,
		.int_clk = false,
	},
};

#endif /* __BOARD_MX6Q_COMMON_H */
