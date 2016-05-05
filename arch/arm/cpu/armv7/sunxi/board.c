/*
 * (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Some init for sunxi platform.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <mmc.h>
#include <i2c.h>
#include <serial.h>
#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#endif
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/spl.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/timer.h>
#include <asm/arch/tzpc.h>
#include <asm/arch/mmc.h>

#include <linux/compiler.h>

#ifndef CONFIG_SPL_BUILD
/* The fel-sdboot.sunxi binary (which is not very useful) */
static char spi_flash_data[8192] = {
	0x06, 0x00, 0x00, 0xEA, 0x65, 0x47, 0x4F, 0x4E, 0x2E, 0x42, 0x54,
	0x30, 0x50, 0xFB, 0xEF, 0xDA, 0x00, 0x20, 0x00, 0x00, 0x53, 0x50,
	0x4C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0x40, 0x2D, 0xE9, 0x07, 0x35, 0xA0, 0xE3, 0x24, 0x20, 0x93, 0xE5,
	0x02, 0x29, 0x82, 0xE3, 0x24, 0x20, 0x83, 0xE5, 0x24, 0x20, 0x93,
	0xE5, 0x24, 0x10, 0x93, 0xE5, 0x02, 0x19, 0xC1, 0xE3, 0x24, 0x10,
	0x83, 0xE5, 0x22, 0x38, 0xA0, 0xE1, 0x39, 0x26, 0x01, 0xE3, 0x02,
	0x00, 0x53, 0xE1, 0x02, 0x00, 0x00, 0x0A, 0x50, 0x20, 0x82, 0xE2,
	0x02, 0x00, 0x53, 0xE1, 0x01, 0x00, 0x00, 0x1A, 0x20, 0x30, 0xA0,
	0xE3, 0x33, 0xFF, 0x2F, 0xE1, 0x04, 0x30, 0x9F, 0xE5, 0x33, 0xFF,
	0x2F, 0xE1, 0x08, 0x80, 0xBD, 0xE8, 0x20, 0x00, 0xFF, 0xFF, 0x00
};
#endif

struct fel_stash {
	uint32_t sp;
	uint32_t lr;
	uint32_t cpsr;
	uint32_t sctlr;
	uint32_t vbar;
	uint32_t cr;
};

struct fel_stash fel_stash __attribute__((section(".data")));

static int gpio_init(void)
{
#if CONFIG_CONS_INDEX == 1 && defined(CONFIG_UART0_PORT_F)
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I)
	/* disable GPB22,23 as uart0 tx,rx to avoid conflict */
	sunxi_gpio_set_cfgpin(SUNXI_GPB(22), SUNXI_GPIO_INPUT);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(23), SUNXI_GPIO_INPUT);
#endif
#if defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPF(2), SUN8I_GPF_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPF(4), SUN8I_GPF_UART0);
#else
	sunxi_gpio_set_cfgpin(SUNXI_GPF(2), SUNXI_GPF_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPF(4), SUNXI_GPF_UART0);
#endif
	sunxi_gpio_set_pull(SUNXI_GPF(4), 1);
#elif CONFIG_CONS_INDEX == 1 && (defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I))
	sunxi_gpio_set_cfgpin(SUNXI_GPB(22), SUN4I_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(23), SUN4I_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(23), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN5I)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(19), SUN5I_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(20), SUN5I_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(20), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN6I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(20), SUN6I_GPH_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(21), SUN6I_GPH_UART0);
	sunxi_gpio_set_pull(SUNXI_GPH(21), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN8I_A33)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(0), SUN8I_A33_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(1), SUN8I_A33_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(1), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN8I_H3)
	sunxi_gpio_set_cfgpin(SUNXI_GPA(4), SUN8I_H3_GPA_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPA(5), SUN8I_H3_GPA_UART0);
	sunxi_gpio_set_pull(SUNXI_GPA(5), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN8I_A83T)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(9), SUN8I_A83T_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(10), SUN8I_A83T_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(10), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN9I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(12), SUN9I_GPH_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(13), SUN9I_GPH_UART0);
	sunxi_gpio_set_pull(SUNXI_GPH(13), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 2 && defined(CONFIG_MACH_SUN5I)
	sunxi_gpio_set_cfgpin(SUNXI_GPG(3), SUN5I_GPG_UART1);
	sunxi_gpio_set_cfgpin(SUNXI_GPG(4), SUN5I_GPG_UART1);
	sunxi_gpio_set_pull(SUNXI_GPG(4), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 3 && defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(0), SUN8I_GPB_UART2);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(1), SUN8I_GPB_UART2);
	sunxi_gpio_set_pull(SUNXI_GPB(1), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 5 && defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPL(2), SUN8I_GPL_R_UART);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(3), SUN8I_GPL_R_UART);
	sunxi_gpio_set_pull(SUNXI_GPL(3), SUNXI_GPIO_PULL_UP);
#else
#error Unsupported console port number. Please fix pin mux settings in board.c
#endif

	return 0;
}

int spl_board_load_image(void)
{
	debug("Returning to FEL sp=%x, lr=%x\n", fel_stash.sp, fel_stash.lr);
	return_to_fel(fel_stash.sp, fel_stash.lr);

	return 0;
}

void s_init(void)
{
#if defined CONFIG_MACH_SUN6I || defined CONFIG_MACH_SUN8I_A23
	/* Magic (undocmented) value taken from boot0, without this DRAM
	 * access gets messed up (seems cache related) */
	setbits_le32(SUNXI_SRAMC_BASE + 0x44, 0x1800);
#endif
#if defined CONFIG_MACH_SUN6I || \
    defined CONFIG_MACH_SUN7I || \
    defined CONFIG_MACH_SUN8I
	/* Enable SMP mode for CPU0, by setting bit 6 of Auxiliary Ctl reg */
	asm volatile(
		"mrc p15, 0, r0, c1, c0, 1\n"
		"orr r0, r0, #1 << 6\n"
		"mcr p15, 0, r0, c1, c0, 1\n");
#endif
#if defined CONFIG_MACH_SUN6I || defined CONFIG_MACH_SUN8I_H3
	/* Enable non-secure access to some peripherals */
	tzpc_init();
#endif

	clock_init();
	timer_init();
	gpio_init();
	i2c_init_board();
}

#ifdef CONFIG_SPL_BUILD
DECLARE_GLOBAL_DATA_PTR;

/* The sunxi internal brom will try to loader external bootloader
 * from mmc0, nand flash, mmc2.
 */
u32 spl_boot_device(void)
{
	__maybe_unused struct mmc *mmc0, *mmc1;
	/*
	 * When booting from the SD card or NAND memory, the "eGON.BT0"
	 * signature is expected to be found in memory at the address 0x0004
	 * (see the "mksunxiboot" tool, which generates this header).
	 *
	 * When booting in the FEL mode over USB, this signature is patched in
	 * memory and replaced with something else by the 'fel' tool. This other
	 * signature is selected in such a way, that it can't be present in a
	 * valid bootable SD card image (because the BROM would refuse to
	 * execute the SPL in this case).
	 *
	 * This checks for the signature and if it is not found returns to
	 * the FEL code in the BROM to wait and receive the main u-boot
	 * binary over USB. If it is found, it determines where SPL was
	 * read from.
	 */
	if (!is_boot0_magic(SPL_ADDR + 4)) /* eGON.BT0 */
		return BOOT_DEVICE_BOARD;

	/* The BROM will try to boot from mmc0 first, so try that first. */
#ifdef CONFIG_MMC
	mmc_initialize(gd->bd);
	mmc0 = find_mmc_device(0);
	if (sunxi_mmc_has_egon_boot_signature(mmc0))
		return BOOT_DEVICE_MMC1;
#endif

	/* Fallback to booting NAND if enabled. */
	if (IS_ENABLED(CONFIG_SPL_NAND_SUPPORT))
		return BOOT_DEVICE_NAND;

#ifdef CONFIG_MMC
	if (CONFIG_MMC_SUNXI_SLOT_EXTRA == 2) {
		mmc1 = find_mmc_device(1);
		if (sunxi_mmc_has_egon_boot_signature(mmc1))
			return BOOT_DEVICE_MMC2;
	}
#endif

	panic("Could not determine boot source\n");
	return -1;		/* Never reached */
}

/* No confirmation data available in SPL yet. Hardcode bootmode */
u32 spl_boot_mode(void)
{
	return MMCSD_MODE_RAW;
}

void board_init_f(ulong dummy)
{
	spl_init();
	preloader_console_init();

#ifdef CONFIG_SPL_I2C_SUPPORT
	/* Needed early by sunxi_board_init if PMU is enabled */
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif
	sunxi_board_init();
}
#endif

void reset_cpu(ulong addr)
{
#ifdef CONFIG_SUNXI_GEN_SUN4I
	static const struct sunxi_wdog *wdog =
		 &((struct sunxi_timer_reg *)SUNXI_TIMER_BASE)->wdog;

	/* Set the watchdog for its shortest interval (.5s) and wait */
	writel(WDT_MODE_RESET_EN | WDT_MODE_EN, &wdog->mode);
	writel(WDT_CTRL_KEY | WDT_CTRL_RESTART, &wdog->ctl);

	while (1) {
		/* sun5i sometimes gets stuck without this */
		writel(WDT_MODE_RESET_EN | WDT_MODE_EN, &wdog->mode);
	}
#endif
#ifdef CONFIG_SUNXI_GEN_SUN6I
	static const struct sunxi_wdog *wdog =
		 ((struct sunxi_timer_reg *)SUNXI_TIMER_BASE)->wdog;

	/* Set the watchdog for its shortest interval (.5s) and wait */
	writel(WDT_CFG_RESET, &wdog->cfg);
	writel(WDT_MODE_EN, &wdog->mode);
	writel(WDT_CTRL_KEY | WDT_CTRL_RESTART, &wdog->ctl);
	while (1) { }
#endif
}

#ifndef CONFIG_SPL_BUILD

#define SUN4I_CTL_ENABLE                        BIT(0)
#define SUN4I_CTL_MASTER                        BIT(1)
#define SUN4I_CTL_CPHA                          BIT(2)
#define SUN4I_CTL_CPOL                          BIT(3)
#define SUN4I_CTL_CS_ACTIVE_LOW                 BIT(4)
#define SUN4I_CTL_LMTF                          BIT(6)
#define SUN4I_CTL_TF_RST                        BIT(8)
#define SUN4I_CTL_RF_RST                        BIT(9)
#define SUN4I_CTL_XCH                           BIT(10)
#define SUN4I_CTL_CS_MASK                       0x3000
#define SUN4I_CTL_CS(cs)                        (((cs) << 12) & SUN4I_CTL_CS_MASK)
#define SUN4I_CTL_DHB                           BIT(15)
#define SUN4I_CTL_CS_MANUAL                     BIT(16)
#define SUN4I_CTL_CS_LEVEL                      BIT(17)
#define SUN4I_CTL_TP                            BIT(18)

#define CCM_SPI2_CLK                            (0x01C20000 + 0xA8)
#define CCM_AHB_GATING0                         (0x01C20000 + 0x60)

#define SPI2_CCTL                               (0x01C17000 + 0x1C)
#define SPI2_CTL                                (0x01C17000 + 0x08)
#define SPI2_RX                                 (0x01C17000 + 0x00)
#define SPI2_TX                                 (0x01C17000 + 0x04)
#define SPI2_FIFO_STA                           (0x01C17000 + 0x28)
#define SPI2_TC                                 (0x01C17000 + 0x24)
#define SPI2_DMACTL                             (0x01C17000 + 0x14)
#define SPI2_BC                                 (0x01C17000 + 0x20)
#define SPI2_XMIT_CNT                           (0x01C17000 + 0x24)

/*
 * Configure the SPI2 pin muxing and setup clocks.
 */
static void setup_spi2(void)
{
	int reg_val, best_pll6_divisor;
	unsigned pll6_hz = clock_get_pll6();

	sunxi_gpio_set_cfgpin(SUNXI_GPE(0), SUN5I_GPE_SPI2);
	sunxi_gpio_set_cfgpin(SUNXI_GPE(1), SUN5I_GPE_SPI2);
	sunxi_gpio_set_cfgpin(SUNXI_GPE(2), SUN5I_GPE_SPI2);
	sunxi_gpio_set_cfgpin(SUNXI_GPE(3), SUN5I_GPE_SPI2);
	sunxi_gpio_set_pull(SUNXI_GPE(0), SUNXI_GPIO_PULL_UP);

	best_pll6_divisor = DIV_ROUND_UP(pll6_hz, 150000000);
	if (best_pll6_divisor > 16) {
		printf("Error: invalid pll6 divisor\n");
		while (1) {}
	}

	reg_val = readl(CCM_AHB_GATING0);
	reg_val |= (1 << 22);
	writel(reg_val, CCM_AHB_GATING0); /* CCM_AHB_GATE_SPI2; */

	reg_val = readl(CCM_SPI2_CLK);
	reg_val &= ~(3 << 24);
	reg_val |= 1 << 24;
	reg_val |= (1 << 31) | (best_pll6_divisor - 1);
	writel(reg_val, CCM_SPI2_CLK);    /* PLL6/best_divisor (<=150MHz) */

	reg_val = (1 << 12) | 1;
	writel(reg_val, SPI2_CCTL);       /* AHB/4 */

	/* Enable SPI2 as SPI slave */
	reg_val = readl(SPI2_CTL);
	reg_val &= ~SUN4I_CTL_MASTER;
	reg_val |= SUN4I_CTL_ENABLE | SUN4I_CTL_TF_RST | SUN4I_CTL_RF_RST;
	writel(reg_val, SPI2_CTL);
}

/*
 * This is a hackish implementation, which only supports the READ DATA BYTES
 * command, tries to predict the desired output and has some sanity checks
 * to verify if the assumptions were correct. We can't really implement a
 * correct handling of the protocol because the timing constraints are too
 * tight.
 *
 * The Allwinner's BROM first requests 256 bytes starting from the address 0,
 * verifies correctness of the header and then rewinds back to the address 0
 * and starts requesting 2048 byte blocks one after another. We just produce
 * this particular output and the BROM seems to be happy.
 *
 * TD;TR; use SPI2 on Allwinner A13 to do a limited SPI NOR flash emulation.
 */
static void emulate_spi_flash(void)
{
	unsigned long before, after;
	int txdatacount = 0, rxcount = 0;
	int offs = 0;

	setup_spi2();

	printf("\nEmulating SPI NOR flash on SPI2...\n");

	/* The dummy padding data, to be exchanged with the first command */
	writeb(0, SPI2_TX);
	writeb(0, SPI2_TX);
	writeb(0, SPI2_TX);
	writeb(0, SPI2_TX);

	/* Fill the TX buffer */
	for (offs = 0; offs < 48; offs++) {
		writeb(spi_flash_data[offs], SPI2_TX);
		txdatacount++;
	}

	/* Wait until we receive something */
	while ((readl(SPI2_FIFO_STA) & 0x7F) == 0) {}

	before = timer_get_us();

	while (1) {
		int rxfifo = readl(SPI2_FIFO_STA) & 0x7F;
		int txfifo = (readl(SPI2_FIFO_STA) >> 16) & 0x7F;

		if (txfifo < 32) {
			if (txdatacount == 256) {
				writeb(0, SPI2_TX);
				writeb(0, SPI2_TX);
				writeb(0, SPI2_TX);
				writeb(0, SPI2_TX);
				offs = 0;
			} else if ((txdatacount - 256) % 2048 == 0) {
				writeb(0, SPI2_TX);
				writeb(0, SPI2_TX);
				writeb(0, SPI2_TX);
				writeb(0, SPI2_TX);
			}
			writeb(spi_flash_data[offs++], SPI2_TX);
			writeb(spi_flash_data[offs++], SPI2_TX);
			writeb(spi_flash_data[offs++], SPI2_TX);
			writeb(spi_flash_data[offs++], SPI2_TX);
			txdatacount += 4;
		}

		if (txfifo < 16) {
			printf("Error: txfifo underflow\n");
			while (1) {};
		}

		if (rxfifo >= 4) {
			int b1 = readb(SPI2_RX);
			int b2 = readb(SPI2_RX);
			int b3 = readb(SPI2_RX);
			int b4 = readb(SPI2_RX);
			int addr = (b2 << 16) | (b3 << 8) | b4;

			if ((rxcount - 260) % 2052 == 0) {
				if (b1 != 3) {
					printf("Error: unexpected command %d (wanted READ DATA BYTES)\n", b1);
					while (1) {};
				}
				if (((rxcount - 260) / 2052) * 2048 != addr) {
					printf("Error: unexpected address %d for READ DATA BYTES\n", addr);
					while (1) {};
				}
			}
			rxcount += 4;
		}

		if (rxfifo > 48) {
			printf("Error: rxfifo overflow\n");
			while (1) {};
		}

		if (offs >= sizeof(spi_flash_data)) {
			while ((txfifo = (readl(SPI2_FIFO_STA) >> 16) & 0x7F)) {}
			break;
		}
	}

	after = timer_get_us();
	printf("All %d bytes of the emulated NOR flash data have been sent.\n",
	       (int)sizeof(spi_flash_data));
	if (after != before)
		printf("The average transfer speed: %d KB/s.\n",
		       (int)(1000 * txdatacount / (after - before)));
}

#endif

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();

#ifndef CONFIG_SPL_BUILD
	/*
	 * We have just enabled the D-cache and have everything ready,
	 * this is a convinient place to inject the SPI demo
	 */
	while (1) {
		emulate_spi_flash();
	}
#endif
}
#endif

#ifdef CONFIG_CMD_NET
/*
 * Initializes on-chip ethernet controllers.
 * to override, implement board_eth_init()
 */
int cpu_eth_init(bd_t *bis)
{
	__maybe_unused int rc;

#ifdef CONFIG_MACPWR
	gpio_request(CONFIG_MACPWR, "macpwr");
	gpio_direction_output(CONFIG_MACPWR, 1);
	mdelay(200);
#endif

#ifdef CONFIG_SUNXI_GMAC
	rc = sunxi_gmac_initialize(bis);
	if (rc < 0) {
		printf("sunxi: failed to initialize gmac\n");
		return rc;
	}
#endif

	return 0;
}
#endif
