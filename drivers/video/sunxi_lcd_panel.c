/*
 * LCD panel driver for Allwinner SoCs.
 *
 * (C) Copyright 2015 Hans de Goede <hdegoede@redhat.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include "sunxi_lcd_panel.h"
#include "ssd2828.h"

#ifdef CONFIG_VIDEO_LCD_PANEL_HITACHI_TX18D42VM

#define SPI_CS		SUNXI_GPA(0)
#define SPI_CLK		SUNXI_GPA(1)
#define SPI_MOSI	SUNXI_GPA(2)

/*
 * Very simple write only SPI support, this does not use the generic SPI infra
 * because that assumes R/W SPI, requiring a MISO pin. Also the necessary glue
 * code alone would be larger then this minimal version.
 */

static void sunxi_lcd_panel_spi_write(unsigned int data, int bits)
{
	int i, offset;

	gpio_direction_output(SPI_CS, 0);
	for (i = 0; i < bits; i++) {
		gpio_direction_output(SPI_CLK, 0);
		offset = (bits - 1) - i;
		gpio_direction_output(SPI_MOSI, (data >> offset) & 1);
		udelay(2);
		gpio_direction_output(SPI_CLK, 1);
		udelay(2);
	}
	gpio_direction_output(SPI_CS, 1);
	udelay(2);
}

void sunxi_lcd_panel_hitachi_tx18d42vm_init(void)
{
	const u16 init_data[] = {
		0x0029,		/* reset */
		0x0025,		/* standby */
		0x0840,		/* enable normally black */
		0x0430,		/* enable FRC/dither */
		0x385f,		/* enter test mode(1) */
		0x3ca4,		/* enter test mode(2) */
		0x3409,		/* enable SDRRS, enlarge OE width */
		0x4041,		/* adopt 2 line / 1 dot */
	};
	int i;

	mdelay(50); /* Wait for lcd controller power on */

	for (i = 0; i < ARRAY_SIZE(init_data); i++)
		sunxi_lcd_panel_spi_write(init_data[i], 16);

	mdelay(50); /* All the tx18d42vm drivers have a delay here ? */

	sunxi_lcd_panel_spi_write(0x00ad, 16); /* display on */
}

#endif

#ifdef CONFIG_VIDEO_LCD_SSD2828

int sunxi_ssd2828_init(const struct ctfb_res_modes *mode)
{
	struct ssd2828_config cfg = {
		.csx_pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_SPI_CS),
		.sck_pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_SPI_SCLK),
		.sdi_pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_SPI_MOSI),
		.sdo_pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_SPI_MISO),
		.reset_pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_SSD2828_RESET),
		.ssd2828_tx_clk_khz                      = 27000,
		.ssd2828_color_depth                     = 24,
#ifdef CONFIG_VIDEO_LCD_PANEL_MIPI_4_LANE_513_MBPS_VIA_SSD2828
		.mipi_dsi_number_of_data_lanes           = 4,
		.mipi_dsi_bitrate_per_data_lane_mbps     = 513,
		.mipi_dsi_delay_after_exit_sleep_mode_ms = 100,
		.mipi_dsi_delay_after_set_display_on_ms  = 200
#else
#error MIPI LCD panel needs configuration parameters
#endif
	};

	if (cfg.csx_pin == -1 || cfg.sck_pin == -1 || cfg.sdi_pin == -1) {
		printf("SSD2828: SPI pins are not properly configured\n");
		return 1;
	}
	if (cfg.reset_pin == -1) {
		printf("SSD2828: Reset pin is not properly configured\n");
		return 1;
	}

	return ssd2828_init(&cfg, mode);
}

#endif
