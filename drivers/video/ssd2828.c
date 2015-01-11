/*
 * (C) 2015 Siarhei Siamashka <siarhei.siamashka@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * Support for the SSD2828 bridge chip, which can take pixel data coming
 * from a parallel LCD interface and translate it on the flight into MIPI DSI
 * interface for driving a MIPI compatible TFT display.
 */

#include <common.h>
#include <mipi_display.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>

#include "videomodes.h"
#include "ssd2828.h"

#define		SSD2828_DIR	0xB0
#define		SSD2828_VICR1	0xB1
#define		SSD2828_VICR2	0xB2
#define		SSD2828_VICR3	0xB3
#define		SSD2828_VICR4	0xB4
#define		SSD2828_VICR5	0xB5
#define		SSD2828_VICR6	0xB6
#define		SSD2828_CFGR	0xB7
#define		SSD2828_VCR	0xB8
#define		SSD2828_PCR	0xB9
#define		SSD2828_PLCR	0xBA
#define		SSD2828_CCR	0xBB
#define		SSD2828_PSCR1	0xBC
#define		SSD2828_PSCR2	0xBD
#define		SSD2828_PSCR3	0xBE
#define		SSD2828_PDR	0xBF
#define		SSD2828_OCR	0xC0
#define		SSD2828_MRSR	0xC1
#define		SSD2828_RDCR	0xC2
#define		SSD2828_ARSR	0xC3
#define		SSD2828_LCR	0xC4
#define		SSD2828_ICR	0xC5
#define		SSD2828_ISR	0xC6
#define		SSD2828_ESR	0xC7
#define		SSD2828_DAR1	0xC9
#define		SSD2828_DAR2	0xCA
#define		SSD2828_DAR3	0xCB
#define		SSD2828_DAR4	0xCC
#define		SSD2828_DAR5	0xCD
#define		SSD2828_DAR6	0xCE
#define		SSD2828_HTTR1	0xCF
#define		SSD2828_HTTR2	0xD0
#define		SSD2828_LRTR1	0xD1
#define		SSD2828_LRTR2	0xD2
#define		SSD2828_TSR	0xD3
#define		SSD2828_LRR	0xD4
#define		SSD2828_PLLR	0xD5
#define		SSD2828_TR	0xD6
#define		SSD2828_TECR	0xD7
#define		SSD2828_ACR1	0xD8
#define		SSD2828_ACR2	0xD9
#define		SSD2828_ACR3	0xDA
#define		SSD2828_ACR4	0xDB
#define		SSD2828_IOCR	0xDC
#define		SSD2828_VICR7	0xDD
#define		SSD2828_LCFR	0xDE
#define		SSD2828_DAR7	0xDF
#define		SSD2828_PUCR1	0xE0
#define		SSD2828_PUCR2	0xE1
#define		SSD2828_PUCR3	0xE2
#define		SSD2828_CBCR1	0xE9
#define		SSD2828_CBCR2	0xEA
#define		SSD2828_CBSR	0xEB
#define		SSD2828_ECR	0xEC
#define		SSD2828_VSDR	0xED
#define		SSD2828_TMR	0xEE
#define		SSD2828_GPIO1	0xEF
#define		SSD2828_GPIO2	0xF0
#define		SSD2828_DLYA01	0xF1
#define		SSD2828_DLYA23	0xF2
#define		SSD2828_DLYB01	0xF3
#define		SSD2828_DLYB23	0xF4
#define		SSD2828_DLYC01	0xF5
#define		SSD2828_DLYC23	0xF6
#define		SSD2828_ACR5	0xF7
#define		SSD2828_RR	0xFF

#define	SSD2828_CFGR_HS					(1 << 0)
#define	SSD2828_CFGR_CKE				(1 << 1)
#define	SSD2828_CFGR_SLP				(1 << 2)
#define	SSD2828_CFGR_VEN				(1 << 3)
#define	SSD2828_CFGR_HCLK				(1 << 4)
#define	SSD2828_CFGR_CSS				(1 << 5)
#define	SSD2828_CFGR_DCS				(1 << 6)
#define	SSD2828_CFGR_REN				(1 << 7)
#define	SSD2828_CFGR_ECD				(1 << 8)
#define	SSD2828_CFGR_EOT				(1 << 9)
#define	SSD2828_CFGR_LPE				(1 << 10)
#define	SSD2828_CFGR_TXD				(1 << 11)

#define	SSD2828_ISR_READ_DATA_READY			(1 << 0)
#define	SSD2828_ISR_BTA_RESPONSE			(1 << 2)

#define	SSD2828_VIDEO_MODE_NON_BURST_WITH_SYNC_PULSES	(0 << 2)
#define	SSD2828_VIDEO_MODE_NON_BURST_WITH_SYNC_EVENTS	(1 << 2)
#define	SSD2828_VIDEO_MODE_BURST			(2 << 2)

#define	SSD2828_VIDEO_PIXEL_FORMAT_16BPP		0
#define	SSD2828_VIDEO_PIXEL_FORMAT_18BPP_PACKED		1
#define	SSD2828_VIDEO_PIXEL_FORMAT_18BPP_LOOSELY_PACKED	2
#define	SSD2828_VIDEO_PIXEL_FORMAT_24BPP		3

#define	SSD2828_LP_CLOCK_DIVIDER(n)			(((n) - 1) & 0x3F)

/*
 * SPI transfer, using the "24-bit 3 wire" mode (that's how it is called in
 * the SSD2828 documentation). The 'dout' input parameter specifies 24-bits
 * of data to be written to SSD2828. Returns the lowest 16-bits of data,
 * that is received back.
 */
static u32 soft_spi_xfer_24bit_3wire(const struct ssd2828_config *drv, u32 dout)
{
	int j, bitlen = 24;
	u32 tmpdin = 0;
	/*
	 * According to the "24 Bit 3 Wire SPI Interface Timing Characteristics"
	 * and "TX_CLK Timing Characteristics" tables in the SSD2828 datasheet,
	 * the lowest possible 'tx_clk' clock frequency is 8MHz, and SPI runs
	 * at 1/8 of that after reset. So using 1 microsecond delays is safe in
	 * the main loop. But the delays around chip select pin manipulations
	 * need to be longer (up to 16 'tx_clk' cycles, or 2 microseconds in
	 * the worst case).
	 */
	const int spi_delay_us = 1;
	const int spi_cs_delay_us = 2;

	gpio_set_value(drv->csx_pin, 0);
	udelay(spi_cs_delay_us);
	for (j = bitlen - 1; j >= 0; j--) {
		gpio_set_value(drv->sck_pin, 0);
		gpio_set_value(drv->sdi_pin, (dout & (1 << j)) != 0);
		udelay(spi_delay_us);
		if (drv->sdo_pin != -1)
			tmpdin = (tmpdin << 1) | gpio_get_value(drv->sdo_pin);
		gpio_set_value(drv->sck_pin, 1);
		udelay(spi_delay_us);
	}
	udelay(spi_cs_delay_us);
	gpio_set_value(drv->csx_pin, 1);
	udelay(spi_cs_delay_us);
	return tmpdin & 0xFFFF;
}

/*
 * Read from a SSD2828 hardware register (regnum >= 0xB0)
 */
static u32 read_hw_register(const struct ssd2828_config *cfg, u8 regnum)
{
	soft_spi_xfer_24bit_3wire(cfg, 0x700000 | regnum);
	return soft_spi_xfer_24bit_3wire(cfg, 0x730000);
}

/*
 * Write to a SSD2828 hardware register (regnum >= 0xB0)
 */
static void write_hw_register(const struct ssd2828_config *cfg, u8 regnum,
			      u16 val)
{
	soft_spi_xfer_24bit_3wire(cfg, 0x700000 | regnum);
	soft_spi_xfer_24bit_3wire(cfg, 0x720000 | val);
}

static int await_completion(const struct ssd2828_config *cfg,
			    u8 reg, u32 mask, u32 val)
{
	unsigned long tmo = timer_get_us() + 100000;

	while ((read_hw_register(cfg, reg) & mask) != val) {
		if (timer_get_us() > tmo)
			return 1;
	}
	return 0;
}

static int await_bits_set(const struct ssd2828_config *cfg,
			  u8 reg, u32 val)
{
	return await_completion(cfg, reg, val, val);
}

static int await_bits_clear(const struct ssd2828_config *cfg,
			    u8 reg, u32 val)
{
	return await_completion(cfg, reg, val, 0);
}

/*
 * Send MIPI command to the LCD panel (cmdnum < 0xB0)
 */
static void send_mipi_dcs_command(const struct ssd2828_config *cfg, u8 cmdnum)
{
	/* Set packet size to 1 (a single command with no parameters) */
	write_hw_register(cfg, SSD2828_PSCR1, 1);
	/* Send the command */
	write_hw_register(cfg, SSD2828_PDR, cmdnum);
}

/*
 * Sends a MIPI DCS command to retrieve information from the LCD panel.
 */
static int send_mipi_dcs_read_command(const struct ssd2828_config *cfg,
				      u8                           cmdnum,
				      u8                          *result_buf,
				      int                          bufsize)
{
	int size, i, result = 0;
	/* Save CFGR register */
	u32 old_cfgr = read_hw_register(cfg, SSD2828_CFGR);

	/* Set the read enable bit */
	write_hw_register(cfg, SSD2828_CFGR, old_cfgr | SSD2828_CFGR_REN);
	/* Clear buffers and bring the state machine to its initial state */
	write_hw_register(cfg, SSD2828_OCR, 1);
	if (await_bits_clear(cfg, SSD2828_OCR, 1) != 0)
		goto err;
	/* Clear the RW1C bits in the ISR register */
	write_hw_register(cfg, SSD2828_ISR, read_hw_register(cfg, SSD2828_ISR));

	/* Set the payload size (only the DCS command itself) */
	write_hw_register(cfg, SSD2828_PSCR1, 1);
	/* Set maximum return size */
	write_hw_register(cfg, SSD2828_MRSR, bufsize);
	/* Write the command */
	write_hw_register(cfg, SSD2828_PDR, cmdnum);

	/* Wait for the response */
	if (await_bits_set(cfg, SSD2828_ISR, SSD2828_ISR_BTA_RESPONSE) != 0)
		goto err; /* Timeout */

	/* Check if there is anything in the read buffer */
	if (!(read_hw_register(cfg, SSD2828_ISR) & SSD2828_ISR_READ_DATA_READY))
		goto err; /* There is no data to read */

	/* The size of the response packet */
	size = read_hw_register(cfg, SSD2828_RDCR);
	if (size > bufsize) {
		debug("SSD2828: Suspicious packet size.\n");
		size = bufsize;
	}
	result = size;

	/*
	 * This is somewhat special and does not seem to be well documented:
	 * 1) In order to properly read the data, all the buffer must be read
	 *    in one go without any interruptions. For example, using individual
	 *    reads done by the "read_hw_register" function only result in
	 *    repeatedly getting all the same first 16-bits of data without
	 *    advancing.
	 * 2) In order to empty the buffer, it is necessary to read as much
	 *    data as was initially configured in the MRSR register. The real
	 *    size of the packet in the RDCR register just indicates how much
	 *    of it is actually valid.
	 */
	soft_spi_xfer_24bit_3wire(cfg, 0x700000 | SSD2828_RR);
	for (i = 0; i < (bufsize + 1) / 2; i++) {
		u16 data = soft_spi_xfer_24bit_3wire(cfg, 0x730000);
		if (size-- > 0)
			*result_buf++ = data & 0xFF;
		if (size-- > 0)
			*result_buf++ = (data >> 8) & 0xFF;
	}

	if (read_hw_register(cfg, SSD2828_ISR) & SSD2828_ISR_READ_DATA_READY)
		debug("SSD2828: There is still some bogus unread data.\n");
err:
	/* Restore CFGR register */
	write_hw_register(cfg, SSD2828_CFGR, old_cfgr);
	return result;
}

/*
 * Reset SSD2828
 */
static void ssd2828_reset(const struct ssd2828_config *cfg)
{
	/* RESET needs 10 milliseconds according to the datasheet */
	gpio_set_value(cfg->reset_pin, 0);
	if (cfg->lcd_reset_pin != -1)
		gpio_set_value(cfg->lcd_reset_pin, 0);
	mdelay(10);
	gpio_set_value(cfg->reset_pin, 1);
	if (cfg->lcd_reset_pin != -1)
		gpio_set_value(cfg->lcd_reset_pin, 1);
	mdelay(10);
}

static int ssd2828_enable_gpio(const struct ssd2828_config *cfg)
{
	if (gpio_request(cfg->csx_pin, "ssd2828_csx")) {
		printf("SSD2828: request for 'ssd2828_csx' pin failed\n");
		return 1;
	}
	if (gpio_request(cfg->sck_pin, "ssd2828_sck")) {
		gpio_free(cfg->csx_pin);
		printf("SSD2828: request for 'ssd2828_sck' pin failed\n");
		return 1;
	}
	if (gpio_request(cfg->sdi_pin, "ssd2828_sdi")) {
		gpio_free(cfg->csx_pin);
		gpio_free(cfg->sck_pin);
		printf("SSD2828: request for 'ssd2828_sdi' pin failed\n");
		return 1;
	}
	if (gpio_request(cfg->reset_pin, "ssd2828_reset")) {
		gpio_free(cfg->csx_pin);
		gpio_free(cfg->sck_pin);
		gpio_free(cfg->sdi_pin);
		printf("SSD2828: request for 'ssd2828_reset' pin failed\n");
		return 1;
	}
	if (cfg->sdo_pin != -1 && gpio_request(cfg->sdo_pin, "ssd2828_sdo")) {
		gpio_free(cfg->csx_pin);
		gpio_free(cfg->sck_pin);
		gpio_free(cfg->sdi_pin);
		gpio_free(cfg->reset_pin);
		printf("SSD2828: request for 'ssd2828_sdo' pin failed\n");
		return 1;
	}
	if (cfg->lcd_reset_pin != -1 && gpio_request(cfg->lcd_reset_pin,
						     "lcd_reset")) {
		gpio_free(cfg->csx_pin);
		gpio_free(cfg->sck_pin);
		gpio_free(cfg->sdi_pin);
		gpio_free(cfg->reset_pin);
		if (cfg->sdo_pin != -1)
			gpio_free(cfg->sdo_pin);
		printf("SSD2828: request for 'lcd_reset' pin failed\n");
		return 1;
	}
	gpio_direction_output(cfg->reset_pin, 0);
	if (cfg->lcd_reset_pin != -1)
		gpio_direction_output(cfg->lcd_reset_pin, 0);
	gpio_direction_output(cfg->csx_pin, 1);
	gpio_direction_output(cfg->sck_pin, 1);
	gpio_direction_output(cfg->sdi_pin, 1);
	if (cfg->sdo_pin != -1)
		gpio_direction_input(cfg->sdo_pin);

	return 0;
}

static int ssd2828_free_gpio(const struct ssd2828_config *cfg)
{
	gpio_free(cfg->csx_pin);
	gpio_free(cfg->sck_pin);
	gpio_free(cfg->sdi_pin);
	gpio_free(cfg->reset_pin);
	if (cfg->sdo_pin != -1)
		gpio_free(cfg->sdo_pin);
	if (cfg->lcd_reset_pin != -1)
		gpio_free(cfg->lcd_reset_pin);
	return 1;
}

/*
 * PLL configuration register settings.
 *
 * See the "PLL Configuration Register Description" in the SSD2828 datasheet.
 */
static u32 construct_pll_config(u32 desired_pll_freq_kbps,
				u32 reference_freq_khz)
{
	u32 div_factor = 1, mul_factor, fr = 0;
	u32 output_freq_kbps;

	/* The intermediate clock after division can't be less than 5MHz */
	while (reference_freq_khz / (div_factor + 1) >= 5000)
		div_factor++;
	if (div_factor > 31)
		div_factor = 31;

	mul_factor = DIV_ROUND_UP(desired_pll_freq_kbps * div_factor,
				  reference_freq_khz);

	output_freq_kbps = reference_freq_khz * mul_factor / div_factor;

	if (output_freq_kbps >= 501000)
		fr = 3;
	else if (output_freq_kbps >= 251000)
		fr = 2;
	else if (output_freq_kbps >= 126000)
		fr = 1;

	return (fr << 14) | (div_factor << 8) | mul_factor;
}

static u32 decode_pll_config(u32 pll_config, u32 reference_freq_khz)
{
	u32 mul_factor = pll_config & 0xFF;
	u32 div_factor = (pll_config >> 8) & 0x1F;
	if (mul_factor == 0)
		mul_factor = 1;
	if (div_factor == 0)
		div_factor = 1;
	return reference_freq_khz * mul_factor / div_factor;
}

static int ssd2828_configure_video_interface(const struct ssd2828_config *cfg,
					     const struct ctfb_res_modes *mode)
{
	u32 val;

	/* RGB Interface Control Register 1 */
	write_hw_register(cfg, SSD2828_VICR1, (mode->vsync_len << 8) |
					      (mode->hsync_len));

	/* RGB Interface Control Register 2 */
	u32 vbp = mode->vsync_len + mode->upper_margin;
	u32 hbp = mode->hsync_len + mode->left_margin;
	write_hw_register(cfg, SSD2828_VICR2, (vbp << 8) | hbp);

	/* RGB Interface Control Register 3 */
	write_hw_register(cfg, SSD2828_VICR3, (mode->lower_margin << 8) |
					      (mode->right_margin));

	/* RGB Interface Control Register 4 */
	write_hw_register(cfg, SSD2828_VICR4, mode->xres);

	/* RGB Interface Control Register 5 */
	write_hw_register(cfg, SSD2828_VICR5, mode->yres);

	/* RGB Interface Control Register 6 */
	val = SSD2828_VIDEO_MODE_BURST;
	switch (cfg->ssd2828_color_depth) {
	case 16:
		val |= SSD2828_VIDEO_PIXEL_FORMAT_16BPP;
		break;
	case 18:
		val |= cfg->mipi_dsi_loosely_packed_pixel_format ?
			SSD2828_VIDEO_PIXEL_FORMAT_18BPP_LOOSELY_PACKED :
			SSD2828_VIDEO_PIXEL_FORMAT_18BPP_PACKED;
		break;
	case 24:
		val |= SSD2828_VIDEO_PIXEL_FORMAT_24BPP;
		break;
	default:
		printf("SSD2828: unsupported color depth\n");
		return 1;
	}
	write_hw_register(cfg, SSD2828_VICR6, val);

	/* Lane Configuration Register */
	write_hw_register(cfg, SSD2828_LCFR,
			  cfg->mipi_dsi_number_of_data_lanes - 1);

	return 0;
}

static void ssd2828_report_dcs_read_result(const struct ssd2828_config *cfg,
					   u8 cmd)
{
	u8 buf[64];
	int size, i;
	size = send_mipi_dcs_read_command(cfg, cmd, buf, sizeof(buf));
	if (size > 0) {
		printf("DCS command 0x%02X returned %2d bytes:", cmd, size);
		for (i = 0; i < size; i++)
			printf(" %02X", buf[i]);
		printf("\n");
	}
}

/*
 * Try to use different DCS commands to identify the panel id and
 * report it in the log.
 */
static void ssd2828_report_panel_id(const struct ssd2828_config *cfg)
{
	printf("Trying standard MIPI DSI commands to identify LCD panel:\n");

	ssd2828_report_dcs_read_result(cfg, MIPI_DCS_GET_DISPLAY_ID);
	ssd2828_report_dcs_read_result(cfg, MIPI_DCS_READ_DDB_START);

	printf("Trying nonstandard MIPI DSI commands to identify LCD panel:\n");

	/* Used by some LG panels (for example, LH350WS1-SD02.pdf) */
	ssd2828_report_dcs_read_result(cfg, 0xB1);

	/* Used by many LCD panels (for example, DA8620.pdf) */
	ssd2828_report_dcs_read_result(cfg, 0xDA);
	ssd2828_report_dcs_read_result(cfg, 0xDB);
	ssd2828_report_dcs_read_result(cfg, 0xDC);
}

int ssd2828_init(const struct ssd2828_config *cfg,
		 const struct ctfb_res_modes *mode)
{
	u32 lp_div, pll_freq_kbps, reference_freq_khz, pll_config;
	/* The LP clock speed is limited by 10MHz */
	const u32 mipi_dsi_low_power_clk_khz = 10000;
	/*
	 * This is just the reset default value of CFGR register (0x301).
	 * Because we are not always able to read back from SPI, have
	 * it initialized here.
	 */
	u32 cfgr_reg = SSD2828_CFGR_EOT | /* EOT Packet Enable */
		       SSD2828_CFGR_ECD | /* Disable ECC and CRC */
		       SSD2828_CFGR_HS;   /* Data lanes are in HS mode */

	/* Initialize the pins */
	if (ssd2828_enable_gpio(cfg) != 0)
		return 1;

	/* Reset the chip */
	ssd2828_reset(cfg);

	/*
	 * If there is a pin to read data back from SPI, then we are lucky. Try
	 * to check if SPI is configured correctly and SSD2828 is actually able
	 * to talk back.
	 */
	if (cfg->sdo_pin != -1) {
		if (read_hw_register(cfg, SSD2828_DIR) != 0x2828 ||
		    read_hw_register(cfg, SSD2828_CFGR) != cfgr_reg) {
			printf("SSD2828: SPI communication failed.\n");
			ssd2828_free_gpio(cfg);
			return 1;
		}
	}

	/*
	 * Pick the reference clock for PLL. If we know the exact 'tx_clk'
	 * clock speed, then everything is good. If not, then we can fallback
	 * to 'pclk' (pixel clock from the parallel LCD interface). In the
	 * case of using this fallback, it is necessary to have parallel LCD
	 * already initialized and running at this point.
	 */
	reference_freq_khz = cfg->ssd2828_tx_clk_khz;
	if (reference_freq_khz  == 0) {
		reference_freq_khz = mode->pixclock_khz;
		/* Use 'pclk' as the reference clock for PLL */
		cfgr_reg |= SSD2828_CFGR_CSS;
	}

	/*
	 * Setup the parallel LCD timings in the appropriate registers.
	 */
	if (ssd2828_configure_video_interface(cfg, mode) != 0) {
		ssd2828_free_gpio(cfg);
		return 1;
	}

	/* Configuration Register */
	cfgr_reg &= ~SSD2828_CFGR_HS;  /* Data lanes are in LP mode */
	cfgr_reg |= SSD2828_CFGR_CKE;  /* Clock lane is in HS mode */
	cfgr_reg |= SSD2828_CFGR_DCS;  /* Only use DCS packets */
	write_hw_register(cfg, SSD2828_CFGR, cfgr_reg);

	/* PLL Configuration Register */
	pll_config = construct_pll_config(
				cfg->mipi_dsi_bitrate_per_data_lane_mbps * 1000,
				reference_freq_khz);
	write_hw_register(cfg, SSD2828_PLCR, pll_config);

	pll_freq_kbps = decode_pll_config(pll_config, reference_freq_khz);
	lp_div = DIV_ROUND_UP(pll_freq_kbps, mipi_dsi_low_power_clk_khz * 8);

	/* VC Control Register */
	write_hw_register(cfg, SSD2828_VCR, 0);

	/* Clock Control Register */
	write_hw_register(cfg, SSD2828_CCR, SSD2828_LP_CLOCK_DIVIDER(lp_div));

	/* PLL Control Register */
	write_hw_register(cfg, SSD2828_PCR, 1); /* Enable PLL */

	/* Wait for PLL lock */
	udelay(500);

	send_mipi_dcs_command(cfg, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(cfg->mipi_dsi_delay_after_exit_sleep_mode_ms);

	/* If it is possible to read back, then try to retrieve the panel id */
	if (cfg->sdo_pin != -1)
		ssd2828_report_panel_id(cfg);

	send_mipi_dcs_command(cfg, MIPI_DCS_SET_DISPLAY_ON);
	mdelay(cfg->mipi_dsi_delay_after_set_display_on_ms);

	cfgr_reg |= SSD2828_CFGR_HS;    /* Enable HS mode for data lanes */
	cfgr_reg |= SSD2828_CFGR_VEN;   /* Enable video pipeline */
	write_hw_register(cfg, SSD2828_CFGR, cfgr_reg);

	return 0;
}
