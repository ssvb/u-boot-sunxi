/*
 * LCD panel driver for Allwinner SoCs.
 *
 * (C) Copyright 2015 Hans de Goede <hdegoede@redhat.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

struct ctfb_res_modes;

void sunxi_lcd_panel_hitachi_tx18d42vm_init(void);
int sunxi_ssd2828_init(const struct ctfb_res_modes *mode);
