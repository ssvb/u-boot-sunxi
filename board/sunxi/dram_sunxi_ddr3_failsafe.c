/* this file is generated, don't edit it yourself */

#include "common.h"
#include <asm/arch/dram.h>

static struct dram_para dram_para = { /* DRAM timings: 6-6-6-15 (396 MHz) */
	.clock = 312,
	.mbus_clock = 208,
	.type = 3,
	.rank_num = 1,
	.cas = 6,
	.zq = 0x7b,
	.odt_en = 0,
	.tpr0 = 0x2a8f6690,
	.tpr1 = 0xa0a0,
	.tpr2 = 0x22a00,
	.tpr3 = 0,
	.tpr4 = 0,
	.tpr5 = 0,
	.emr1 = 4,
	.emr2 = 0,
	.emr3 = 0,
	.active_windowing = 1,
};

unsigned long sunxi_dram_init(void)
{
	return dramc_init(&dram_para);
}
