/*
 * Entry point of the FEL mode SPL.
 *
 * Copyright (c) 2015 Siarhei Siamashka <siarhei.siamashka@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm-offsets.h>
#include <config.h>
#include <linux/linkage.h>

ENTRY(_start_fel)
	ldr	r9, =gdata
	b	s_init
ENDPROC(_start_fel)
