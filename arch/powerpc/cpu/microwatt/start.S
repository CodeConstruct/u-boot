/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 Paul Mackerras <paulus@ozlabs.org>,
 * IBM Corp.
 */

#include <config.h>
#include <ppc_asm.tmpl>
#include <ppc_defs.h>
#include <version.h>

#include <asm/cache.h>
#include <asm/u-boot.h>

	.text
	.long	0x27051956		/* U-Boot Magic Number */
	.globl	version_string
version_string:
	.ascii U_BOOT_VERSION_STRING, "\0"
	.balign	4

	.globl	_start
_start:
	/* Get TOC pointer into r2 */
	bcl	20,31,$+4
0:	mflr	r2
	addis	r2, r2, (.TOC. - 0b)@ha
	addi	r2, r2, (.TOC. - 0b)@l

	/* Set up stack pointer and make initial stack frame */
	lis	r1, 0x0e00
	li	r0, 0
	stdu	r0, -32(r1)

	bl	cpu_init_f
	nop

	li	r3, 0
	bl	board_init_f
	nop
	/* board_init_f doesn't return */
	b	.

_GLOBAL(reloc_and_go)
	mr	r1, r5
	li	r0, 0
	stdu	r0, -32(r1)
	bl	1f
1:	mflr	r9
	add	r9, r9, r6
	addi	r9, r9, 2f - 1b
	mtlr	r9
	blr			/* jump into the copy at 2f */
2:	add	r2, r2, r6
	mr	r31, r3
	mr	r30, r4
	mr	r3, r6
	bl	relocate
	nop
	mr	r3, r31
	mr	r4, r30
	bl	board_init_r
	nop
	b	.
