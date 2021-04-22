/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Code to process dynamic relocations in u-boot.
 *
 * Copyright 2008 Paul Mackerras, IBM Corp.
 */

#include <ppc_asm.tmpl>
#include <ppc_defs.h>
#include <asm/u-boot.h>

RELA = 7
RELACOUNT = 0x6ffffff9
R_PPC64_RELATIVE = 22

/*
 * r3 = relocation offset to be applied, i.e. final address - link-time address
 * The image could currently be based at an address which is
 * neither the link-time address nor the final address.
 */
_GLOBAL(relocate)
	mflr	r0
	bcl	20,31,$+4
0:	mflr	r12		/* r12 has runtime addr of label 0 */
	mtlr	r0
	ld	r11,(p_dyn - 0b)(r12)
	add	r11,r11,r12	/* r11 has runtime addr of .dynamic section */
	ld	r9,(p_rela - 0b)(r12)
	add	r9,r9,r12	/* r9 has runtime addr of .rela.dyn section */

	/*
	 * Scan the dynamic section for the RELA and RELACOUNT entries.
	 */
	li	r7,0
	li	r8,0
1:	ld	r6,0(r11)	/* get tag */
	cmpdi	r6,0
	beq	4f		/* end of list */
	cmpdi	r6,RELA
	bne	2f
	ld	r7,8(r11)	/* get RELA pointer in r7 */
	b	3f
2:	addis	r6,r6,(-RELACOUNT)@ha
	cmpdi	r6,RELACOUNT@l
	bne	3f
	ld	r8,8(r11)	/* get RELACOUNT value in r8 */
3:	addi	r11,r11,16
	b	1b
4:	cmpdi	r7,0		/* check we have both RELA and RELACOUNT */
	cmpdi	cr1,r8,0
	beq	6f
	beq	cr1,6f
	subf	r7,r7,r9	/* cur_offset = rela.run - rela.link */

	/*
	 * Run through the list of relocations and process the
	 * R_PPC64_RELATIVE ones.
	 */
	mtctr	r8
5:	ld	r0,8(r9)	/* ELF64_R_TYPE(reloc->r_info) */
	cmpdi	r0,R_PPC64_RELATIVE
	bne	6f
	ld	r6,0(r9)	/* reloc->r_offset */
	ld	r0,16(r9)	/* reloc->r_addend */
	add	r0,r0,r3
	stdx	r0,r7,r6
	addi	r9,r9,24
	bdnz	5b

6:	blr

.balign 8
p_dyn:	.8byte	__dynamic_start - 0b
p_rela:	.8byte	__rela_dyn_start - 0b
