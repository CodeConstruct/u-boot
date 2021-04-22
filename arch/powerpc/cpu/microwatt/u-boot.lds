/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2006, 2007 Freescale Semiconductor, Inc.
 */

OUTPUT_ARCH(powerpc:common64)
ENTRY(_start)

SECTIONS
{
  /* Read-only sections, merged into text segment: */
  .text :
  {
    _stext = .;
    *(.text*)
  }
  .rodata    :
  {
    *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
  }

  /* Read-write section, merged into data segment: */
  . = ALIGN(256);
  .dynstr : { *(.dynstr) }
  .dynamic :
  {
    __dynamic_start = .;
    *(.dynamic)
  }
  .rela.dyn :
  {
    __rela_dyn_start = .;
    *(.rela*)
  }
  .data    :
  {
    *(.data*)
    *(.sdata*)
  }
  . = ALIGN(64);
  .got :
  {
    *(.got)
    *(.toc)
  }
  _edata  =  .;
  PROVIDE (edata = .);

  . = .;

  . = ALIGN(4);
  .u_boot_list : {
	KEEP(*(SORT(.u_boot_list*)));
  }

  . = .;
  __start___ex_table = .;
  __ex_table : { *(__ex_table) }
  __stop___ex_table = .;

  . = ALIGN(256);
  __init_begin = .;
  .text.init : { *(.text.init) }
  .data.init : { *(.data.init) }
  . = ALIGN(256);
  __init_end = .;

  __bss_start = .;
  .bss (NOLOAD)       :
  {
   *(.bss*)
   *(.sbss*)
   *(COMMON)
   . = ALIGN(4);
  }
  __bss_end = . ;
  PROVIDE (end = .);
  PROVIDE (_end = .);
}
