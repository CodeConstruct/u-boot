#include <common.h>
#include <init.h>
#include <asm/global_data.h>
#include <asm/processor.h>

gd_t *gd = (gd_t *)(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_GBL_DATA_OFFSET);

void cpu_init_f(void)
{
	unsigned long i;

	board_init_f_init_reserve(CONFIG_SYS_INIT_RAM_ADDR +
				  CONFIG_SYS_GBL_DATA_OFFSET);

	for (i = 0; i < 0x1000; i += 0x100) {
		*(unsigned int *)i = 0x7d5c02a6;
		*(unsigned int *)(i + 4) = 0x48000000;
	}
	__asm__ volatile("isync; icbi 0,0");
}

int cpu_init_r(void)
{
	return 0;
}

extern u32 intr_handler[8];
extern u32 handle_interrupt[];

void trap_init(unsigned long x)
{
	unsigned long vec;

	for (vec = 0x100; vec < 0x1000; vec += 0x100)
		memcpy((u32 *)vec, intr_handler, 32);
	__asm__ volatile("isync; icbi 0,0");
	mtspr(SPRN_SPRG0, (unsigned long) handle_interrupt);
}

void reloc_and_go(struct global_data *new_gd, ulong relocaddr,
		  ulong start_addr_sp, ulong reloc_off)
	__attribute__ ((noreturn));

extern char _stext[], _end[];
extern char __bss_start[], __bss_end[];

__attribute__ ((noreturn)) void
relocate_code(ulong start_addr_sp, struct global_data *new_gd,
	      ulong relocaddr)
{
	unsigned long tpd_size = __bss_start - _stext;
	unsigned long bss_size = __bss_end - __bss_start;

	memcpy((void *)relocaddr, _stext, tpd_size);
	memset((void *)relocaddr + tpd_size, 0, bss_size);
	__asm__ volatile("sync; icbi 0,%0" : : "r" (relocaddr));
	reloc_and_go(new_gd, relocaddr, start_addr_sp, new_gd->reloc_off);
	for (;;)
		;
}

