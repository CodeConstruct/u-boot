#include <common.h>
#include <cpu_func.h>
#include <asm/global_data.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

static void
__board_reset(void)
{
}
void board_reset(void) __attribute__((weak, alias("__board_reset")));

int
checkcpu(void)
{
	puts("CPU: Microwatt\n");
	return 0;
}

int do_reset(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	board_reset();

	while (1)
		;
}

unsigned long
get_tbclk(void)
{
	return 100000000;
}

void print_reginfo(void)
{
}

int dram_init(void)
{
	/* XXX get from syscon? */
	gd->ram_size = 0x10000000;

	return 0;
}
