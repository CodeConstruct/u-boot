// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <asm/global_data.h>
#include <asm/processor.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

int get_clocks(void)
{
	/* XXX get from syscon */
	gd->cpu_clk = 100000000;
	gd->bus_clk = 100000000;
	return 0;
}

int get_serial_clock(void)
{
	return gd->cpu_clk = 100000000;
}
