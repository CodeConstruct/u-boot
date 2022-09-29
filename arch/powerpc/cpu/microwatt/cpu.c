#include <common.h>
#include <cpu_func.h>
#include <asm/global_data.h>
#include <asm/io.h>

/* From microwatt_soc.h */
#define SYSCON_BASE	0xc0000000  /* System control regs */

#define SYS_REG_SIGNATURE		0x00
#define SYS_REG_INFO			0x08
#define   SYS_REG_INFO_HAS_UART 		(1ull << 0)
#define   SYS_REG_INFO_HAS_DRAM 		(1ull << 1)
#define   SYS_REG_INFO_HAS_BRAM 		(1ull << 2)
#define   SYS_REG_INFO_HAS_SPI_FLASH 		(1ull << 3)
#define   SYS_REG_INFO_HAS_LITEETH 		(1ull << 4)
#define   SYS_REG_INFO_HAS_LARGE_SYSCON	        (1ull << 5)
#define   SYS_REG_INFO_HAS_UART1 		(1ull << 6)
#define   SYS_REG_INFO_HAS_ARTB                 (1ull << 7)
#define   SYS_REG_INFO_HAS_LITESDCARD 		(1ull << 8)
#define SYS_REG_BRAMINFO		0x10
#define   SYS_REG_BRAMINFO_SIZE_MASK		0xfffffffffffffull
#define SYS_REG_DRAMINFO		0x18
#define   SYS_REG_DRAMINFO_SIZE_MASK		0xfffffffffffffull
#define SYS_REG_CLKINFO			0x20
#define   SYS_REG_CLKINFO_FREQ_MASK		0xffffffffffull
#define SYS_REG_CTRL			0x28
#define   SYS_REG_CTRL_DRAM_AT_0		(1ull << 0)
#define   SYS_REG_CTRL_CORE_RESET		(1ull << 1)
#define   SYS_REG_CTRL_SOC_RESET		(1ull << 2)
#define SYS_REG_DRAMINITINFO		0x30
#define SYS_REG_SPI_INFO		0x38
#define   SYS_REG_SPI_INFO_FLASH_OFF_MASK	0xffffffff
#define SYS_REG_UART0_INFO		0x40
#define SYS_REG_UART1_INFO		0x48
#define   SYS_REG_UART_IS_16550			(1ull << 32)
#define SYS_REG_GIT_INFO		0x50
#define   SYS_REG_GIT_IS_DIRTY			(1ull << 63)

DECLARE_GLOBAL_DATA_PTR;

int
checkcpu(void)
{
	puts("CPU: Microwatt\n");
	return 0;
}

int do_reset(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	u32 v = readl(SYSCON_BASE + SYS_REG_CTRL);
	writel(v | SYS_REG_CTRL_SOC_RESET, SYSCON_BASE + SYS_REG_CTRL);
	/* not reached */
	return 1;
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
