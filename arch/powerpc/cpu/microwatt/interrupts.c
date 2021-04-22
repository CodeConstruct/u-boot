#include <common.h>
#include <config.h>
#include <irq_func.h>

void interrupt_init_cpu(unsigned *dec_cnt)
{
	*dec_cnt = get_tbclk() / CONFIG_SYS_HZ;
}

void timer_interrupt_cpu(struct pt_regs *regs)
{
}

void irq_install_handler(int vec, interrupt_handler_t *handler, void *arg)
{
}

void irq_free_handler(int vec)
{
}

int do_irqinfo(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	return 0;
}

void external_interrupt(struct pt_regs *regs)
{
}
