// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for litex-mmc interface.
 *
 * Mostly borrowed from the Linux driver.
 *
 * Copyright (C) 2019-2020 Antmicro <www.antmicro.com>
 * Copyright 2021 Paul Mackerras, IBM Corp.
 *
 * With contributions from Gabriel Somlo.
 */
#include <common.h>
#include <dm.h>
#include <mmc.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "litex_mmc.h"

#define SDCARD_CTRL_DATA_TRANSFER_NONE  0
#define SDCARD_CTRL_DATA_TRANSFER_READ  1
#define SDCARD_CTRL_DATA_TRANSFER_WRITE 2

#define SDCARD_CTRL_RESPONSE_NONE	0
#define SDCARD_CTRL_RESPONSE_SHORT	1
#define SDCARD_CTRL_RESPONSE_LONG	2
#define SDCARD_CTRL_RESPONSE_SHORT_BUSY	3

#define SD_OK         0

#define MMC_BUS_WIDTH_4	2

struct litex_mmc_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

#define REGS_SDPHY	0
#define REGS_SDCORE	1
#define REGS_SDREADER	2
#define REGS_SDWRITER	3

struct litex_mmc_host {
	u32 sys_clock;
	u32 clock;
	u8 app_cmd;
	u8 is_bus_width_set;
	u16 rca;
	void __iomem *sdphy;
	void __iomem *sdcore;
	void __iomem *sdreader;
	void __iomem *sdwriter;
	struct udevice *dev;
	struct mmc *mmc;
	struct litex_mmc_plat *plat;
	u32 resp[4];
	char *data;
};

void sdclk_set_clk(struct litex_mmc_host *host, unsigned int clk_freq)
{
	u32 div = clk_freq ? host->sys_clock / clk_freq : 256;
	u32 p2;

	/* Going below about 6.25MHz seems to cause problems */
	for (p2 = 2; p2 < 8; p2 *= 2)
		if (p2 >= div)
			break;
	litex_write16(host->sdphy + LITEX_MMC_SDPHY_CLOCKERDIV_OFF, p2);
}


static int sdcard_wait_done(void __iomem *reg)
{
	u8 evt;

	for (;;) {
		evt = litex_read8(reg);
		if (evt & 0x1)
			break;
		udelay(5);
	}
	if (evt == 0x1)
		return SD_OK;
	if (evt & 0x2)
		return -EIO;
	if (evt & 0x4)
		return -ETIMEDOUT;
	if (evt & 0x8)
		return -EILSEQ;
	pr_err("sdcard_wait_done: unknown error evt=%x\n", evt);
	return -EINVAL;
}

static int send_cmd(struct litex_mmc_host *host, u8 cmd, u32 arg,
		    u8 response_len, u8 transfer)
{
	void __iomem *reg;
	int n;
	int i;
	int status;

	litex_write32(host->sdcore + LITEX_MMC_SDCORE_CMDARG_OFF, arg);
	litex_write32(host->sdcore + LITEX_MMC_SDCORE_CMDCMD_OFF,
			 cmd << 8 | transfer << 5 | response_len);
	litex_write8(host->sdcore + LITEX_MMC_SDCORE_CMDSND_OFF, 1);

	status = sdcard_wait_done(host->sdcore + LITEX_MMC_SDCORE_CMDEVT_OFF);

	if (status != SD_OK) {
		printf("litex_mmc: Command (%s %d) failed, status %d\n",
		       (host->app_cmd? "app": "cmd"), cmd, status);
		return status;
	}

	if (response_len != SDCARD_CTRL_RESPONSE_NONE) {
		reg = host->sdcore + LITEX_MMC_SDCORE_CMDRSP_OFF;
		for (i = 0; i < 4; i++) {
			host->resp[i] = litex_read32(reg);
			reg += _next_reg_off(0, sizeof(u32));
		}
	}

	udelay(10);

	if (!host->app_cmd && cmd == SD_CMD_SEND_RELATIVE_ADDR)
		host->rca = (host->resp[3] >> 16) & 0xffff;

	host->app_cmd = (cmd == MMC_CMD_APP_CMD);

	if (transfer == SDCARD_CTRL_DATA_TRANSFER_NONE)
		return status; /* SD_OK from prior sdcard_wait_done(cmd_evt) */

	status = sdcard_wait_done(host->sdcore + LITEX_MMC_SDCORE_DATAEVT_OFF);
	if (status != SD_OK){
		pr_err("Data xfer (cmd %d) failed, status %d\n", cmd, status);
		return status;
	}

	/* wait for completion of (read or write) DMA transfer */
	reg = (transfer == SDCARD_CTRL_DATA_TRANSFER_READ) ?
		host->sdreader + LITEX_MMC_SDBLK2MEM_DONE_OFF :
		host->sdwriter + LITEX_MMC_SDMEM2BLK_DONE_OFF;
	n = 200000;
	while ((litex_read8(reg) & 0x01) == 0) {
		if (--n < 0) {
			printf("litex_mmc: DMA timeout (cmd %d)\n", cmd);
			return -ETIMEDOUT;
		}
		udelay(10);
	}

	return status;
}

// CMD55
static inline int send_app_cmd(struct litex_mmc_host *host)
{
	return send_cmd(host, MMC_CMD_APP_CMD, host->rca << 16,
			SDCARD_CTRL_RESPONSE_SHORT,
			SDCARD_CTRL_DATA_TRANSFER_NONE);
}

// ACMD6
static inline int send_app_set_bus_width_cmd(
		struct litex_mmc_host *host, u32 width)
{
	return send_cmd(host, SD_CMD_APP_SET_BUS_WIDTH, width,
			SDCARD_CTRL_RESPONSE_SHORT,
			SDCARD_CTRL_DATA_TRANSFER_NONE);
}

static int litex_set_bus_width(struct litex_mmc_host *host)
{
	bool app_cmd_sent = host->app_cmd; /* was preceding command app_cmd? */
	int status;

	/* ensure 'app_cmd' precedes 'app_set_bus_width_cmd' */
	if (!app_cmd_sent)
		send_app_cmd(host);

	/* litesdcard only supports 4-bit bus width */
	status = send_app_set_bus_width_cmd(host, MMC_BUS_WIDTH_4);

	/* re-send 'app_cmd' if necessary */
	if (app_cmd_sent)
		send_app_cmd(host);

	return status;
}

static int litex_mmc_get_cd(struct udevice *dev)
{
	struct litex_mmc_host *host = dev_get_priv(dev);
	int ret;

	/* use gateware card-detect bit */
	ret = !litex_read8(host->sdphy +
			   LITEX_MMC_SDPHY_CARDDETECT_OFF);

	/* ensure bus width will be set (again) upon card (re)insertion */
	if (ret == 0)
		host->is_bus_width_set = false;

	return ret;
}

static u32 litex_response_len(struct mmc_cmd *cmd)
{
	u32 response_len = SDCARD_CTRL_RESPONSE_NONE;

	if (cmd->resp_type & MMC_RSP_136) {
		response_len = SDCARD_CTRL_RESPONSE_LONG;
	} else if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_BUSY)
			response_len = SDCARD_CTRL_RESPONSE_SHORT_BUSY;
		else
			response_len = SDCARD_CTRL_RESPONSE_SHORT;
	}

	return response_len;
}

/*
 * Send request to a card. Command, data transfer, things like this.
 */
static int litex_mmc_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	struct litex_mmc_host *host = dev_get_priv(dev);
	int status;
	dma_addr_t dma_handle;
	unsigned int length = 0;

	u32 response_len = litex_response_len(cmd);
	u32 transfer = SDCARD_CTRL_DATA_TRANSFER_NONE;

	/* First check that the card is still there */
	if (litex_read8(host->sdphy + LITEX_MMC_SDPHY_CARDDETECT_OFF))
		return -ENOMEDIUM;

	host->data = NULL;
	if (data) {
		/* LiteSDCard only supports 4-bit bus width; therefore, we MUST
		 * inject a SET_BUS_WIDTH (acmd6) before the very first data
		 * transfer, earlier than when the mmc subsystem would normally
		 * get around to it!
		 */
		if (!host->is_bus_width_set) {
			if (litex_set_bus_width(host) != SD_OK) {
				printf("litex_mmc: Can't set bus width!\n");
				return -ETIMEDOUT;
			}
			host->is_bus_width_set = true;
		}

		/*
		 * We do DMA directly to/from the data buffer.
		 */
		dma_handle = (dma_addr_t) data->dest;
		length = data->blocksize * data->blocks;
		host->data = data->dest;

		if (data->flags & MMC_DATA_READ) {
			litex_write8(host->sdreader +
					 LITEX_MMC_SDBLK2MEM_ENA_OFF, 0);
			litex_write64(host->sdreader +
					 LITEX_MMC_SDBLK2MEM_BASE_OFF,
					 dma_handle);
			litex_write32(host->sdreader +
					 LITEX_MMC_SDBLK2MEM_LEN_OFF,
					 length);
			litex_write8(host->sdreader +
					 LITEX_MMC_SDBLK2MEM_ENA_OFF, 1);

			transfer = SDCARD_CTRL_DATA_TRANSFER_READ;

		} else if (data->flags & MMC_DATA_WRITE) {
			litex_write8(host->sdwriter +
					 LITEX_MMC_SDMEM2BLK_ENA_OFF, 0);
			litex_write64(host->sdwriter +
					 LITEX_MMC_SDMEM2BLK_BASE_OFF,
					 dma_handle);
			litex_write32(host->sdwriter +
					 LITEX_MMC_SDMEM2BLK_LEN_OFF,
					 length);
			litex_write8(host->sdwriter +
					 LITEX_MMC_SDMEM2BLK_ENA_OFF, 1);

			transfer = SDCARD_CTRL_DATA_TRANSFER_WRITE;
		}

		litex_write16(host->sdcore + LITEX_MMC_SDCORE_BLKLEN_OFF,
				 data->blocksize);
		litex_write32(host->sdcore + LITEX_MMC_SDCORE_BLKCNT_OFF,
				 data->blocks);
	}

	status = send_cmd(host, cmd->cmdidx, cmd->cmdarg,
			  response_len, transfer);
	host->data = NULL;

	if (status != SD_OK)
		/* card may be gone; don't assume bus width is still set */
		host->is_bus_width_set = false;

	// It looks strange I know, but it's as it should be
	if (response_len == SDCARD_CTRL_RESPONSE_SHORT ||
	    response_len == SDCARD_CTRL_RESPONSE_SHORT_BUSY) {
		cmd->response[0] = host->resp[3];
		cmd->response[1] = host->resp[2] & 0xFF;
	} else if (response_len == SDCARD_CTRL_RESPONSE_LONG) {
		cmd->response[0] = host->resp[0];
		cmd->response[1] = host->resp[1];
		cmd->response[2] = host->resp[2];
		cmd->response[3] = host->resp[3];
	}

	if (transfer == SDCARD_CTRL_DATA_TRANSFER_READ)
		asm volatile ("dcbf 0,%0" : : "r" (data->dest));

	return status;
}

static int litex_mmc_set_ios(struct udevice *dev)
{
	struct litex_mmc_host *host = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	/* updated mmc->bus_width -- do nothing;
	 * This happens right after the mmc core subsystem has sent its
	 * own acmd6 to notify the card of the bus-width change, and it's
	 * effectively a no-op given that we already forced bus-width to 4
	 * by snooping on the command flow, and inserting an acmd6 before
	 * the first data xfer comand!
	 */

	if (mmc->clock != host->clock) {
		sdclk_set_clk(host, mmc->clock);
		host->clock = mmc->clock;
	}

	return 0;
}

static int litex_mmc_probe(struct udevice *dev)
{
	struct litex_mmc_plat *plat = dev_get_plat(dev);
	struct litex_mmc_host *host = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	unsigned long phys_addr;
	unsigned long i;
	void __iomem *regs[4];

	host->dev = dev;
	host->mmc = mmc;
	host->plat = plat;
	upriv->mmc = &plat->mmc;
	plat->cfg.name = dev->name;

	for (i = 0; i < 4; ++i) {
		phys_addr = dev_read_addr_index(dev, i);
		if (phys_addr == FDT_ADDR_T_NONE)
			return -EINVAL;
		regs[i] = (void __iomem *) phys_addr;
	}
	host->sdphy = regs[REGS_SDPHY];
	host->sdcore = regs[REGS_SDCORE];
	host->sdreader = regs[REGS_SDREADER];
	host->sdwriter = regs[REGS_SDWRITER];

	host->sys_clock = 100000000;	/* XXX get from fdt */

	plat->cfg.f_max = host->sys_clock / 2;
	plat->cfg.f_min = host->sys_clock / 512;
	plat->cfg.b_max = 65535;

	plat->cfg.host_caps = MMC_MODE_4BIT  | MMC_CAP_NEEDS_POLL;
	if (plat->cfg.f_max > 25000000)
		plat->cfg.host_caps |= MMC_MODE_HS;

	plat->cfg.voltages = MMC_VDD_32_33 | MMC_VDD_33_34;

	return 0;
}

static const struct udevice_id litex_mmc_match[] = {
	{ .compatible = "litex,mmc" },
	{ }
};

static const struct dm_mmc_ops litex_mmc_ops = {
	.send_cmd = litex_mmc_send_cmd,
	.set_ios = litex_mmc_set_ios,
	.get_cd = litex_mmc_get_cd,
};

static int litex_mmc_bind(struct udevice *dev)
{
	struct litex_mmc_plat *plat = dev_get_plat(dev);

	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

U_BOOT_DRIVER(litex_mmc_host) = {
	.name = "litex-mmc-host",
	.id = UCLASS_MMC,
	.of_match = litex_mmc_match,
	.bind = litex_mmc_bind,
	.probe = litex_mmc_probe,
	.priv_auto = sizeof(struct litex_mmc_host),
	.plat_auto = sizeof(struct litex_mmc_plat),
	.ops = &litex_mmc_ops,
};
