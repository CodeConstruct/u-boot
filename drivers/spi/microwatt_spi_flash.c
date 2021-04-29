// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Paul Mackerras, IBM Corp.
 */

#include <common.h>
#include <dm.h>
#include <malloc.h>
#include <spi.h>
#include <asm/io.h>

/*
 * Register definitions for the SPI flash controller
 */
#define SPI_REG_DATA       		0x00 /* Byte access: single wire transfer */
#define SPI_REG_DATA_DUAL       	0x01 /* Byte access: dual wire transfer */
#define SPI_REG_DATA_QUAD       	0x02 /* Byte access: quad wire transfer */
#define SPI_REG_CTRL			0x04 /* Reset and manual mode control */
#define   SPI_REG_CTRL_RESET            	0x01  /* reset all registers */
#define   SPI_REG_CTRL_MANUAL_CS	        0x02  /* assert CS, enable manual mode */
#define   SPI_REG_CTRL_CKDIV_SHIFT		8     /* clock div */
#define   SPI_REG_CTRL_CKDIV_MASK		(0xff << SPI_REG_CTRL_CKDIV_SHIFT)
#define SPI_REG_AUTO_CFG		0x08 /* Automatic map configuration */
#define   SPI_REG_AUTO_CFG_CMD_SHIFT		0     /* Command to use for reads */
#define   SPI_REG_AUTO_CFG_CMD_MASK		(0xff << SPI_REG_AUTO_CFG_CMD_SHIFT)
#define   SPI_REG_AUTO_CFG_DUMMIES_SHIFT        8     /* # dummy cycles */
#define   SPI_REG_AUTO_CFG_DUMMIES_MASK         (0x7  << SPI_REG_AUTO_CFG_DUMMIES_SHIFT)
#define   SPI_REG_AUTO_CFG_MODE_SHIFT           11    /* SPI wire mode */
#define   SPI_REG_AUTO_CFG_MODE_MASK            (0x3  << SPI_REG_AUTO_CFG_MODE_SHIFT)
#define     SPI_REG_AUT_CFG_MODE_SINGLE         (0 << 11)
#define     SPI_REG_AUT_CFG_MODE_DUAL           (2 << 11)
#define     SPI_REG_AUT_CFG_MODE_QUAD           (3 << 11)
#define   SPI_REG_AUTO_CFG_ADDR4                (1u << 13) /* 3 or 4 addr bytes */
#define   SPI_REG_AUTO_CFG_CKDIV_SHIFT          16    /* clock div */
#define   SPI_REG_AUTO_CFG_CKDIV_MASK           (0xff << SPI_REG_AUTO_CFG_CKDIV_SHIFT)
#define   SPI_REG_AUTO_CFG_CSTOUT_SHIFT         24    /* CS timeout */
#define   SPI_REG_AUTO_CFG_CSTOUT_MASK          (0x3f << SPI_REG_AUTO_CFG_CSTOUT_SHIFT)

struct microwatt_spi_priv {
	u8 __iomem *regs;
	u32	clock;
};

static int microwatt_spi_of_to_plat(struct udevice *dev)
{
	struct microwatt_spi_priv *priv = dev_get_priv(dev);

	priv->regs = (u8 __iomem *) dev_read_addr(dev);
	priv->clock = 100000000;	/* XXX get from device tree */

	return 0;
}

static int microwatt_spi_probe(struct udevice *dev)
{
	return 0;
}

static int microwatt_spi_xfer(struct udevice *dev, uint bitlen,
			      const void *outp, void *inp, ulong flags)
{
	struct udevice *bus = dev->parent;
	struct microwatt_spi_priv *priv = dev_get_priv(bus);
	const u8 *op = outp;
	u8 *ip = inp;
	unsigned long len;

	if (bitlen & 7)
		return -ENOTSUPP;

	if (flags & SPI_XFER_BEGIN)
		writeb(SPI_REG_CTRL_MANUAL_CS, priv->regs + SPI_REG_CTRL);

	/*
	 * There is a mismatch between the expectations of (some) callers
	 * of this function, and how the spi_flash interface works: for
	 * a given byte time (8 clocks), the interface doesn't let us both
	 * specify the bits to be transmitted and retrieve the bits
	 * received; you can only do one or the other.
	 * Fortunately, many callers only have one or outp and inp non-NULL,
	 * so for now we assume we're doing output if outp != NULL.
	 */
	for (len = bitlen >> 3; len; --len) {
		if (outp)
			writeb(*op++, priv->regs + SPI_REG_DATA);
		else if (inp)
			*ip++ = readb(priv->regs + SPI_REG_DATA);
	}

	if (flags & SPI_XFER_END) {
		writeb(0, priv->regs + SPI_REG_CTRL);
		__asm__ volatile("sync");
	}

	return 0;
}

static int microwatt_spi_set_speed(struct udevice *dev, uint speed)
{
	struct microwatt_spi_priv *priv = dev_get_priv(dev);
	unsigned int divisor;

	divisor = (priv->clock / 2 + speed - 1) / speed;
	if (divisor > 256)
		divisor = 256;
	writel(((divisor - 1) << SPI_REG_CTRL_CKDIV_SHIFT) & SPI_REG_CTRL_CKDIV_MASK,
	       (u32 __iomem *)(priv->regs + SPI_REG_CTRL));

	return 0;
}

static int microwatt_spi_set_mode(struct udevice *dev, uint mode)
{
	return 0;
}

static const struct dm_spi_ops microwatt_spi_ops = {
	.xfer	 = microwatt_spi_xfer,
	.set_speed = microwatt_spi_set_speed,
	.set_mode  = microwatt_spi_set_mode,
};

static const struct udevice_id microwatt_spi_ids[] = {
	{ .compatible = "microwatt-spi-flash" },
	{ }
};

U_BOOT_DRIVER(microwatt_spi_flash) = {
	.name = "microwatt_spi_flash",
	.id   = UCLASS_SPI,
	.of_match = microwatt_spi_ids,
	.ops = &microwatt_spi_ops,
	.of_to_plat = microwatt_spi_of_to_plat,
	.probe = microwatt_spi_probe,
	.priv_auto  = sizeof(struct microwatt_spi_priv),
};
