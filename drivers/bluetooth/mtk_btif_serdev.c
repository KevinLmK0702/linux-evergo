// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek BTIF (Bluetooth Interface) serdev controller
 *
 * BTIF is the internal serial block MTK SoCs use to reach the Bluetooth core
 * inside the CONN subsystem.  It looks like a 16550 but has no baud generator:
 * the link to the MCU runs at a fixed rate, so a serdev controller only has to
 * provide the write path, the receive path and the line-status handshake.
 *
 * Register map and bit layouts were taken from the vendor tree,
 * drivers/misc/mediatek/btif/common/plat_inc/{btif_priv.h,plat_common.h}, and
 * verified on hardware through the CONN driver's /proc/conn_wmt path.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serdev.h>

/* Register offsets. */
#define BTIF_RBR		0x00	/* read:  receive buffer */
#define BTIF_THR		0x00	/* write: transmit holding */
#define BTIF_IER		0x04
#define BTIF_FIFOCTRL		0x08
#define BTIF_FAKELCR		0x0C
#define BTIF_LSR		0x14
#define BTIF_DMA_EN		0x4C
#define BTIF_TRI_LVL		0x60
#define BTIF_WAK		0x64
#define BTIF_HANDSHAKE		0x6C

/* LSR bits. */
#define BTIF_LSR_DR		BIT(0)
#define BTIF_LSR_THRE		BIT(5)
#define BTIF_LSR_TEMT		BIT(6)

/* IER bits. */
#define BTIF_IER_RXFEN		BIT(0)

/* FIFOCTRL bits. */
#define BTIF_FIFOCTRL_CLR_RX	BIT(1)
#define BTIF_FIFOCTRL_CLR_TX	BIT(2)

/* DMA_EN bits. */
#define BTIF_DMA_EN_RX		BIT(0)
#define BTIF_DMA_EN_TX		BIT(1)
#define BTIF_DMA_EN_AUTORST	BIT(2)

/* HANDSHAKE bits. */
#define BTIF_HANDSHAKE_EN	BIT(0)

/* TRI_LVL fields: TX 16-byte FIFO, RX 8-byte FIFO. */
#define BTIF_TRI_LVL_TX(x)	(((x) & 0xf) << 0)
#define BTIF_TRI_LVL_RX(x)	(((x) & 0x7) << 4)

#define BTIF_RX_CHUNK		64
#define BTIF_POLL_US		1
#define BTIF_POLL_GUARD		20000

struct mtk_btif {
	struct serdev_controller *ctrl;
	void __iomem *base;
	struct clk *clk_btif;
	struct clk *clk_apdma;
};

static void mtk_btif_putc(struct mtk_btif *bt, u8 byte)
{
	int guard = BTIF_POLL_GUARD;

	while (!(readl(bt->base + BTIF_LSR) & (BTIF_LSR_THRE | BTIF_LSR_TEMT))) {
		if (!--guard)
			return;
		udelay(BTIF_POLL_US);
	}
	writeb(byte, bt->base + BTIF_THR);
}

static ssize_t mtk_btif_serdev_write_buf(struct serdev_controller *ctrl,
					 const u8 *buf, size_t count)
{
	struct mtk_btif *bt = serdev_controller_get_drvdata(ctrl);
	size_t i;

	for (i = 0; i < count; i++)
		mtk_btif_putc(bt, buf[i]);

	return count;
}

static void mtk_btif_serdev_write_flush(struct serdev_controller *ctrl)
{
	struct mtk_btif *bt = serdev_controller_get_drvdata(ctrl);
	int guard = BTIF_POLL_GUARD;

	while (!(readl(bt->base + BTIF_LSR) & BTIF_LSR_TEMT)) {
		if (!--guard)
			return;
		udelay(BTIF_POLL_US);
	}
}

static int mtk_btif_serdev_open(struct serdev_controller *ctrl)
{
	return 0;
}

static void mtk_btif_serdev_close(struct serdev_controller *ctrl)
{
}

static const struct serdev_controller_ops mtk_btif_serdev_ops = {
	.write_buf	= mtk_btif_serdev_write_buf,
	.write_flush	= mtk_btif_serdev_write_flush,
	.open		= mtk_btif_serdev_open,
	.close		= mtk_btif_serdev_close,
};

static irqreturn_t mtk_btif_irq(int irq, void *data)
{
	struct mtk_btif *bt = data;
	u8 buf[BTIF_RX_CHUNK];
	int n = 0;

	while (n < BTIF_RX_CHUNK) {
		if (!(readl(bt->base + BTIF_LSR) & BTIF_LSR_DR))
			break;
		buf[n++] = readl(bt->base + BTIF_RBR) & 0xff;
	}

	if (n)
		serdev_controller_receive_buf(bt->ctrl, buf, n);

	return IRQ_HANDLED;
}

/*
 * Program BTIF the way vendor hal_btif_hw_init() does: normal LCR, MCU
 * handshake enabled, both FIFOs flushed, trigger levels set and PIO mode with
 * only the receive interrupt armed.
 */
static void mtk_btif_hw_init(struct mtk_btif *bt)
{
	void __iomem *b = bt->base;
	u32 v;

	writel(0, b + BTIF_FAKELCR);

	v = readl(b + BTIF_HANDSHAKE);
	writel(v | BTIF_HANDSHAKE_EN, b + BTIF_HANDSHAKE);

	v = readl(b + BTIF_FIFOCTRL);
	writel(v | BTIF_FIFOCTRL_CLR_RX, b + BTIF_FIFOCTRL);
	writel(v & ~BTIF_FIFOCTRL_CLR_RX, b + BTIF_FIFOCTRL);
	v = readl(b + BTIF_FIFOCTRL);
	writel(v | BTIF_FIFOCTRL_CLR_TX, b + BTIF_FIFOCTRL);
	writel(v & ~BTIF_FIFOCTRL_CLR_TX, b + BTIF_FIFOCTRL);

	writel(BTIF_TRI_LVL_TX(8) | BTIF_TRI_LVL_RX(1), b + BTIF_TRI_LVL);

	/* PIO only: clear the RX/TX DMA enables, keep auto-reset. */
	v = readl(b + BTIF_DMA_EN);
	v &= ~(BTIF_DMA_EN_RX | BTIF_DMA_EN_TX);
	writel(v | BTIF_DMA_EN_AUTORST, b + BTIF_DMA_EN);

	writel(BTIF_IER_RXFEN, b + BTIF_IER);

	/* Wake the peer. */
	v = readl(b + BTIF_WAK);
	writel(v | 1, b + BTIF_WAK);
	udelay(50);

	dev_info(&bt->ctrl->dev, "BTIF ready: handshake=%#010x lsr=%#010x\n",
		 readl(b + BTIF_HANDSHAKE), readl(b + BTIF_LSR));
}

static int mtk_btif_serdev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct serdev_controller *ctrl;
	struct mtk_btif *bt;
	void __iomem *base;
	struct clk *clk_btif, *clk_apdma;
	int ret, irq;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base), "cannot map BTIF\n");

	clk_btif = devm_clk_get(dev, "btifc");
	if (IS_ERR(clk_btif))
		return dev_err_probe(dev, PTR_ERR(clk_btif), "no btifc clock\n");

	clk_apdma = devm_clk_get_optional(dev, "apdmac");
	if (IS_ERR(clk_apdma))
		return dev_err_probe(dev, PTR_ERR(clk_apdma), "bad apdmac clock\n");

	ret = clk_prepare_enable(clk_btif);
	if (ret)
		return ret;

	if (clk_apdma) {
		ret = clk_prepare_enable(clk_apdma);
		if (ret)
			goto err_btif_clk;
	}

	ctrl = serdev_controller_alloc(dev, dev, sizeof(*bt));
	if (!ctrl) {
		ret = -ENOMEM;
		goto err_all_clk;
	}

	bt = serdev_controller_get_drvdata(ctrl);
	bt->ctrl = ctrl;
	bt->base = base;
	bt->clk_btif = clk_btif;
	bt->clk_apdma = clk_apdma;

	ctrl->ops = &mtk_btif_serdev_ops;
	platform_set_drvdata(pdev, bt);

	mtk_btif_hw_init(bt);

	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, mtk_btif_irq, 0,
				       dev_name(dev), bt);
		if (ret) {
			dev_warn(dev, "IRQ %d unusable (%d), RX disabled\n",
				 irq, ret);
			/* Leave RX interrupts off so we do not spin. */
			writel(0, bt->base + BTIF_IER);
		}
	} else {
		dev_warn(dev, "no RX interrupt, TX only\n");
		writel(0, bt->base + BTIF_IER);
	}

	ret = serdev_controller_add(ctrl);
	if (ret)
		goto err_all_clk;

	return 0;

err_all_clk:
	if (clk_apdma)
		clk_disable_unprepare(clk_apdma);
err_btif_clk:
	clk_disable_unprepare(clk_btif);

	return ret;
}

static void mtk_btif_serdev_remove(struct platform_device *pdev)
{
	struct mtk_btif *bt = platform_get_drvdata(pdev);

	serdev_controller_remove(bt->ctrl);
	serdev_controller_put(bt->ctrl);

	if (bt->clk_apdma)
		clk_disable_unprepare(bt->clk_apdma);
	clk_disable_unprepare(bt->clk_btif);
}

static const struct of_device_id mtk_btif_of_match[] = {
	{ .compatible = "mediatek,btif" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_btif_of_match);

static struct platform_driver mtk_btif_serdev_driver = {
	.probe	= mtk_btif_serdev_probe,
	.remove	= mtk_btif_serdev_remove,
	.driver	= {
		.name		= "mtk-btif",
		.of_match_table	= mtk_btif_of_match,
	},
};
module_platform_driver(mtk_btif_serdev_driver);

MODULE_DESCRIPTION("MediaTek BTIF serdev controller");
MODULE_LICENSE("GPL");
