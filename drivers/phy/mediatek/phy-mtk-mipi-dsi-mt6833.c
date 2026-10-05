// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 *
 * MT6833 MIPI TX PHY in C-PHY mode.
 *
 * The MT6833 display bus of the evergo (and of every mt6833 board using the
 * panels this tree knows about) is a C-PHY link: three trios, no clock lane,
 * 16/7 coding.  The mainline mt8183 D-PHY sequences cannot bring that up -
 * they leave RG_DSI_CPHY_EN clear and never load a lane map, so the panel
 * receives neither a clock nor any data.
 *
 * The sequences below are ported from the vendor kernel's
 * drivers/gpu/drm/mediatek/mtk_mipi_tx.c, where "mediatek,mt6833-mipi-tx-cphy"
 * is backed by mt6873_mipitx_cphy_data:
 *   - mtk_mipi_tx_pll_cphy_prepare_mt6873()
 *   - mtk_mipi_tx_pll_cphy_unprepare_mt6873()
 *   - the "no lane swap" branch of mtk_mipi_tx_cphy_lane_config()
 */

#include <linux/math64.h>
#include <linux/minmax.h>

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-dsi.h"

#define MIPITX_LANE_CON		0x000c
#define RG_DSI_CPHY_EN		BIT(3)
#define RG_DSI_BG_LPF_EN	BIT(6)
#define RG_DSI_BG_CORE_EN	BIT(7)

/* Whole-register values the vendor writes, reserved bits included. */
#define MT6833_LANE_CON_BG	0x3fff0080
#define MT6833_LANE_CON_CPHY	0x3fff0088
#define MT6833_LANE_CON_CPHY_LPF	0x3fff00c8
#define MT6833_LANE_CON_OFF	0x3fff0000

#define MIPITX_VOLTAGE_SEL	0x0010
#define MIPITX_PRESERVED	0x0014

#define MIPITX_PLL_PWR		0x0028
#define AD_DSI_PLL_SDM_PWR_ON	BIT(0)
#define AD_DSI_PLL_SDM_ISO_EN	BIT(1)

#define MIPITX_PLL_CON0		0x002c
#define MIPITX_PLL_CON1		0x0030
#define MIPITX_PLL_CON4		0x003c
#define RG_DSI_PLL_EN		BIT(4)
#define RG_DSI_PLL_POSDIV	GENMASK(10, 8)

#define MIPITX_PHY_SEL0		0x0040
#define MIPITX_PHY_SEL1		0x0044
#define MIPITX_PHY_SEL2		0x0048
#define MIPITX_PHY_SEL3		0x004c

static int mtk_mipi_tx_pll_cphy_enable(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;
	unsigned int txdiv, txdiv0;
	u64 pcw;
	u32 rate = mipi_tx->data_rate / 1000000;

	dev_dbg(mipi_tx->dev, "enable: %u MHz (C-PHY)\n", rate);

	if (rate >= 2000) {
		txdiv = 1;
		txdiv0 = 0;
	} else if (rate >= 1000) {
		txdiv = 2;
		txdiv0 = 1;
	} else if (rate >= 500) {
		txdiv = 4;
		txdiv0 = 2;
	} else if (rate > 250) {
		txdiv = 8;
		txdiv0 = 3;
	} else if (rate >= 125) {
		txdiv = 16;
		txdiv0 = 4;
	} else {
		return -EINVAL;
	}

	/*
	 * C-PHY bias and LDO reference.  The vendor value carries the
	 * 500 mV HSTX LDO reference this mode needs (0x6a) together with
	 * the bank's power-on defaults.
	 */
	writel(0x4444236a, base + MIPITX_VOLTAGE_SEL);
	writel(0x0, base + MIPITX_PRESERVED);
	writel(0x00ff12e0, base + MIPITX_PLL_CON4);

	/* BG_LPF_EN=0 BG_CORE_EN=1 with C-PHY selected, then enable BG_LPF */
	writel(MT6833_LANE_CON_CPHY, base + MIPITX_LANE_CON);
	writel(MT6833_LANE_CON_CPHY_LPF, base + MIPITX_LANE_CON);

	/* SDM power on, then release the isolation */
	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);
	udelay(100);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);

	pcw = div_u64(((u64)mipi_tx->data_rate * txdiv) << 24, 26000000);
	writel(pcw, base + MIPITX_PLL_CON0);

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);
	mtk_phy_update_field(base + MIPITX_PLL_CON1, RG_DSI_PLL_POSDIV,
			     txdiv0);
	mtk_phy_set_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);
	udelay(100);

	return 0;
}

static void mtk_mipi_tx_pll_cphy_disable(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);

	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);

	writel(MT6833_LANE_CON_BG, base + MIPITX_LANE_CON);
	writel(MT6833_LANE_CON_OFF, base + MIPITX_LANE_CON);
}

static int mtk_mipi_tx_pll_cphy_determine_rate(struct clk_hw *hw,
					       struct clk_rate_request *req)
{
	req->rate = clamp_val(req->rate, 125000000, 1600000000);

	return 0;
}

static const struct clk_ops mtk_mipi_tx_pll_cphy_ops = {
	.enable = mtk_mipi_tx_pll_cphy_enable,
	.disable = mtk_mipi_tx_pll_cphy_disable,
	.determine_rate = mtk_mipi_tx_pll_cphy_determine_rate,
	.set_rate = mtk_mipi_tx_pll_set_rate,
	.recalc_rate = mtk_mipi_tx_pll_recalc_rate,
};

static void mtk_mipi_tx_cphy_enable_signal(struct phy *phy)
{
	struct mtk_mipi_tx *mipi_tx = phy_get_drvdata(phy);
	void __iomem *base = mipi_tx->regs;

	/*
	 * Lane map for the straight (unswapped) wiring: T0..T2 each get their
	 * three wires, and PHY_SEL0 bit 0 selects C-PHY signalling for the pad
	 * mux.  These are the vendor's values for every C-PHY panel that does
	 * not set lane_swap_en, which includes both K16A modules.
	 */
	writel(0x65432101, base + MIPITX_PHY_SEL0);
	writel(0x24210987, base + MIPITX_PHY_SEL1);
	writel(0x68543102, base + MIPITX_PHY_SEL2);
	writel(0x00000007, base + MIPITX_PHY_SEL3);
}

static void mtk_mipi_tx_cphy_disable_signal(struct phy *phy)
{
	/*
	 * Nothing to undo here: the lane map survives a power cycle, and
	 * mtk_mipi_tx_pll_cphy_disable() has already put LANE_CON - which
	 * carries the C-PHY mode bit - back to its idle value.
	 */
}

const struct mtk_mipitx_data mt6833_mipitx_cphy_data = {
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_cphy_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_cphy_enable_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_cphy_disable_signal,
};
