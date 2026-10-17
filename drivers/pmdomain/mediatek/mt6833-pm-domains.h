/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT6833 scpsys power domain data (direct SPM MTCMOS control).
 *
 * Register map and masks are taken from the vendor clk-mt6833-pg.c
 * (spm_mtcmos_ctrl_mfg*_pwr()):
 *
 *   PWR_STATUS      0x016c      PWR_STATUS_2ND  0x0170
 *   MFG0_PWR_CON    0x0308      (ASYNC)
 *   MFG1_PWR_CON    0x030c      (MFG_TOP)
 *   MFG2_PWR_CON    0x0310      (shader stack 0)
 *   MFG3_PWR_CON    0x0314      (shader stack 2)
 *
 * Only the four GPU domains are described for now; the remaining SPM
 * domains (DIS, ISP, ...) are currently left to the bootloader state
 * that the display/bring-up stack has always relied on.
 *
 * Not modelled yet: the MFG1 "way_en" step (INFRA_PDN + 0x004c,
 * bits 11/12) the vendor sequence performs around the TOP_AXI bus
 * protection steps below.
 */
#ifndef __SOC_MEDIATEK_MT6833_PM_DOMAINS_H
#define __SOC_MEDIATEK_MT6833_PM_DOMAINS_H

#include "mtk-pm-domains.h"
#include <dt-bindings/power/mediatek,mt6833-power.h>

#define MT6833_PWR_STA_OFFSET		0x016c
#define MT6833_PWR_STA2ND_OFFSET	0x0170

/* INFRA_TOPAXI bus protection (infracfg block) */
#define MT6833_TOP_AXI_PROT_EN_1_MFG1		BIT(21)
#define MT6833_TOP_AXI_PROT_EN_2_MFG1		GENMASK(6, 5)
#define MT6833_TOP_AXI_PROT_EN_MFG1		GENMASK(22, 21)
#define MT6833_TOP_AXI_PROT_EN_2_MFG1_2ND	BIT(7)

#define MT6833_TOP_AXI_PROTECTEN_SET		0x02a0
#define MT6833_TOP_AXI_PROTECTEN_CLR		0x02a4
#define MT6833_TOP_AXI_PROTECTEN_STA1		0x0228
#define MT6833_TOP_AXI_PROTECTEN_1_SET		0x02a8
#define MT6833_TOP_AXI_PROTECTEN_1_CLR		0x02ac
#define MT6833_TOP_AXI_PROTECTEN_1_STA1		0x0258
#define MT6833_TOP_AXI_PROTECTEN_2_SET		0x0714
#define MT6833_TOP_AXI_PROTECTEN_2_CLR		0x0718
#define MT6833_TOP_AXI_PROTECTEN_2_STA1		0x0724

static enum scpsys_bus_prot_block scpsys_bus_prot_blocks_mt6833[] = {
	BUS_PROT_BLOCK_INFRA
};

static const struct scpsys_domain_data scpsys_domain_data_mt6833[] = {
	[MT6833_POWER_DOMAIN_MFG0] = {
		.name = "mfg0",
		.sta_mask = BIT(2),
		.ctl_offs = 0x0308,
		.pwr_sta_offs = MT6833_PWR_STA_OFFSET,
		.pwr_sta2nd_offs = MT6833_PWR_STA2ND_OFFSET,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_DOMAIN_SUPPLY,
	},
	[MT6833_POWER_DOMAIN_MFG1] = {
		.name = "mfg1",
		.sta_mask = BIT(3),
		.ctl_offs = 0x030c,
		.pwr_sta_offs = MT6833_PWR_STA_OFFSET,
		.pwr_sta2nd_offs = MT6833_PWR_STA2ND_OFFSET,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.bp_cfg = {
			/* step1: TOP_AXI_PROTECTEN_1 bit 21 */
			BUS_PROT_WR(INFRA,
				    MT6833_TOP_AXI_PROT_EN_1_MFG1,
				    MT6833_TOP_AXI_PROTECTEN_1_SET,
				    MT6833_TOP_AXI_PROTECTEN_1_CLR,
				    MT6833_TOP_AXI_PROTECTEN_1_STA1),
			/* step1.1: TOP_AXI_PROTECTEN_2 bits 5-6 */
			BUS_PROT_WR(INFRA,
				    MT6833_TOP_AXI_PROT_EN_2_MFG1,
				    MT6833_TOP_AXI_PROTECTEN_2_SET,
				    MT6833_TOP_AXI_PROTECTEN_2_CLR,
				    MT6833_TOP_AXI_PROTECTEN_2_STA1),
			/* step2: TOP_AXI_PROTECTEN bits 21-22 */
			BUS_PROT_WR(INFRA,
				    MT6833_TOP_AXI_PROT_EN_MFG1,
				    MT6833_TOP_AXI_PROTECTEN_SET,
				    MT6833_TOP_AXI_PROTECTEN_CLR,
				    MT6833_TOP_AXI_PROTECTEN_STA1),
			/* step2.1: TOP_AXI_PROTECTEN_2 bit 7 */
			BUS_PROT_WR(INFRA,
				    MT6833_TOP_AXI_PROT_EN_2_MFG1_2ND,
				    MT6833_TOP_AXI_PROTECTEN_2_SET,
				    MT6833_TOP_AXI_PROTECTEN_2_CLR,
				    MT6833_TOP_AXI_PROTECTEN_2_STA1),
		},
		.caps = MTK_SCPD_DOMAIN_SUPPLY,
	},
	[MT6833_POWER_DOMAIN_MFG2] = {
		.name = "mfg2",
		.sta_mask = BIT(4),
		.ctl_offs = 0x0310,
		.pwr_sta_offs = MT6833_PWR_STA_OFFSET,
		.pwr_sta2nd_offs = MT6833_PWR_STA2ND_OFFSET,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
	[MT6833_POWER_DOMAIN_MFG3] = {
		.name = "mfg3",
		.sta_mask = BIT(5),
		.ctl_offs = 0x0314,
		.pwr_sta_offs = MT6833_PWR_STA_OFFSET,
		.pwr_sta2nd_offs = MT6833_PWR_STA2ND_OFFSET,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
};

static const struct scpsys_soc_data mt6833_scpsys_data = {
	.domains_data = scpsys_domain_data_mt6833,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt6833),
	.bus_prot_blocks = scpsys_bus_prot_blocks_mt6833,
	.num_bus_prot_blocks = ARRAY_SIZE(scpsys_bus_prot_blocks_mt6833),
};

#endif /* __SOC_MEDIATEK_MT6833_PM_DOMAINS_H */
