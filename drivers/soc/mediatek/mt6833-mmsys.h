/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __SOC_MEDIATEK_MT6833_MMSYS_H
#define __SOC_MEDIATEK_MT6833_MMSYS_H

#define MT6833_DISP_OVL0_MOUT_EN	0xf18
#define MT6833_DISP_DSI0_SEL_IN		0xf30

#define MT6833_OVL0_MOUT_EN_RDMA0	BIT(0)
/*
 * the bootloader programs f30 (DSI0_SEL_IN) with value 1, i.e. the
 * DITHER0 input -- vendor headers define DITHER0 as 1 and RDMA0_RSZ0_SOUT
 * as 0.  Writing 0 killed the live stream on evergo; 1 matches
 * the bootloader and is verified on hardware.
 */
#define MT6833_DSI0_SEL_IN_RDMA0	0x1

static const struct mtk_mmsys_routes mmsys_mt6833_routing_table[] = {
	MMSYS_ROUTE(OVL, 0, RDMA, 0,
		    MT6833_DISP_OVL0_MOUT_EN, MT6833_OVL0_MOUT_EN_RDMA0,
		    MT6833_OVL0_MOUT_EN_RDMA0),
	MMSYS_ROUTE(RDMA, 0, DSI, 0,
		    MT6833_DISP_DSI0_SEL_IN, GENMASK(3, 0),
		    MT6833_DSI0_SEL_IN_RDMA0),
};

#endif /* __SOC_MEDIATEK_MT6833_MMSYS_H */