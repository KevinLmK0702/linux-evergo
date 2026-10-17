/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * MT6833 SPM power domain IDs.
 *
 * The numbering follows the SPM subsys order used by the vendor kernel
 * (drivers/clk/mediatek/clk-mt6833-pg.c, enum subsys_id) so that the
 * scpsys pg-clock IDs (SCP_SYS_*) later map 1:1 onto these.
 */

#ifndef _DT_BINDINGS_POWER_MEDIATEK_MT6833_POWER_H
#define _DT_BINDINGS_POWER_MEDIATEK_MT6833_POWER_H

#define MT6833_POWER_DOMAIN_MD1			0
#define MT6833_POWER_DOMAIN_CONN		1
#define MT6833_POWER_DOMAIN_MFG0		2
#define MT6833_POWER_DOMAIN_MFG1		3
#define MT6833_POWER_DOMAIN_MFG2		4
#define MT6833_POWER_DOMAIN_MFG3		5
#define MT6833_POWER_DOMAIN_ISP			6
#define MT6833_POWER_DOMAIN_ISP2		7
#define MT6833_POWER_DOMAIN_IPE			8
#define MT6833_POWER_DOMAIN_VDEC		9
#define MT6833_POWER_DOMAIN_VENC		10
#define MT6833_POWER_DOMAIN_DISP		11
#define MT6833_POWER_DOMAIN_AUDIO		12
#define MT6833_POWER_DOMAIN_CAM			13
#define MT6833_POWER_DOMAIN_CAM_RAWA		14
#define MT6833_POWER_DOMAIN_CAM_RAWB		15

#endif /* _DT_BINDINGS_POWER_MEDIATEK_MT6833_POWER_H */
