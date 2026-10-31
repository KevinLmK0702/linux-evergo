/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6833 (Dimensity 810) integrated CONN subsystem
 *
 * This is the public interface of the CONN ("connectivity") subsystem driver.
 * It is the seam between the SoC-level CONN hardware and the WLAN / Bluetooth
 * drivers: it owns power sequencing of the subsystem, the register windows the
 * radios poke, the chip-id handshake and the reserved-memory handed to the
 * firmware.
 *
 * Consumers must not touch the CONN register block before mt6833_conn_power_on()
 * has succeeded, and must call mt6833_conn_power_off() before dropping their
 * handle.  Both calls are reference counted, so WLAN and BT can be up at the same
 * time and the last user powers the block down.
 *
 * Register window indices are the DTS 'reg' order of the consys node; see
 * arch/arm64/boot/dts/mediatek/mt6833.dtsi and enum mt6833_conn_window below.
 */
#ifndef __LINUX_SOC_MEDIATEK_MT6833_CONN_H
#define __LINUX_SOC_MEDIATEK_MT6833_CONN_H

#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/types.h>

struct mt6833_conn;

/*
 * CONNSYS IP version, read from CONN_MCU_CFG + 0x10 (0x180b1010) once the
 * block is powered, and the only hardware identity this SoC offers.  The SoC
 * "chip id" the vendor driver reports (0x6833) is a compile-time constant.
 */
#define MT6833_CONN_IP_VER_ID		0x100a0000
#define MT6833_CONN_IP_VER_OFFSET	0x0010

/* DTS reg order of the consys node. */
enum mt6833_conn_window {
	MT6833_CONN_WIN_ON = 0,		/* 0x18002000 CONN_ON / PMU / top	*/
	MT6833_CONN_WIN_TOPRGU,		/* 0x10007000 TOP RGU (CPU reset)	*/
	MT6833_CONN_WIN_INFRA_AO,	/* 0x10001000 INFRA_AO bus protect	*/
	MT6833_CONN_WIN_SPM,		/* 0x10006000 SPM MTCMOS control	*/
	MT6833_CONN_WIN_OFF,		/* 0x18007000 CONN_OFF / WFSYS		*/
	MT6833_CONN_WIN_MCU_CFG,	/* 0x180b1000 CONN_MCU_CONFG		*/
	MT6833_CONN_WIN_MCU_CR,		/* 0x180a3000 CONN_MCU_CR		*/
	MT6833_CONN_WIN_MCU_PDA,	/* 0x180a5000 CONN_MCU_PDA		*/
	MT6833_CONN_WIN_MCU_DBG,	/* 0x180c1000 CONN_MCU_DBG		*/
	MT6833_CONN_WIN_MCU_EMI,	/* 0x18004000 CONN_MCU_EMI		*/
	MT6833_CONN_WIN_AP_CCIF4,	/* 0x1024c000 AP_CCIF4 mailbox		*/
	MT6833_CONN_WIN_PERICFG,	/* 0x10003000 PERICFG			*/
	MT6833_CONN_WIN_MAX,
};

/* Reserved-memory areas the CONN firmware and the WLAN DMA need. */
enum mt6833_conn_memory {
	/* consys node memory-region: CONNSYS EMI, 5 MB, 16 MB aligned */
	MT6833_CONN_MEM_FW = 0,
	/* wifi node memory-region: WLAN DMA / WFDMA descriptors, 6 MB */
	MT6833_CONN_MEM_WIFI,
};

/*
 * Handle management.  mt6833_conn_get() returns -EPROBE_DEFER from an ERR_PTR
 * while the consys node exists but this driver has not probed yet, and -ENODEV
 * if the board has no CONN node at all.  Each successful get must be paired
 * with exactly one mt6833_conn_put().
 */
struct mt6833_conn *mt6833_conn_get(struct device *consumer);
void mt6833_conn_put(struct mt6833_conn *conn);
struct device *mt6833_conn_dev(struct mt6833_conn *conn);

/*
 * Register window base, or NULL if that window was not described by the DTS
 * (the optional ones are INFRACFG_AO, SPM, MCU_PDA and AP_CCIF4 on some
 * board revisions).  Never dereference a NULL window.
 */
void __iomem *mt6833_conn_reg(struct mt6833_conn *conn,
			      enum mt6833_conn_window window);

/*
 * Power up/down the whole CONN subsystem (PMIC rails, CCIF4 clock, CONN MTCMOS
 * domain through the DT power domain, and the AP->CONN CCU handshake).
 * Reference counted: the first on does the work, the last off undoes it.
 * mt6833_conn_power_on() additionally verifies the chip id and returns -EIO if
 * the block did not answer.
 */
int mt6833_conn_power_on(struct mt6833_conn *conn);
void mt6833_conn_power_off(struct mt6833_conn *conn);

/* Number of power-on holders, 0 when the subsystem is off. */
unsigned int mt6833_conn_power_users(struct mt6833_conn *conn);

/* Powered *and* answering with the expected IP version. */
bool mt6833_conn_is_up(struct mt6833_conn *conn);

/* Raw read of CONN_MCU_CFG + 0x10.  Does not power anything up. */
int mt6833_conn_read_chip_id(struct mt6833_conn *conn, u32 *chip_id);

/* Convenience: is the value read back the MT6833 CONN IP version? */
static inline bool mt6833_conn_chip_id_valid(u32 chip_id)
{
	return chip_id == MT6833_CONN_IP_VER_ID;
}

/* Same, but reads the register.  False when the block does not answer. */
bool mt6833_conn_chip_id_ok(struct mt6833_conn *conn);

/*
 * Reserved memory.  mt6833_conn_memory() resolves the "memory-region" phandle of
 * the consys node (MT6833_CONN_MEM_FW) or of the wifi node
 * (MT6833_CONN_MEM_WIFI) and returns the base/size the bootloader reserved.
 * mt6833_conn_reserved_mem() does the same for an arbitrary consumer device, so
 * a radio can resolve its own memory-region without knowing about this driver.
 * Both return -ENOENT when the property or the reservation is missing; base and
 * size are left untouched in that case.
 */
int mt6833_conn_memory(struct mt6833_conn *conn, enum mt6833_conn_memory which,
		       phys_addr_t *base, size_t *size);
int mt6833_conn_reserved_mem(struct device *consumer, phys_addr_t *base,
			     size_t *size);

/*
 * HCLK/OSCCLK liveness check of the already powered block: the vendor dump path
 * writes 0x30000 to CONN_OFF and expects bits 16 and 17 back.  Returns 0 when
 * both clocks are running.  Diagnostic only, does not power anything.
 */
int mt6833_conn_check_clock(struct mt6833_conn *conn);

#endif /* __LINUX_SOC_MEDIATEK_MT6833_CONN_H */
