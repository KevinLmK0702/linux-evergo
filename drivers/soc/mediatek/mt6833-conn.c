// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6833 (Dimensity 810) integrated CONN subsystem driver
 *
 * The MT6833 has one CONN block that holds the CONNSYS MCU, the WiFi MAC/BT/GPS
 * modem and the CCIF4 mailbox between it and the AP.  It is a plain MTCMOS power
 * domain with a handful of AP-side registers; there is no firmware interface on
 * the AP side.  This driver owns that block so that the WLAN and Bluetooth
 * drivers do not each reimplement power sequencing:
 *
 *   - single instance, refcounted power on/off (WLAN and BT share the block)
 *   - the AP->CONN power handshake in PERICFG (CON_PWR_ON)
 *   - the chip-id ("is it really alive") check against 0x100a0000
 *   - register window accessors for the windows the radios need
 *   - the reserved-memory base/size of the CONNSYS EMI area and of the WiFi DMA
 *     pool
 *
 * What this driver deliberately does NOT do, because it is not needed to bring
 * the block up and was proven unnecessary by the bring-up probe on this board:
 *
 *   - EMI MPU programming: the bootloader (LK) sets the MPU.  The MT6895
 *     mainline port does the same and returns success from its EMI driver with
 *     "MPU is set in lk".  Consumers use of_reserved_mem_lookup() instead.
 *   - the infracfg_ao EMI remap windows (0x380 / 0x388 / 0x39c): those belong to
 *     the firmware loader, which is a separate piece of work.
 *   - ROM patch download and the 0x18002600 == 0x1d1e idle handshake: again the
 *     firmware loader.
 *   - clk_buf / XO control: not required on this board; the XO is on.
 *
 * The power sequence is the one the vendor driver uses in
 * connectivity/common/common_main/platform/mt6833.c, reduced to what mainline
 * models natively:
 *
 *   1. VCN18 (1.8 V) and VCN33_1 (3.3 V) rails, if the board describes them.
 *      Optional: LK leaves them on, so a board with no supply properties still
 *      powers up.
 *   2. clk "ccif" (infracfg_ao CLK_IFRAO_CCIF4_AP), the AP <-> CONN mailbox
 *      clock.
 *   3. the CONN MTCMOS domain, through the DT power domain
 *      MT6833_POWER_DOMAIN_CONN and pm_runtime; mainline scpsys does the
 *      PWR_ON/PWR_ON_2ND/ISO/RST_B sequence and the INFRA bus protection.
 *   4. the SPM power-on configuration clock (SPM + 0x000, bit 0 with key 0x0b16),
 *      without which SPM writes may be dropped.
 *   5. CON_PWR_ON: write 1 to PERICFG + 0x314, the AP2MD CCIF4 CCU config.  This
 *      is the one step with no mainline analogue and the one most easily missed.
 *   6. poll CONN_MCU_CFG + 0x10 (0x180b1010) for 0x100a0000, 10 x 20 ms.
 *
 * Power-off mirrors it: clear CON_PWR_ON, drop the domain, clear the CCIF4 ACK
 * (AP_CCIF4 + 0x14 = 0xff), then the clock and the rails.
 *
 * Four of the twelve windows are also claimed by other nodes (infracfg_ao,
 * pericfg, spm, the watchdog), so this uses of_iomap() and not
 * devm_ioremap_resource(): the latter would fail with -EBUSY.  The vendor driver
 * does raw ioremap for exactly the same reason.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of_irq.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/moduleparam.h>
#include <linux/soc/mediatek/mt6833-conn.h>

/*
 * A-die (external RF companion) identification over the SoC's CONN RF SPI.
 * mt6833.c:1807-1900.
 */
#define CONN_ON_ADIE_CTL_OFFSET		0x0500
#define CONN_RF_SPI_BASE		0x180c6000
#define CONN_SPI_TOP_ADDR		0x50
#define CONN_SPI_TOP_WDAT		0x54
#define CONN_SPI_TOP_RDAT		0x58
#define CONN_ADIE_RD_ADDR_PRIMARY	0x0000b024
#define CONN_ADIE_RD_ADDR_SECONDARY	0x0000b02C
#define CONN_ADIE_PRIMARY		0x6631

u32 mt6833_conn_adie_id __read_mostly;
EXPORT_SYMBOL_GPL(mt6833_conn_adie_id);

/* The WiFi reserved-memory window, cached for the WLAN driver's EMI copy. */
phys_addr_t conn_emi_base_cached;
u32 conn_emi_size_cached;
EXPORT_SYMBOL_GPL(conn_emi_base_cached);
EXPORT_SYMBOL_GPL(conn_emi_size_cached);

phys_addr_t mt6833_conn_emi_base(void)
{
	return conn_emi_base_cached;
}
EXPORT_SYMBOL_GPL(mt6833_conn_emi_base);

u32 mt6833_conn_emi_size(void)
{
	return conn_emi_size_cached;
}
EXPORT_SYMBOL_GPL(mt6833_conn_emi_size);

/* Plain 32-bit read of an AP-side address, for drivers that cannot use the
 * inband register command. */
u32 mt6833_conn_read32(phys_addr_t addr)
{
	void __iomem *p = ioremap(addr, 4);
	u32 v;

	if (!p)
		return 0;
	v = readl(p);
	iounmap(p);
	return v;
}
EXPORT_SYMBOL_GPL(mt6833_conn_read32);

/*
 * CONN AXI bus protection, from spm_mtcmos_ctrl_conn_bus_prot()
 * (clk-mt6833-pg.c:1045-1124).  This is the "prepare" half of the vendor's
 * conn-clock enable and was never done by this driver.
 */
#define CONN_PROT_STEP1_0_MASK		(BIT(13) | BIT(18))
#define CONN_PROT_STEP2_0_MASK		BIT(14)
#define CONN_PROT_STEP2_1_MASK		BIT(10)
#define CONN_INFRA_PROTECTEN		0x0220
#define CONN_INFRA_PROTECTEN_1		0x0250
#define CONN_INFRA_PROTECTEN_CLR	0x02a4
#define CONN_INFRA_PROTECTEN_1_CLR	0x02ac

/*
 * CONN MTCMOS domain control, from the vendor clk-mt6833-pg.c:1126-1191.
 * This driver has to do it by hand because mainline ships no MT6833 power
 * domain driver, so pm_runtime for MT6833_POWER_DOMAIN_CONN does nothing.
 */
#define CONN_SPM_PWR_CON		0x0304
#define CONN_SPM_PWR_STATUS		0x016c
#define CONN_SPM_PWR_STATUS_2ND		0x0170
#define CONN_SPM_STA_MASK		BIT(1)
#define CONN_SPM_PWR_RST_B		BIT(0)
#define CONN_SPM_PWR_ISO		BIT(1)
#define CONN_SPM_PWR_ON			BIT(2)
#define CONN_SPM_PWR_ON_2ND		BIT(3)
#define CONN_SPM_PWR_CLK_DIS		BIT(4)

/*
 * Remaining vendor reg_ctrl() steps.  Sources:
 *   mt6833.c:486   consys_set_if_pinmux()
 *   mt6833.c:596   consys_hw_spm_clk_gating_enable()
 *   mt6833.c:999   consys_set_access_emi_hw_mode()
 *   mt6833.c:1120  consys_identify_adie()
 *   mt6833.c:1189  consys_bus_timeout_config()
 */
#define CONN_IF_PINMUX_BASE		0x10005000
#define CONN_IF_PINMUX_01_OFFSET	0x0480
#define CONN_IF_PINMUX_01_MASK		0x88888888
#define CONN_IF_PINMUX_01_VALUE		0x11111111
#define CONN_IF_PINMUX_02_OFFSET	0x0490
#define CONN_IF_PINMUX_02_MASK		0xFFFFFFF8
#define CONN_IF_PINMUX_02_VALUE		0x00000001
#define CONN_IF_PINMUX_DRV_BASE		0x11EA0000
#define CONN_IF_PINMUX_DRV_VALUE_1	0x09048000
#define CONN_IF_PINMUX_DRV_MASK_1	0xC0007FFF
#define CONN_IF_PINMUX_DRV_VALUE_2	0x00000000
#define CONN_IF_PINMUX_DRV_MASK_2	0xFFFFF000

#define CONN_SPM_PWR_ON_CLK		(0x0B16 << 16 | BIT(0))
#define CONN_AHB_TIMEOUT_ADDR_OFFSET	0x0440
#define CONN_AHB_TIMEOUT_VALUE		0x90000002

/* mcu_top_misc_on_base, proven by 0x180C1130 == base + 0x130. */
#define CONN_TOP_MISC_ON_OFFSET		0x0130
#define CONN_IDENTIFY_ADIE_ENABLE	BIT(8)
#define CONN_ADIE_SECONDARY		0x6635

/*
 * Vendor steps this driver used to skip.  Offsets are from the vendor source:
 *   mt6833.c:1020  consys_emi_entry_address()
 *   mt6833.c:999   consys_set_access_emi_hw_mode()
 *   mt6833.c:1032  consys_set_xo_osc_ctrl()
 * with MCU_BASE and MCU_TOP_MISC_ON both inside the CONN_ON window.
 */
#define CONN_EMI_BT_ENTRY_OFFSET	0x0504
#define CONN_EMI_WIFI_ENTRY_OFFSET	0x0508
#define CONN_EMI_BT_ENTRY_ADDRESS	0xF0170000
#define CONN_EMI_WIFI_ENTRY_ADDRESS	0xF02A0000
#define CONN_ACCESS_EMI_HW_MODE_OFFSET	0x0168
#define CONN_COCLOCK_RC_CTL_0_OFFSET	0x0040
#define CONN_COCLOCK_TOP_OSC_RC_EN	BIT(7)

/*
 * WFSYS power-on registers, as used by the working ports.
 *   0x18000000  WFSYS_ON_TOP_PWR_CTL   key 0x5746 in [31:16], PWR_ON bit 7
 *   0x18000110  WFSYS_CPU area (dumped by the vendor on failure)
 *   0x18000120  WFSYS_CPU_SW_RST_B     bit 0 = reset release
 *   0x180602CC  CONNSYS_PWR_STATUS     bit 2 = wfsys rgu off released
 *   0x18060B10  WF_ON_MONFLG_OUT       0x1D1E = CONN CPU in ROM idle loop
 */
#define CONN_WF_ON_TOP_PWR_CTL		0x18000000
#define CONN_WF_CPU_RST_B		0x18000120
#define CONN_WF_CPU_RST_B_ALT		0x0010
#define CONN_WF_PWR_STATUS		0x180602CC
#define CONN_WF_MONFLG_OUT		0x18060B10
#define CONN_WF_PWR_CTL_KEY		0x57460080
#define CONN_WF_PWR_CTL_CLR		0x0000FF7F
#define CONN_WF_PWR_STATUS_RDY		BIT(2)
#define CONN_WF_ROM_IDLE		0x1D1E

/*
 * CONN EMI remap, INFRA_AO relative (vendor names them on topckgen_base, which
 * on mt6833 is INFRA_AO 0x10001000).  Bits [19:0] take address bits [35:16].
 */
#define CONN_EMI_MAPPING_OFFSET		0x0380
#define CONN_EMI_PERI_MAPPING_OFFSET	0x0388
#define CONN_EMI_MAP_MASK		0x000FFFFF
#define CONN_EMI_PERI_BASE		0x10003000ULL

/*
 * AP_RGU: CONN CPU software reset.  Vendor: platform/mt6833.c:553-566 writes
 * AP_RGU + 0x18 with CONSYS_CPU_SW_RST_BIT (bit 12) and
 * CONSYS_CPU_SW_RST_CTRL_KEY (0x88 in [31:24]) on every write.
 */
#define CONN_AP_RGU_CPU_SW_RST_OFFSET	0x0018
#define CONN_CPU_SW_RST_BIT		BIT(12)
/* Whole-CONNSYS reset; vendor clears this too (mt6833.c:707-709). */
#define CONN_SYS_SW_RST_BIT		BIT(9)
#define CONN_CPU_SW_RST_CTRL_KEY	(0x88 << 24)

/*
 * INFRA_AO AP2CONN / CONN2AP bus sleep protection.  Offsets and masks copied
 * from the vendor's platform/mt6833.c (CONSYS_AXI_RX_PROT_* / AHB_RX_PROT_*),
 * which targets the same INFRA_AO window this driver maps.
 */
#define CONN_INFRA_AXI_PROT_EN_CLR	0x02A4
#define CONN_INFRA_AXI_PROT_STA		0x0228
#define CONN_INFRA_AHB_PROT_EN_CLR	0x02AC
#define CONN_INFRA_AHB_PROT_STA		0x0258
#define CONN_PROT_AXI_RX		BIT(14)
#define CONN_PROT_AXI_TX		BIT(18)
#define CONN_PROT_AHB_TX		BIT(13)
#define CONN_PROT_AHB_RX		BIT(10)

/*
 * WFSYS window: 0x1800_0000.  Only the CONN_ON/CONN_OFF slices are mapped by
 * this driver, so the WiFi block is mapped separately where it is needed.
 */
#define CONN_WFSYS_BASE			0x18000000
#define CONN_WFSYS_MAP_SIZE		0x00800000
#define CONN_WFSYS_CPU_SW_RST_OFFSET	0x0120
#define CONN_WFSYS_IDLE_MONFLG_OFFSET	0x60B10
#define CONN_WFSYS_CPU_SW_RST_BIT	BIT(0)

/*
 * CONN MCU ROM patch.  Struct wmt_rom_patch is 32 bytes and is stripped before
 * the body is copied into EMI (wmt_ic_soc.c:3592-3598).
 */
#define CONN_ROM_PATCH_HDR_LEN	32
#define CONN_ROM_PATCH_MCU_SECONDARY	"soc2_2_ram_mcu_1a_1_hdr.bin"
#define CONN_ROM_PATCH_MCU_PRIMARY	"soc2_2_ram_mcu_1_1_hdr.bin"
#define CONN_ROM_PATCH_BT_SECONDARY	"soc2_2_ram_bt_1a_1_hdr.bin"
#define CONN_ROM_PATCH_BT_PRIMARY	"soc2_2_ram_bt_1_1_hdr.bin"
#define CONN_ROM_PATCH_WIFI_SECONDARY	"soc2_2_ram_wifi_1a_1_hdr.bin"
#define CONN_ROM_PATCH_WIFI_PRIMARY	"soc2_2_ram_wifi_1_1_hdr.bin"

/* EMI offsets, verbatim from the working vendor kernel. */
/* Vendor: emi_phy_addr 0xf0088000 -> AP offset 0x88000. */
#define CONN_ROM_PATCH_MCU_OFFSET	0x000000
#define CONN_ROM_PATCH_BT_OFFSET	0x170000
#define CONN_ROM_PATCH_WIFI_OFFSET	0x2a0000

static unsigned int rom_patch_emi_offset;
module_param(rom_patch_emi_offset, uint, 0644);
MODULE_PARM_DESC(rom_patch_emi_offset,
		 "EMI offset for the CONN MCU ROM patch (vendor gets it from userspace)");

/* PERICFG: AP2MD PCCIF4 AP CCU config, bit 0 CON_PWR_ON, bit 1 CON_SW_READY. */
#define CONN_PERICFG_CON_STA_OFFSET	0x0314
#define CONN_PERI_CON_PWR_ON		BIT(0)

/* CONN_MCU_CFG: CONNSYS IP version. */
#define CONN_CHIP_ID_OFFSET		0x0010
#define CONN_CHIP_ID_RETRIES		10
#define CONN_CHIP_ID_DELAY_MS		20

/* AP_CCIF4: PCCIF ACK, cleared to 0xff on power-off. */
#define CONN_AP_CCIF4_ACK_OFFSET	0x0014
#define CONN_AP_CCIF4_ACK_CLEAR		0xff

/* SPM: power-on configuration clock enable, key 0x0b16 in the upper half. */
#define CONN_SPM_PWRON_CFG_OFFSET	0x0000
#define CONN_SPM_PWRON_CFG_EN		(BIT(0) | (0x0b16 << 16))

/* CONN_OFF: writing 0x30000 makes the block report HCLK/OSCCLK liveness. */
#define CONN_OFF_CLOCK_CHECK_OFFSET	0x0000
#define CONN_OFF_CLOCK_CHECK_VALUE	(BIT(16) | BIT(17))

/* SPM power status, for failure diagnostics only. */
#define CONN_SPM_PWR_STATUS_OFFSET	0x016c
#define CONN_SPM_PWR_STATUS_2ND_OFFSET	0x0170
#define CONN_SPM_PWR_STA_MASK		BIT(1)

/* Vendor settles for 5 ms after CON_PWR_ON before the chip id read. */
#define CONN_PWR_ON_SETTLE_MS		5

/* Vendor waits 100 us after dropping the domain before clearing the CCIF ACK. */
#define CONN_PWR_OFF_SETTLE_US		100

#define CONN_MTCMOS_TIMEOUT_MS		10

struct mt6833_conn {
	struct device *dev;
	void __iomem *win[MT6833_CONN_WIN_MAX];
	struct clk *ccif;
	struct regulator *vcn18;
	struct regulator *vcn33_1;
	struct regulator *vcn33_2_wifi;
	struct mutex lock;		/* serialises power on/off and remove */
	unsigned int power_users;
	bool powered;
	bool dead;			/* driver removed, API returns -ENODEV */
	u32 chip_id;			/* last value read from 0x180b1010 */
	struct kref ref;
};

static struct mt6833_conn *mt6833_conn_instance;
static DEFINE_MUTEX(mt6833_conn_instance_lock);

static bool conn_window_required(enum mt6833_conn_window win)
{
	switch (win) {
	case MT6833_CONN_WIN_ON:
	case MT6833_CONN_WIN_OFF:
	case MT6833_CONN_WIN_MCU_CFG:
	case MT6833_CONN_WIN_MCU_CR:
	case MT6833_CONN_WIN_MCU_EMI:
	case MT6833_CONN_WIN_PERICFG:
		return true;
	default:
		return false;
	}
}

static const char *conn_window_name(enum mt6833_conn_window win)
{
	static const char *const names[MT6833_CONN_WIN_MAX] = {
		[MT6833_CONN_WIN_ON]		= "conn_on",
		[MT6833_CONN_WIN_TOPRGU]	= "toprgu",
		[MT6833_CONN_WIN_INFRA_AO]	= "infracfg_ao",
		[MT6833_CONN_WIN_SPM]		= "spm",
		[MT6833_CONN_WIN_OFF]		= "conn_off",
		[MT6833_CONN_WIN_MCU_CFG]	= "mcu_cfg",
		[MT6833_CONN_WIN_MCU_CR]	= "mcu_cr",
		[MT6833_CONN_WIN_MCU_PDA]	= "mcu_pda",
		[MT6833_CONN_WIN_MCU_DBG]	= "mcu_dbg",
		[MT6833_CONN_WIN_MCU_EMI]	= "mcu_emi",
		[MT6833_CONN_WIN_AP_CCIF4]	= "ap_ccif4",
		[MT6833_CONN_WIN_PERICFG]	= "pericfg",
	};

	return names[win];
}

static void mt6833_conn_unmap_all(struct mt6833_conn *conn)
{
	unsigned int i;

	for (i = 0; i < MT6833_CONN_WIN_MAX; i++) {
		if (conn->win[i]) {
			iounmap(conn->win[i]);
			conn->win[i] = NULL;
		}
	}
}

static void mt6833_conn_release(struct kref *ref)
{
	struct mt6833_conn *conn = container_of(ref, struct mt6833_conn, ref);

	mt6833_conn_unmap_all(conn);
	kfree(conn);
}

/* ------------------------------------------------------------------------- */
/* Register access							     */
/* ------------------------------------------------------------------------- */

void __iomem *mt6833_conn_reg(struct mt6833_conn *conn,
			      enum mt6833_conn_window window)
{
	if (!conn || window >= MT6833_CONN_WIN_MAX)
		return NULL;

	return conn->win[window];
}
EXPORT_SYMBOL_GPL(mt6833_conn_reg);

struct device *mt6833_conn_dev(struct mt6833_conn *conn)
{
	return conn ? conn->dev : NULL;
}
EXPORT_SYMBOL_GPL(mt6833_conn_dev);

struct mt6833_conn *mt6833_conn_get(struct device *consumer)
{
	struct mt6833_conn *conn;
	struct device_node *np;

	if (!consumer)
		return ERR_PTR(-EINVAL);

	mutex_lock(&mt6833_conn_instance_lock);
	conn = mt6833_conn_instance;
	if (!conn) {
		/*
		 * Distinguish "this board has a CONN node, the driver is just not
		 * there yet" from "this board has no CONN at all": the former must
		 * defer, the latter must not spin forever.
		 */
		np = of_find_compatible_node(NULL, NULL, "mediatek,mt6833-consys");
		mutex_unlock(&mt6833_conn_instance_lock);
		if (!np)
			return ERR_PTR(-ENODEV);

		of_node_put(np);
		return ERR_PTR(-EPROBE_DEFER);
	}

	if (conn->dead) {
		mutex_unlock(&mt6833_conn_instance_lock);
		return ERR_PTR(-ENODEV);
	}

	kref_get(&conn->ref);
	mutex_unlock(&mt6833_conn_instance_lock);

	dev_dbg(consumer, "took a reference on the MT6833 CONN subsystem\n");

	return conn;
}
EXPORT_SYMBOL_GPL(mt6833_conn_get);

void mt6833_conn_put(struct mt6833_conn *conn)
{
	if (IS_ERR_OR_NULL(conn))
		return;

	kref_put(&conn->ref, mt6833_conn_release);
}
EXPORT_SYMBOL_GPL(mt6833_conn_put);

/* ------------------------------------------------------------------------- */
/* Chip id								     */
/* ------------------------------------------------------------------------- */

int mt6833_conn_read_chip_id(struct mt6833_conn *conn, u32 *chip_id)
{
	u32 val;

	if (!conn || !conn->win[MT6833_CONN_WIN_MCU_CFG])
		return -ENODEV;

	val = readl(conn->win[MT6833_CONN_WIN_MCU_CFG] + CONN_CHIP_ID_OFFSET);
	if (chip_id)
		*chip_id = val;

	return 0;
}
EXPORT_SYMBOL_GPL(mt6833_conn_read_chip_id);

bool mt6833_conn_chip_id_ok(struct mt6833_conn *conn)
{
	u32 val;

	if (mt6833_conn_read_chip_id(conn, &val))
		return false;

	return mt6833_conn_chip_id_valid(val);
}
EXPORT_SYMBOL_GPL(mt6833_conn_chip_id_ok);

static int conn_poll_chip_id(struct mt6833_conn *conn)
{
	unsigned int i;
	u32 val = 0;

	for (i = 0; i < CONN_CHIP_ID_RETRIES; i++) {
		val = readl(conn->win[MT6833_CONN_WIN_MCU_CFG] + CONN_CHIP_ID_OFFSET);
		if (mt6833_conn_chip_id_valid(val))
			break;
		msleep(CONN_CHIP_ID_DELAY_MS);
	}

	conn->chip_id = val;

	if (!mt6833_conn_chip_id_valid(val)) {
		dev_err(conn->dev,
			"CONN did not answer: chip id %#010x, expected %#010x after %u tries\n",
			val, MT6833_CONN_IP_VER_ID, CONN_CHIP_ID_RETRIES);
		return -EIO;
	}

	dev_info(conn->dev, "CONN up, chip id %#010x after %u tries\n",
		 val, i + 1);

	return 0;
}

/* ------------------------------------------------------------------------- */
/* Diagnostics								     */
/* ------------------------------------------------------------------------- */

static void conn_dump_state(struct mt6833_conn *conn)
{
	u32 hw_id = 0, fw_id = 0;
	u32 status = 0, status2 = 0;

	if (conn->win[MT6833_CONN_WIN_ON]) {
		hw_id = readl(conn->win[MT6833_CONN_WIN_ON] + 0x0000);
		fw_id = readl(conn->win[MT6833_CONN_WIN_ON] + 0x0004);
	}

	if (conn->win[MT6833_CONN_WIN_SPM]) {
		status = readl(conn->win[MT6833_CONN_WIN_SPM] +
			       CONN_SPM_PWR_STATUS_OFFSET);
		status2 = readl(conn->win[MT6833_CONN_WIN_SPM] +
				CONN_SPM_PWR_STATUS_2ND_OFFSET);
	}

	dev_err(conn->dev,
		"CONN state: conn_on[hw %#06x fw %#06x] conn_off %#010x mcu_cr %#010x mcu_emi %#010x\n",
		hw_id & 0xffff, fw_id & 0xffff,
		conn->win[MT6833_CONN_WIN_OFF] ?
			readl(conn->win[MT6833_CONN_WIN_OFF]) : 0,
		conn->win[MT6833_CONN_WIN_MCU_CR] ?
			readl(conn->win[MT6833_CONN_WIN_MCU_CR]) : 0,
		conn->win[MT6833_CONN_WIN_MCU_EMI] ?
			readl(conn->win[MT6833_CONN_WIN_MCU_EMI]) : 0);

	if (conn->win[MT6833_CONN_WIN_SPM])
		dev_err(conn->dev,
			"CONN state: spm pwr_status %#010x (conn bit %u) pwr_status_2nd %#010x (conn bit %u)\n",
			status, !!(status & CONN_SPM_PWR_STA_MASK),
			status2, !!(status2 & CONN_SPM_PWR_STA_MASK));
}

int mt6833_conn_check_clock(struct mt6833_conn *conn)
{
	u32 val, saved;

	if (!conn || !conn->win[MT6833_CONN_WIN_OFF])
		return -ENODEV;

	if (!READ_ONCE(conn->powered))
		return -ENODEV;

	/*
	 * Diagnostic write: CONN_OFF must be restored afterwards.  Leaving the
	 * check value behind makes the next power-on read the chip id as
	 * 0x00030001 (measured), i.e. the block stays in clock-test state.
	 */
	saved = readl(conn->win[MT6833_CONN_WIN_OFF] +
		      CONN_OFF_CLOCK_CHECK_OFFSET);
	writel(CONN_OFF_CLOCK_CHECK_VALUE,
	       conn->win[MT6833_CONN_WIN_OFF] + CONN_OFF_CLOCK_CHECK_OFFSET);
	udelay(200);
	val = readl(conn->win[MT6833_CONN_WIN_OFF] + CONN_OFF_CLOCK_CHECK_OFFSET);
	writel(saved, conn->win[MT6833_CONN_WIN_OFF] +
	       CONN_OFF_CLOCK_CHECK_OFFSET);

	if ((val & CONN_OFF_CLOCK_CHECK_VALUE) != CONN_OFF_CLOCK_CHECK_VALUE) {
		dev_warn(conn->dev,
			 "CONN clocks: hclk %d osclk %d (conn_off %#010x)\n",
			 !!(val & BIT(16)), !!(val & BIT(17)), val);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mt6833_conn_check_clock);

/* ------------------------------------------------------------------------- */
/* Reserved memory							     */
/* ------------------------------------------------------------------------- */

/* The vendor puts the CONN shared memory at 0xbe000000 and its MCU boots
 * expecting that address; the packaged DTB allocates it dynamically.
 */
static unsigned long emi_base_fixed;
module_param(emi_base_fixed, ulong, 0644);
MODULE_PARM_DESC(emi_base_fixed, "physical base of the CONN EMI window (0 = use device tree)");

static int conn_reserved_mem_of(struct device_node *np, phys_addr_t *base,
				size_t *size)
{
	struct reserved_mem *rmem;
	struct device_node *rmem_np;

	if (!np)
		return -ENODEV;

	rmem_np = of_parse_phandle(np, "memory-region", 0);
	if (!rmem_np)
		return -ENOENT;

	rmem = of_reserved_mem_lookup(rmem_np);
	of_node_put(rmem_np);
	if (!rmem)
		return -ENOENT;

	if (base)
		*base = emi_base_fixed ? (phys_addr_t)emi_base_fixed : rmem->base;
	if (size)
		*size = rmem->size;

	return 0;
}

int mt6833_conn_reserved_mem(struct device *consumer, phys_addr_t *base,
			     size_t *size)
{
	if (!consumer)
		return -EINVAL;

	return conn_reserved_mem_of(consumer->of_node, base, size);
}
EXPORT_SYMBOL_GPL(mt6833_conn_reserved_mem);

int mt6833_conn_memory(struct mt6833_conn *conn, enum mt6833_conn_memory which,
		       phys_addr_t *base, size_t *size)
{
	struct device_node *np;
	int ret;

	if (!conn)
		return -EINVAL;

	switch (which) {
	case MT6833_CONN_MEM_FW:
		return conn_reserved_mem_of(conn->dev->of_node, base, size);
	case MT6833_CONN_MEM_WIFI:
		np = of_find_compatible_node(NULL, NULL, "mediatek,wifi");
		if (!np)
			return -ENODEV;
		ret = conn_reserved_mem_of(np, base, size);
		of_node_put(np);
		return ret;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(mt6833_conn_memory);

/* ------------------------------------------------------------------------- */
/* Power								     */
/* ------------------------------------------------------------------------- */

/*
 * MTCMOS power on/off for the CONN domain.  `on` follows the vendor sequence
 * step by step, including the two status waits; a single combined write is not
 * equivalent because the domain asserts its ACK between the steps.
 */
/*
 * Release the CONN AXI bus protection.  Vendor order: step 2 (second bank),
 * step 2 (first bank), then step 1.
 */
static void conn_release_bus_protect(struct mt6833_conn *conn)
{
	void __iomem *infra = conn->win[MT6833_CONN_WIN_INFRA_AO];

	if (!infra) {
		dev_warn(conn->dev, "bus protect: no INFRA_AO window\n");
		return;
	}

	dev_info(conn->dev,
		 "bus protect: before PROTECTEN=%#010x PROTECTEN_1=%#010x\n",
		 readl(infra + CONN_INFRA_PROTECTEN),
		 readl(infra + CONN_INFRA_PROTECTEN_1));

	writel(CONN_PROT_STEP2_1_MASK, infra + CONN_INFRA_PROTECTEN_1_CLR);
	writel(CONN_PROT_STEP2_0_MASK, infra + CONN_INFRA_PROTECTEN_CLR);
	writel(CONN_PROT_STEP1_0_MASK, infra + CONN_INFRA_PROTECTEN_CLR);
	udelay(1);

	dev_info(conn->dev,
		 "bus protect: after  PROTECTEN=%#010x PROTECTEN_1=%#010x\n",
		 readl(infra + CONN_INFRA_PROTECTEN),
		 readl(infra + CONN_INFRA_PROTECTEN_1));
}

static int conn_mtcmos_set(struct mt6833_conn *conn, bool on)
{
	void __iomem *spm = conn->win[MT6833_CONN_WIN_SPM];
	u32 v;
	int i;

	if (!spm)
		return -ENODEV;

	if (!on) {
		v = readl(spm + CONN_SPM_PWR_CON);
		writel(v | CONN_SPM_PWR_ISO, spm + CONN_SPM_PWR_CON);
		v = readl(spm + CONN_SPM_PWR_CON);
		writel(v | CONN_SPM_PWR_CLK_DIS, spm + CONN_SPM_PWR_CON);
		v = readl(spm + CONN_SPM_PWR_CON);
		writel(v & ~CONN_SPM_PWR_RST_B, spm + CONN_SPM_PWR_CON);
		v = readl(spm + CONN_SPM_PWR_CON);
		writel(v & ~CONN_SPM_PWR_ON, spm + CONN_SPM_PWR_CON);
		v = readl(spm + CONN_SPM_PWR_CON);
		writel(v & ~CONN_SPM_PWR_ON_2ND, spm + CONN_SPM_PWR_CON);
		return 0;
	}

	/* PWR_ON */
	v = readl(spm + CONN_SPM_PWR_CON);
	writel(v | CONN_SPM_PWR_ON, spm + CONN_SPM_PWR_CON);
	/* PWR_ON_2ND */
	v = readl(spm + CONN_SPM_PWR_CON);
	writel(v | CONN_SPM_PWR_ON_2ND, spm + CONN_SPM_PWR_CON);

	/* Wait for both status registers to acknowledge. */
	for (i = 0; i < 100; i++) {
		u32 sta = readl(spm + CONN_SPM_PWR_STATUS);
		u32 sta2 = readl(spm + CONN_SPM_PWR_STATUS_2ND);

		if ((sta & CONN_SPM_STA_MASK) && (sta2 & CONN_SPM_STA_MASK))
			break;
		udelay(100);
	}
	if (i == 100) {
		dev_err(conn->dev,
			"MTCMOS: CONN did not acknowledge (status %#010x / %#010x)\n",
			readl(spm + CONN_SPM_PWR_STATUS),
			readl(spm + CONN_SPM_PWR_STATUS_2ND));
		return -ETIMEDOUT;
	}

	/* release clock gating, isolation, then the domain reset */
	v = readl(spm + CONN_SPM_PWR_CON);
	writel(v & ~CONN_SPM_PWR_CLK_DIS, spm + CONN_SPM_PWR_CON);
	v = readl(spm + CONN_SPM_PWR_CON);
	writel(v & ~CONN_SPM_PWR_ISO, spm + CONN_SPM_PWR_CON);
	v = readl(spm + CONN_SPM_PWR_CON);
	writel(v | CONN_SPM_PWR_RST_B, spm + CONN_SPM_PWR_CON);

	dev_info(conn->dev, "MTCMOS: CONN on, PWR_CON %#010x status %#010x/%#010x\n",
		 readl(spm + CONN_SPM_PWR_CON),
		 readl(spm + CONN_SPM_PWR_STATUS),
		 readl(spm + CONN_SPM_PWR_STATUS_2ND));
	return 0;
}

static void conn_snapshot_early(struct mt6833_conn *conn, const char *when);
static void conn_probe_hif_base(struct mt6833_conn *conn);
static void conn_topckgen_setup(void);

/*
 * AP2CONN hclk parent select: CLK_CFG_20_CLR=0xF, CLK_CFG_20_SET=0x1,
 * CLK_CFG_UPDATE2=0x40000.  Enabled by default; without it the AP2CONN bus
 * does not clock and the CONN registers read as 0xdeadfeed.
 */
static int minimal = 0;
module_param(minimal, int, 0644);
MODULE_PARM_DESC(minimal, "only MTCMOS + CON_PWR_ON + chip id + reset + idle poll");
static int emi_use_wifi = 0;
module_param(emi_use_wifi, int, 0644);
MODULE_PARM_DESC(emi_use_wifi, "1 = use the WLAN reservation (be000000), 0 = the CONN one (bf000000)");
static int no_clear = 1;
module_param(no_clear, int, 0644);
MODULE_PARM_DESC(no_clear, "skip the EMI wipe (default 0)");
static int patch_hdr = 48;
module_param(patch_hdr, int, 0644);
MODULE_PARM_DESC(patch_hdr, "header bytes stripped from a patch blob (default 32)");
/* installed by the WMT shim (a module) so userspace can re-run the probe */
void (*conn_replay_probe_hook)(void);
EXPORT_SYMBOL_GPL(conn_replay_probe_hook);
static int stp_before_probe = 0;
module_param(stp_before_probe, int, 0644);
MODULE_PARM_DESC(stp_before_probe, "run the WMT/STP sequence before replaying the probe");
static int replay_probe;
module_param(replay_probe, int, 0644);
MODULE_PARM_DESC(replay_probe, "re-run the WLAN probe after each power cycle");
static unsigned int infra_cg_extra = 0x88800000;
module_param(infra_cg_extra, uint, 0644);
MODULE_PARM_DESC(infra_cg_extra, "bits OR-ed into INFRA_AO+0x80 at power-on");
static int idle_poll_ms = 400;
module_param(idle_poll_ms, int, 0644);
MODULE_PARM_DESC(idle_poll_ms, "ms to wait for the ROM idle loop (default 400)");
static int patch_mode;
module_param(patch_mode, int, 0644);
MODULE_PARM_DESC(patch_mode, "0=all, 1=MCU only, 2=BT+WiFi only, 3=none");
static int steps = 3;
module_param(steps, int, 0644);
MODULE_PARM_DESC(steps, "bitmask of extra steps to add back in minimal mode: "
		 "1=EMI remap, 2=ROM patch, 4=vendor prep, 8=sleep-protect+WFSYS probe");
#define STEP_EMI_REMAP	BIT(0)
#define STEP_ROM_PATCH	BIT(1)
#define STEP_VENDOR	BIT(2)
#define STEP_PROBE	BIT(3)
static int wf_pwr_ctl_write = 0;
module_param(wf_pwr_ctl_write, int, 0644);
MODULE_PARM_DESC(wf_pwr_ctl_write, "write the MT6895 WFSYS pwr_ctl key (default 0)");
static int wf_cpu_rst_write = 0;
module_param(wf_cpu_rst_write, int, 0644);
MODULE_PARM_DESC(wf_cpu_rst_write, "write the MT6895 WFSYS CPU reset bit (default 0)");
static int power_cycle = 1;
module_param(power_cycle, int, 0644);
MODULE_PARM_DESC(power_cycle,
		 "power the CONN domain down before powering it up (default 1)");
static int ap2conn_hclk;
module_param(ap2conn_hclk, int, 0644);
MODULE_PARM_DESC(ap2conn_hclk, "write the AP2CONN hclk mux sequence (default 1)");

#define TOPCKGEN_BASE			0x10000000
#define TOPCKGEN_CLK_CFG(n)		(0x010 + 0x10 * (n))
#define TOPCKGEN_CLK_CFG_SET(n)		(0x014 + 0x10 * (n))
#define TOPCKGEN_CLK_CFG_CLR(n)		(0x018 + 0x10 * (n))
#define TOPCKGEN_UPDATE			0x004
#define TOPCKGEN_UPDATE1		0x008
#define TOPCKGEN_UPDATE2		0x012
#define TOPCKGEN_AP2CONN_CFG		20
static void conn_scan_regions(struct mt6833_conn *conn);
static void conn_wfsys_power_on(struct mt6833_conn *conn);
static void conn_vendor_prep_steps(struct mt6833_conn *conn);
static void conn_setup_emi_remap(struct mt6833_conn *conn);
static int conn_dl_rom_patch(struct mt6833_conn *conn);
static void conn_probe_wfsys(struct mt6833_conn *conn);
static void conn_release_sleep_protect(struct mt6833_conn *conn);
static void conn_log_idle_loop(struct mt6833_conn *conn);

static int conn_hw_on(struct mt6833_conn *conn)
{
	/* Before any write: what did the bootloader leave behind? */
	conn_snapshot_early(conn, "pre-poweron");

	int ret;

	/*
	 * The A-die rails.  Optional: the bootloader leaves VCN18 and VCN33_1 on,
	 * and the block comes up without them, so a board that does not describe
	 * the rails in DT still works.  Enabling them is what makes a full
	 * power-off/power-on cycle survive.
	 */
	if (conn->vcn18) {
		ret = regulator_enable(conn->vcn18);
		if (ret) {
			dev_err(conn->dev, "cannot enable vcn18: %d\n", ret);
			return ret;
		}
	}

	if (conn->vcn33_1) {
		ret = regulator_enable(conn->vcn33_1);
		if (ret) {
			dev_err(conn->dev, "cannot enable vcn33_1: %d\n", ret);
			goto err_vcn18;
		}
	}

	ret = clk_prepare_enable(conn->ccif);
	if (ret) {
		dev_err(conn->dev, "cannot enable the ccif clock: %d\n", ret);
		goto err_vcn33;
	}

	ret = pm_runtime_resume_and_get(conn->dev);
	if (ret) {
		dev_err(conn->dev, "cannot power on the CONN domain: %d\n", ret);
		goto err_clk;
	}

	/*
	 * SPM power-on configuration clock.  The vendor does this before touching
	 * the MTCMOS registers ("apply this for SPM's CONNSYS power control
	 * related CR accessing"); mainline scpsys has already written them by the
	 * time pm_runtime returned, and LK normally leaves this bit set, so the
	 * write is an idempotent safety net rather than the thing that makes it
	 * work.  Skip it silently if the board does not describe SPM.
	 */
	if (conn->win[MT6833_CONN_WIN_SPM]) {
		u32 val = readl(conn->win[MT6833_CONN_WIN_SPM] +
				CONN_SPM_PWRON_CFG_OFFSET);

		writel(val | CONN_SPM_PWRON_CFG_EN,
		       conn->win[MT6833_CONN_WIN_SPM] + CONN_SPM_PWRON_CFG_OFFSET);
	}

	/* mainline has no MT6833 power-domain driver: do the MTCMOS by hand. */
	if (power_cycle) {
		/*
		 * LK usually leaves the domain powered, which makes every step
		 * below a no-op.  The vendor always enters its power-on from a
		 * powered-down block, so cycle it to get the same starting point.
		 */
		conn_mtcmos_set(conn, false);
		msleep(20);
	}
	conn_release_bus_protect(conn);

	/*
	 * CONN bus/hif clock gates, the mainline stand-in for the vendor's
	 * clk_scp_conn_main.  Without these the CONN_HIF_ON window answers
	 * 0xdeadfeed.
	 */
	if (conn->win[MT6833_CONN_WIN_INFRA_AO] && infra_cg_extra) {
		u32 g = readl(conn->win[MT6833_CONN_WIN_INFRA_AO] + 0x80);

		writel(g | infra_cg_extra,
		       conn->win[MT6833_CONN_WIN_INFRA_AO] + 0x80);
		dev_info(conn->dev, "infra gate: +0x80 %#010x -> %#010x\n", g,
			 readl(conn->win[MT6833_CONN_WIN_INFRA_AO] + 0x80));
	}
	conn_mtcmos_set(conn, true);
	udelay(10);

	/*
	 * Assert the CONN CPU reset before the power request, exactly as the
	 * vendor does (mtk_wcn_consys_hw.c:665, before power_ctrl(ENABLE)).
	 */
	if (conn->win[MT6833_CONN_WIN_TOPRGU]) {
		u32 rgu;

		rgu = readl(conn->win[MT6833_CONN_WIN_TOPRGU] +
			    CONN_AP_RGU_CPU_SW_RST_OFFSET);
		writel(rgu | CONN_CPU_SW_RST_BIT | CONN_CPU_SW_RST_CTRL_KEY,
		       conn->win[MT6833_CONN_WIN_TOPRGU] +
		       CONN_AP_RGU_CPU_SW_RST_OFFSET);
		udelay(10);
	}

	/* The AP -> CONN power request.  This is the meat of it. */
	writel(CONN_PERI_CON_PWR_ON,
	       conn->win[MT6833_CONN_WIN_PERICFG] + CONN_PERICFG_CON_STA_OFFSET);

	msleep(CONN_PWR_ON_SETTLE_MS);

	ret = conn_poll_chip_id(conn);
	if (ret) {
		conn_dump_state(conn);
		goto err_pm;
	}

	if (!minimal) {
		conn_release_sleep_protect(conn);
		conn_probe_wfsys(conn);
		conn_setup_emi_remap(conn);
		conn_vendor_prep_steps(conn);
	} else {
		dev_info(conn->dev, "minimal: skipping all vendor prep steps\n");
	}
	/*
	 * Vendor order is bus clock first, then WFSYS power / CPU reset - but
	 * writing CLK_CFG_20 panicked this handset, so that step is opt-in via
	 * the ap2conn_hclk parameter and the default order is unchanged.
	 */
	if (minimal) {
		u32 v;

		if (steps & STEP_EMI_REMAP)
			conn_setup_emi_remap(conn);
		if (steps & STEP_PROBE) {
			conn_release_sleep_protect(conn);
			conn_probe_wfsys(conn);
		}
		if (steps & STEP_VENDOR)
			conn_vendor_prep_steps(conn);
		if (steps & STEP_ROM_PATCH)
			conn_dl_rom_patch(conn);

		v = readl(conn->win[MT6833_CONN_WIN_TOPRGU] +
			      CONN_AP_RGU_CPU_SW_RST_OFFSET);

		writel((v & ~(CONN_CPU_SW_RST_BIT | CONN_SYS_SW_RST_BIT)) |
		       CONN_CPU_SW_RST_CTRL_KEY,
		       conn->win[MT6833_CONN_WIN_TOPRGU] +
		       CONN_AP_RGU_CPU_SW_RST_OFFSET);
		dev_info(conn->dev, "minimal: rgu %#010x -> %#010x\n", v,
			 readl(conn->win[MT6833_CONN_WIN_TOPRGU] +
			       CONN_AP_RGU_CPU_SW_RST_OFFSET));
		msleep(5);
		conn_log_idle_loop(conn);
		return 0;
	}

	if (ap2conn_hclk)
		conn_topckgen_setup();
	conn_probe_hif_base(conn);
	conn_wfsys_power_on(conn);
	conn_scan_regions(conn);


	/*
	 * Release the CONN CPU reset: the CPU starts executing its ROM here.
	 * Vendor: consys_ic_hw_reset_bit_set(DISABLE) at mtk_wcn_consys_hw.c:701,
	 * after the chip id answered and before the idle-loop poll at :704.
	 */
	if (conn->win[MT6833_CONN_WIN_TOPRGU]) {
		u32 rgu;

		rgu = readl(conn->win[MT6833_CONN_WIN_TOPRGU] +
			    CONN_AP_RGU_CPU_SW_RST_OFFSET);
		writel((rgu & ~(CONN_CPU_SW_RST_BIT | CONN_SYS_SW_RST_BIT)) |
		       CONN_CPU_SW_RST_CTRL_KEY,
		       conn->win[MT6833_CONN_WIN_TOPRGU] +
		       CONN_AP_RGU_CPU_SW_RST_OFFSET);
		dev_info(conn->dev,
			 "RGU reset released: %#010x -> %#010x (bit12 cpu %d->%d, bit9 sys %d->%d)\n",
			 rgu,
			 readl(conn->win[MT6833_CONN_WIN_TOPRGU] +
			       CONN_AP_RGU_CPU_SW_RST_OFFSET),
			 !!(rgu & CONN_CPU_SW_RST_BIT),
			 !!(readl(conn->win[MT6833_CONN_WIN_TOPRGU] +
				  CONN_AP_RGU_CPU_SW_RST_OFFSET) & CONN_CPU_SW_RST_BIT),
			 !!(rgu & CONN_SYS_SW_RST_BIT),
			 !!(readl(conn->win[MT6833_CONN_WIN_TOPRGU] +
				  CONN_AP_RGU_CPU_SW_RST_OFFSET) & CONN_SYS_SW_RST_BIT));
	}

	conn->powered = true;

	conn_log_idle_loop(conn);

	return 0;

err_pm:
	pm_runtime_put_sync(conn->dev);
err_clk:
	clk_disable_unprepare(conn->ccif);
err_vcn33:
	if (conn->vcn33_1)
		regulator_disable(conn->vcn33_1);
err_vcn18:
	if (conn->vcn18)
		regulator_disable(conn->vcn18);

	return ret;
}

/*
 * Vendor check that the CONN CPU reached its ROM idle loop:
 * platform/mt6833.c:568 polls CONN_ON + 0x600 for 0x1D1E ("apply this for
 * guarantee that CONNSYS CPU goes to cos_idle_loop").  Diagnostic only: the
 * result is logged, not enforced, because it is exactly the measurement that
 * tells us whether the CONN CPU executes at all.
 */
#define CONN_IDLE_LOOP_OFFSET	0x0600
#define CONN_IDLE_LOOP_MAGIC	0x1D1E
#define CONN_PWR_STATUS_OFFSET	0x016C
#define CONN_CPUPCR_OFFSET	0x0104

/*
 * Release AP2CONN (AHB) and CONN2AP (AXI, both directions) sleep protection.
 * Written as a diagnostic: it logs the status before and after, so the effect
 * is visible in dmesg even if it does not fix the MAC handshake.
 */
/*
 * Diagnostic: is the WFSYS CPU held in reset, and does the alternate idle
 * register report the ROM idle value?  Only touches the reset register if it
 * reads back as plausible.
 */
/*
 * Copy the CONN MCU ROM patch into the CONN EMI reservation with the CPU held
 * in reset.  Returns 0 on success, negative otherwise; a failure is logged but
 * does not fail power-on, so the rest of the bring-up can still be observed.
 */
/*
 * Clear the CONN EMI reservation and program the AP->CONN EMI remap window.
 * Mirrors mtk_wcn_consys_hw.c:371-382 + consys_emi_set_remapping_reg().
 */
/*
 * Bring WFSYS up exactly like the working ports do, then report whether the
 * CONN CPU reached its ROM idle loop.  Diagnostic first: every step is logged
 * with before/after values so a single boot identifies where it stops.
 */
/*
 * Summarise a register window in 0x100-byte blocks: dead-word count, whether
 * the block varies, and a sample.  A block that is all 0x42424242 (or all
 * zero) is a hole; a block that varies is real hardware.
 */
static void conn_scan_window(struct mt6833_conn *conn, const char *name,
			     void __iomem *base, unsigned int size)
{
	unsigned int off;
	int i;

	if (!base)
		return;

	for (off = 0; off + 0x100 <= size; off += 0x100) {
		u32 sample = 0, prev = 0;
		int dead = 0, varies = 0, first = 1;

		for (i = 0; i < 0x100; i += 4) {
			u32 v = readl(base + off + i);

			if (v == 0x42424242 || v == 0xFFFFFFFF || v == 0)
				dead++;
			if (first) {
				sample = v;
				prev = v;
				first = 0;
			} else if (v != prev) {
				varies = 1;
			}
		}
		dev_info(conn->dev, "scan %s +%#05x dead=%2d/64 varies=%d sample=%#010x\n",
			 name, off, dead, varies, sample);
	}
}

/*
 * Dump the TOPCKGEN clock-mux registers (read-only).  CLK_CFG_n lives at
 * 0x010 + 0x10*n in the mt6833 topckgen, per mainline clk-mt6833-topckgen.c.
 */
/*
 * Read-only word dump, 8 words per line, for base-address comparison.
 */
static void conn_word_dump(struct mt6833_conn *conn, const char *tag,
			   unsigned long phys, unsigned int len)
{
	void __iomem *p = ioremap(phys, len);
	unsigned int off;

	if (!p) {
		dev_warn(conn->dev, "%s: ioremap %#lx failed\n", tag, phys);
		return;
	}

	for (off = 0; off < len; off += 32) {
		dev_info(conn->dev, "%s +%#05x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			 tag, off,
			 readl(p + off + 0x00), readl(p + off + 0x04),
			 readl(p + off + 0x08), readl(p + off + 0x0c),
			 readl(p + off + 0x10), readl(p + off + 0x14),
			 readl(p + off + 0x18), readl(p + off + 0x1c));
	}
	iounmap(p);
}

/*
 * Read-only snapshot of CONN/WFSYS, to answer "was it already running?".
 * Called before any write, so it reflects what the bootloader left behind.
 */
static void conn_snapshot_early(struct mt6833_conn *conn, const char *when)
{
	void __iomem *wf;
	void __iomem *on = conn->win[MT6833_CONN_WIN_ON];
	void __iomem *off = conn->win[MT6833_CONN_WIN_OFF];
	void __iomem *rgw = conn->win[MT6833_CONN_WIN_TOPRGU];
	u32 rgu = rgw ? readl(rgw + CONN_AP_RGU_CPU_SW_RST_OFFSET) : 0;

	wf = ioremap(0x18000000, 0x70000);
	dev_info(conn->dev,
		 "SNAP %s: rgu=%#010x wf_pwr_ctl=%#010x wf_cpu_rst_b=%#010x idle600=%#010x monflg=%#010x hostcsr=%#010x\n",
		 when, rgu,
		 wf ? readl(wf + 0x0000) : 0,
		 wf ? readl(wf + 0x0010) : 0,
		 on ? readl(on + 0x0600) : 0,
		 wf ? readl(wf + (CONN_WF_MONFLG_OUT - CONN_WF_ON_TOP_PWR_CTL)) : 0,
		 off ? readl(off + 0x0000) : 0);
	if (wf)
		iounmap(wf);
}

static void conn_probe_hif_base(struct mt6833_conn *conn)
{
	conn_word_dump(conn, "hif70", 0x18007000, 0x200);
	conn_word_dump(conn, "hifE0", 0x1800E000, 0x100);
	/* the windows the vendor module dumps, for a direct diff */
	conn_word_dump(conn, "conon", 0x18002000, 0x800);
	conn_word_dump(conn, "topmisc", 0x180c1000, 0x600);
	/* clock state, to compare with connclk.ko on the vendor kernel */
	conn_word_dump(conn, "topckgen", 0x10000000, 0x400);
	conn_word_dump(conn, "infraao", 0x10001000, 0x200);
	conn_word_dump(conn, "srclken", 0x1000f800, 0x40);
	/* the CONN subsystem's own clock / EMI / cfg windows */
	conn_word_dump(conn, "mcu_cr", 0x180a3000, 0x400);
	conn_word_dump(conn, "mcu_cfg", 0x180b1000, 0x400);
	conn_word_dump(conn, "hifpdma", 0x18004000, 0x400);
}

static void conn_topckgen_setup(void)
{
	void __iomem *top;
	unsigned int n;
	u32 upd, upd1, upd2;

	top = ioremap(0x10000000, 0x1000);
	if (!top)
		return;

	upd = readl(top + 0x04);
	upd1 = readl(top + 0x08);
	upd2 = readl(top + 0x12);
	pr_info("topckgen: UPDATE=%#010x UPDATE1=%#010x UPDATE2=%#010x\n",
		upd, upd1, upd2);

	for (n = 0; n <= 24; n++) {
		u32 base = 0x010 + 0x10 * n;

		pr_info("topckgen: CLK_CFG_%u [%#05x]=%#010x (set=%#010x clr=%#010x)\n",
			n, base, readl(top + base),
			readl(top + base + 0x04), readl(top + base + 0x08));
	}

	/* Select the AP2CONN hclk parent (see topckgen_setup()). */
	if (ap2conn_hclk) {
		u32 before = readl(top + TOPCKGEN_CLK_CFG(TOPCKGEN_AP2CONN_CFG));

		writel(0x0000000F, top + TOPCKGEN_CLK_CFG_CLR(TOPCKGEN_AP2CONN_CFG));
		writel(0x00000001, top + TOPCKGEN_CLK_CFG_SET(TOPCKGEN_AP2CONN_CFG));
		writel(0x00040000, top + TOPCKGEN_UPDATE2);
		udelay(50);

		pr_info("topckgen: AP2CONN hclk CLK_CFG_%d %#010x -> %#010x (UPDATE2=%#010x)\n",
			TOPCKGEN_AP2CONN_CFG, before,
			readl(top + TOPCKGEN_CLK_CFG(TOPCKGEN_AP2CONN_CFG)),
			readl(top + TOPCKGEN_UPDATE2));
	}

	iounmap(top);
}

static void conn_scan_regions(struct mt6833_conn *conn)
{
	void __iomem *wf;
	void __iomem *spm = conn->win[MT6833_CONN_WIN_SPM];
	static const unsigned int spm_offs[] = {
		0x0160, 0x0164, 0x0168, 0x016c, 0x0170, 0x0174, 0x0178,
		0x0300, 0x0304, 0x0308, 0x030c,
	};
	unsigned int i;

	/* Windows this driver already has mapped. */
	conn_scan_window(conn, "conn_on", conn->win[MT6833_CONN_WIN_ON], 0x1000);
	conn_scan_window(conn, "conn_off", conn->win[MT6833_CONN_WIN_OFF], 0x1000);

	/* WFSYS is not in the DT windows; map it just for the scan. */
	wf = ioremap(0x18000000, 0x1000);
	if (wf) {
		conn_scan_window(conn, "wfsys", wf, 0x1000);
		iounmap(wf);
	} else {
		dev_warn(conn->dev, "scan: cannot map WFSYS\n");
	}

	/* The SPM registers that gate/reflect the CONN power domain. */
	if (spm) {
		for (i = 0; i < ARRAY_SIZE(spm_offs); i++)
			dev_info(conn->dev, "scan spm +%#05x = %#010x\n",
				 spm_offs[i], readl(spm + spm_offs[i]));
	}
}

static void conn_wfsys_power_on(struct mt6833_conn *conn)
{
	/* Still held in reset here, as the vendor requires, and the
	 * A-die is known by now so the right patch variant is used. */
	conn_dl_rom_patch(conn);

	void __iomem *wf;
	u32 pwr, rst, sta, mon, v;
	int i;

	wf = ioremap(CONN_WF_ON_TOP_PWR_CTL, 0x70000);
	if (!wf) {
		dev_warn(conn->dev, "WFSYS: ioremap failed\n");
		return;
	}

	pwr = readl(wf + 0x0000);
	rst = readl(wf + (CONN_WF_CPU_RST_B - CONN_WF_ON_TOP_PWR_CTL));
	dev_info(conn->dev, "WFSYS: pwr_ctl[%#x]=%#010x cpu_rst_b[%#x]=%#010x\n",
		 CONN_WF_ON_TOP_PWR_CTL, pwr, CONN_WF_CPU_RST_B, rst);

	/*
	 * The MT6833 vendor code never writes this key-protected register: on the
	 * working system it reads 0x42424242 and stays that way.  The 0x57460080
	 * sequence comes from the MT6895 SOC7_0 port and is off by default here.
	 */
	if (wf_pwr_ctl_write) {
		v = (pwr & CONN_WF_PWR_CTL_CLR) | CONN_WF_PWR_CTL_KEY;
		writel(v, wf + 0x0000);
		udelay(10);
		dev_info(conn->dev, "WFSYS: wrote %#010x to %#x -> %#010x\n",
			 v, CONN_WF_ON_TOP_PWR_CTL, readl(wf + 0x0000));
	} else {
		dev_info(conn->dev, "WFSYS: pwr_ctl=%#010x (vendor default, not writing)\n",
			 pwr);
	}

	/* Poll the WFSYS power status (CONNAC2X sequence). */
	sta = 0;
	for (i = 0; i < 20; i++) {
		sta = readl(wf + (CONN_WF_PWR_STATUS - CONN_WF_ON_TOP_PWR_CTL));
		if (sta & CONN_WF_PWR_STATUS_RDY)
			break;
		udelay(500);
	}
	dev_info(conn->dev, "WFSYS: pwr_status[%#x]=%#010x (rdy bit2=%d)\n",
		 CONN_WF_PWR_STATUS, sta, !!(sta & CONN_WF_PWR_STATUS_RDY));

	/*
	 * De-assert the WFSYS CPU reset.  0x18000120 (the SOC7_0 offset) does not
	 * stick on this chip, so use the other member of the CONNAC2X pair,
	 * 0x18000010, and report whether it sticks.
	 */
	/*
	 * Both of these are MT6895-isms.  On MT6833 the CONN CPU reset is
	 * AP_RGU+0x18 bit 12, which the rest of this driver already handles, and
	 * the vendor leaves 0x18000110 at zero.
	 */
	if (wf_cpu_rst_write) {
		rst = readl(wf + CONN_WF_CPU_RST_B_ALT);
		writel(rst | BIT(0), wf + CONN_WF_CPU_RST_B_ALT);
		udelay(10);
		dev_info(conn->dev, "WFSYS: cpu_rst_b[%#x] %#010x -> %#010x\n",
			 CONN_WF_ON_TOP_PWR_CTL + CONN_WF_CPU_RST_B_ALT, rst,
			 readl(wf + CONN_WF_CPU_RST_B_ALT));
	} else {
		dev_info(conn->dev, "WFSYS: cpu_rst_b=%#010x (not writing)\n",
			 readl(wf + CONN_WF_CPU_RST_B_ALT));
	}

	/* Poll for the ROM idle value at the address the working port uses. */
	mon = 0;
	for (i = 0; i < 200; i++) {
		mon = readl(wf + (CONN_WF_MONFLG_OUT - CONN_WF_ON_TOP_PWR_CTL));
		if (mon == CONN_WF_ROM_IDLE)
			break;
		udelay(1000);
	}
	dev_info(conn->dev, "WFSYS: monflg[%#x]=%#010x (%s)\n",
		 CONN_WF_MONFLG_OUT, mon,
		 mon == CONN_WF_ROM_IDLE ? "ROM IDLE REACHED" : "not idle");

	iounmap(wf);
}

/*
 * The four real steps from the vendor's reg_ctrl() that this driver skipped.
 * Called after the chip id reads back and before the CPU reset is released,
 * which is exactly where the vendor does them.
 */
/*
 * Pinmux the D-die pads that carry the A-die interface, then ungate the SPM
 * clock needed to reach the CONNSYS power-control registers.  Both run before
 * the MTCMOS power-on in the vendor sequence.
 */
/*
 * Read the A-die chip id over the CONN RF SPI.  Returns 0 on failure.
 */
static u32 conn_read_adie_once(void __iomem *dbg, void __iomem *spi, u32 addr)
{
	u32 chipid;
	int retry;

	writel(readl(dbg + CONN_ON_ADIE_CTL_OFFSET) | BIT(0),
	       dbg + CONN_ON_ADIE_CTL_OFFSET);
	for (retry = 10; retry > 0; retry--) {
		if ((readl(spi) & BIT(0)) == 0)
			break;
		udelay(500);
	}

	writel(addr, spi + CONN_SPI_TOP_ADDR);
	writel(0, spi + CONN_SPI_TOP_WDAT);
	for (retry = 10; retry > 0; retry--) {
		if ((readl(spi) & BIT(5)) == 0)
			break;
		udelay(500);
	}

	chipid = readl(spi + CONN_SPI_TOP_RDAT) >> 16;

	writel(readl(dbg + CONN_ON_ADIE_CTL_OFFSET) & ~BIT(0),
	       dbg + CONN_ON_ADIE_CTL_OFFSET);
	return chipid;
}

static void conn_read_adie(struct mt6833_conn *conn)
{
	void __iomem *dbg = conn->win[MT6833_CONN_WIN_MCU_DBG];
	void __iomem *spi;
	u32 id;

	if (!dbg)
		return;
	spi = ioremap(CONN_RF_SPI_BASE, 0x100);
	if (!spi) {
		dev_warn(conn->dev, "adie: cannot map the RF SPI\n");
		return;
	}

	id = conn_read_adie_once(dbg, spi, CONN_ADIE_RD_ADDR_PRIMARY);
	if (id != CONN_ADIE_PRIMARY) {
		u32 alt = conn_read_adie_once(dbg, spi,
					      CONN_ADIE_RD_ADDR_SECONDARY);

		dev_info(conn->dev, "adie: primary read %#06x, secondary read %#06x\n",
			 id, alt);
		if (alt)
			id = alt;
	} else {
		dev_info(conn->dev, "adie: primary read %#06x\n", id);
	}

	iounmap(spi);

	if (!id) {
		dev_warn(conn->dev, "adie: no id read; keeping the existing setting\n");
		return;
	}
	mt6833_conn_adie_id = id;
	dev_info(conn->dev, "adie: detected A-die id %#06x (%s)\n", id,
		 id == CONN_ADIE_PRIMARY ? "MT6631 primary" : "MT6635 secondary");
}

static void conn_vendor_early_steps(struct mt6833_conn *conn)
{
	void __iomem *pin = ioremap(CONN_IF_PINMUX_BASE, 0x1000);
	void __iomem *drv = ioremap(CONN_IF_PINMUX_DRV_BASE, 0x100);
	void __iomem *spm = conn->win[MT6833_CONN_WIN_SPM];
	u32 v;

	if (pin) {
		v = readl(pin + CONN_IF_PINMUX_01_OFFSET);
		writel((v & CONN_IF_PINMUX_01_MASK) | CONN_IF_PINMUX_01_VALUE,
		       pin + CONN_IF_PINMUX_01_OFFSET);
		v = readl(pin + CONN_IF_PINMUX_02_OFFSET);
		writel((v & CONN_IF_PINMUX_02_MASK) | CONN_IF_PINMUX_02_VALUE,
		       pin + CONN_IF_PINMUX_02_OFFSET);
		dev_info(conn->dev, "vendor: if_pinmux 01=%#010x 02=%#010x\n",
			 readl(pin + CONN_IF_PINMUX_01_OFFSET),
			 readl(pin + CONN_IF_PINMUX_02_OFFSET));
		iounmap(pin);
	} else {
		dev_warn(conn->dev, "vendor: if_pinmux ioremap failed\n");
	}

	if (drv) {
		v = readl(drv + 0x0);
		writel((v & CONN_IF_PINMUX_DRV_MASK_1) | CONN_IF_PINMUX_DRV_VALUE_1,
		       drv + 0x0);
		v = readl(drv + 0x10);
		writel((v & CONN_IF_PINMUX_DRV_MASK_2) | CONN_IF_PINMUX_DRV_VALUE_2,
		       drv + 0x10);
		dev_info(conn->dev, "vendor: pad drive set to 2 mA\n");
		iounmap(drv);
	}

	if (spm) {
		v = readl(spm);
		writel(v | CONN_SPM_PWR_ON_CLK, spm);
		dev_info(conn->dev, "vendor: spm pwr_on_clk %#010x -> %#010x\n", v,
			 readl(spm));
	}
}

static void conn_vendor_prep_steps(struct mt6833_conn *conn)
{
	conn_vendor_early_steps(conn);

	conn_read_adie(conn);

	void __iomem *on = conn->win[MT6833_CONN_WIN_ON];
	void __iomem *dbg = conn->win[MT6833_CONN_WIN_MCU_DBG];
	u32 v;

	if (!on)
		return;

	/*
	 * 1. conn2ap EMI access in hardware mode.  This lives in MCU_TOP_MISC_ON,
	 * which is 0x180C1000 (proven by identify_adie: 0x180C1130 == base+0x130),
	 * not the CONN_ON window - writing 0x18002168 silently did nothing.
	 */
	if (dbg) {
		v = readl(dbg + CONN_ACCESS_EMI_HW_MODE_OFFSET);
		writel(v | BIT(0), dbg + CONN_ACCESS_EMI_HW_MODE_OFFSET);
		dev_info(conn->dev, "vendor: access_emi_hw_mode %#010x -> %#010x\n", v,
			 readl(dbg + CONN_ACCESS_EMI_HW_MODE_OFFSET));
	}

	/* 2. Where the CONN CPU's code starts. */
	writel(CONN_EMI_BT_ENTRY_ADDRESS, on + CONN_EMI_BT_ENTRY_OFFSET);
	writel(CONN_EMI_WIFI_ENTRY_ADDRESS, on + CONN_EMI_WIFI_ENTRY_OFFSET);
	dev_info(conn->dev, "vendor: emi entry bt=%#010x wifi=%#010x\n",
		 readl(on + CONN_EMI_BT_ENTRY_OFFSET),
		 readl(on + CONN_EMI_WIFI_ENTRY_OFFSET));

	/* 3. Not in co-clock mode: turn the RC oscillator path off. */
	if (dbg) {
		v = readl(dbg + CONN_COCLOCK_RC_CTL_0_OFFSET);
		writel(v & ~CONN_COCLOCK_TOP_OSC_RC_EN,
		       dbg + CONN_COCLOCK_RC_CTL_0_OFFSET);
		dev_info(conn->dev, "vendor: coclock rc_ctl_0 %#010x -> %#010x\n", v,
			 readl(dbg + CONN_COCLOCK_RC_CTL_0_OFFSET));
	} else {
		dev_warn(conn->dev, "vendor: no MCU debug window for coclock\n");
	}

	/* 4. The WiFi rail (vcn33_2), which this driver never enabled. */
	if (conn->vcn33_2_wifi) {
		int ret = regulator_set_voltage(conn->vcn33_2_wifi, 2800000, 2800000);

		if (ret)
			dev_warn(conn->dev, "vendor: vcn33_2 set_voltage %d\n", ret);
		ret = regulator_enable(conn->vcn33_2_wifi);
		dev_info(conn->dev, "vendor: vcn33_2_wifi enabled (%d)\n", ret);
	}
	/*
	 * identify A-die: 0x180C1130 bit 8.  The vendor SETS it for a primary
	 * A-die (0x6631) and CLEARS it for a secondary one (0x6635); the direction
	 * follows whatever conn_read_adie() actually found.
	 */
	if (dbg) {
		u32 mask = mt6833_conn_adie_id == CONN_ADIE_PRIMARY ?
			   CONN_IDENTIFY_ADIE_ENABLE : 0;

		v = readl(dbg + CONN_TOP_MISC_ON_OFFSET);
		writel((v & ~CONN_IDENTIFY_ADIE_ENABLE) | mask,
		       dbg + CONN_TOP_MISC_ON_OFFSET);
		dev_info(conn->dev, "vendor: identify_adie %#010x -> %#010x (adie %#x)\n",
			 v, readl(dbg + CONN_TOP_MISC_ON_OFFSET),
			 mt6833_conn_adie_id);
	}

	/* CONNSYS AHB bus timeout */
	v = readl(on + CONN_AHB_TIMEOUT_ADDR_OFFSET);
	writel(CONN_AHB_TIMEOUT_VALUE, on + CONN_AHB_TIMEOUT_ADDR_OFFSET);
	dev_info(conn->dev, "vendor: ahb_timeout %#010x -> %#010x\n", v,
		 readl(on + CONN_AHB_TIMEOUT_ADDR_OFFSET));
}

static void conn_setup_emi_remap(struct mt6833_conn *conn)
{
	void __iomem *infra = conn->win[MT6833_CONN_WIN_INFRA_AO];
	phys_addr_t base = 0;
	size_t size = 0;
	void __iomem *emi;
	u32 map, peri, before_map, before_peri;

	if (!infra)
		return;
	if (mt6833_conn_memory(conn, emi_use_wifi ? MT6833_CONN_MEM_WIFI : MT6833_CONN_MEM_FW, &base, &size)) {
		dev_warn(conn->dev, "EMI remap: no reservation\n");
		return;
	}

	/* The vendor wipes the whole reservation before handing it over. */
	emi = ioremap(base, size);
	if (emi) {
		if (!no_clear)
			memset_io(emi, 0, size);
		iounmap(emi);
	}

	map = (u32)((base >> 16) & CONN_EMI_MAP_MASK);
	peri = (u32)((CONN_EMI_PERI_BASE >> 16) & CONN_EMI_MAP_MASK);

	before_map = readl(infra + CONN_EMI_MAPPING_OFFSET);
	before_peri = readl(infra + CONN_EMI_PERI_MAPPING_OFFSET);

	writel((before_map & ~CONN_EMI_MAP_MASK) | map,
	       infra + CONN_EMI_MAPPING_OFFSET);
	writel((before_peri & ~CONN_EMI_MAP_MASK) | peri,
	       infra + CONN_EMI_PERI_MAPPING_OFFSET);

	dev_info(conn->dev,
		 "EMI remap: base %#llx (cleared %zu bytes), reg[%#x] %#010x -> %#010x, reg[%#x] %#010x -> %#010x\n",
		 (unsigned long long)base, size,
		 CONN_EMI_MAPPING_OFFSET, before_map,
		 readl(infra + CONN_EMI_MAPPING_OFFSET),
		 CONN_EMI_PERI_MAPPING_OFFSET, before_peri,
		 readl(infra + CONN_EMI_PERI_MAPPING_OFFSET));
}

/*
 * Copy one patch blob (minus its 32-byte struct wmt_rom_patch header) into the
 * CONN EMI at the given offset.
 */

/*
 * Build-time stamp slots in the EMI coredump area.  The vendor writes each
 * patch's 16-byte timestamp here right after copying the body
 * (wmt_ic_soc.c:3637-3648, mt6833.h:177-190).
 */
#define CONN_EMI_COREDUMP_OFFSET	0x88000
#define CONN_EMI_BT_BUILDTIME_OFFSET	(CONN_EMI_COREDUMP_OFFSET + 0x300)
#define CONN_EMI_WIFI_BUILDTIME_OFFSET	(CONN_EMI_COREDUMP_OFFSET + 0x310)
#define CONN_EMI_MCU_BUILDTIME_OFFSET	(CONN_EMI_COREDUMP_OFFSET + 0x320)
#define CONN_EMI_BUILDTIME_SIZE		16

static void conn_write_buildtime(phys_addr_t emi_base, size_t emi_size,
				 u32 offset, const u8 *stamp)
{
	void __iomem *p;

	if (offset + CONN_EMI_BUILDTIME_SIZE > emi_size)
		return;
	p = ioremap(emi_base + offset, CONN_EMI_BUILDTIME_SIZE);
	if (!p)
		return;
	memcpy_toio(p, stamp, CONN_EMI_BUILDTIME_SIZE);
	iounmap(p);
}

static int conn_copy_patch(struct mt6833_conn *conn, phys_addr_t emi_base,
			   size_t emi_size, const char *name, u32 offset)
{
	const struct firmware *fw;
	void __iomem *dst;
	size_t body_len;
	int ret;

	ret = request_firmware(&fw, name, conn->dev);
	if (ret) {
		dev_warn(conn->dev, "ROM patch: %s not available (%d)\n", name, ret);
		return ret;
	}
	if (fw->size <= (size_t)patch_hdr) {
		dev_err(conn->dev, "ROM patch: %s too small (%zu)\n", name, fw->size);
		release_firmware(fw);
		return -EINVAL;
	}
	body_len = fw->size - patch_hdr;
	if (offset + body_len > emi_size) {
		dev_err(conn->dev, "ROM patch: %s does not fit at %#x\n", name, offset);
		release_firmware(fw);
		return -EINVAL;
	}

	dst = ioremap(emi_base + offset, body_len);
	if (!dst) {
		dev_err(conn->dev, "ROM patch: cannot map %s destination\n", name);
		release_firmware(fw);
		return -ENOMEM;
	}
	memcpy_toio(dst, fw->data + patch_hdr, body_len);
	iounmap(dst);

	/* Stamp the build time where the vendor does; the firmware reads these. */
	if (fw->size >= CONN_EMI_BUILDTIME_SIZE) {
		u32 bt = 0;	/* buildtime offset for this patch */

		switch (offset) {
		case 0x000000:
			bt = CONN_EMI_MCU_BUILDTIME_OFFSET;
			break;
		case 0x170000:
			bt = CONN_EMI_BT_BUILDTIME_OFFSET;
			break;
		case 0x2a0000:
			bt = CONN_EMI_WIFI_BUILDTIME_OFFSET;
			break;
		default:
			break;
		}
		if (bt) {
			conn_write_buildtime(emi_base, emi_size, bt, fw->data);
			dev_info(conn->dev,
				 "ROM patch: %s buildtime -> EMI+%#x [%c%c%c%c...]\n",
				 name, bt, fw->data[0], fw->data[1], fw->data[2],
				 fw->data[3]);
		}
	}

	release_firmware(fw);

	dev_info(conn->dev, "ROM patch: %s -> EMI %#llx+%#x (%zu bytes)\n",
		 name, (unsigned long long)emi_base, offset, body_len);
	return 0;
}

/*
 * Download the three CONN patches at the vendor offsets.  The vendor does this
 * with the CPU held in reset, immediately before the idle poll; with only the
 * MCU blob present the CPU never comes up.
 */
static int conn_dl_rom_patch(struct mt6833_conn *conn)
{
	phys_addr_t emi_base = 0;
	size_t emi_size = 0;
	bool sec = (mt6833_conn_adie_id == 0x6635);
	int ret;

	ret = mt6833_conn_memory(conn, emi_use_wifi ? MT6833_CONN_MEM_WIFI : MT6833_CONN_MEM_FW, &emi_base, &emi_size);
	if (ret) {
		dev_warn(conn->dev, "ROM patch: no EMI reservation (%d)\n", ret);
		return ret;
	}

	dev_info(conn->dev, "ROM patch: A-die %#x, flavour %s, EMI %#llx (%zu bytes)\n",
		 mt6833_conn_adie_id, sec ? "'a'" : "none",
		 (unsigned long long)emi_base, emi_size);

	conn_emi_base_cached = emi_base;
	conn_emi_size_cached = (u32)emi_size;

	/* MCU first: this is the one the CONN CPU runs. */
	if (patch_mode == 0 || patch_mode == 1)
		conn_copy_patch(conn, emi_base, emi_size,
				sec ? CONN_ROM_PATCH_MCU_SECONDARY
				    : CONN_ROM_PATCH_MCU_PRIMARY,
				CONN_ROM_PATCH_MCU_OFFSET);
	if (patch_mode == 0 || patch_mode == 2) {
		conn_copy_patch(conn, emi_base, emi_size,
				sec ? CONN_ROM_PATCH_BT_SECONDARY
				    : CONN_ROM_PATCH_BT_PRIMARY,
				CONN_ROM_PATCH_BT_OFFSET);
		conn_copy_patch(conn, emi_base, emi_size,
				sec ? CONN_ROM_PATCH_WIFI_SECONDARY
				    : CONN_ROM_PATCH_WIFI_PRIMARY,
				CONN_ROM_PATCH_WIFI_OFFSET);
	}
	return 0;
}

static void conn_probe_wfsys(struct mt6833_conn *conn)
{
	void __iomem *wf;
	u32 rst, mon, on600;

	wf = ioremap(CONN_WFSYS_BASE, CONN_WFSYS_MAP_SIZE);
	if (!wf) {
		dev_warn(conn->dev, "WFSYS: ioremap failed\n");
		return;
	}

	rst = readl(wf + CONN_WFSYS_CPU_SW_RST_OFFSET);
	mon = readl(wf + CONN_WFSYS_IDLE_MONFLG_OFFSET);
	on600 = conn->win[MT6833_CONN_WIN_ON] ?
		readl(conn->win[MT6833_CONN_WIN_ON] + 0x600) : 0;

	dev_info(conn->dev,
		 "WFSYS: rst[%#x]=%#010x monflg[%#x]=%#010x conn_on+600=%#010x\n",
		 CONN_WFSYS_CPU_SW_RST_OFFSET, rst,
		 CONN_WFSYS_IDLE_MONFLG_OFFSET, mon, on600);

	if (rst != 0 && rst != 0xFFFFFFFF) {
		writel(rst | CONN_WFSYS_CPU_SW_RST_BIT,
		       wf + CONN_WFSYS_CPU_SW_RST_OFFSET);
		udelay(2);
		writel(rst & ~CONN_WFSYS_CPU_SW_RST_BIT,
		       wf + CONN_WFSYS_CPU_SW_RST_OFFSET);
		mdelay(2);
		dev_info(conn->dev,
			 "WFSYS: CPU SW reset pulsed on %#010x (now %#010x)\n",
			 rst, readl(wf + CONN_WFSYS_CPU_SW_RST_OFFSET));
	} else {
		dev_warn(conn->dev,
			 "WFSYS: rst reads %#010x, looks unmapped - not touching it\n",
			 rst);
	}

	iounmap(wf);
}

static void conn_release_sleep_protect(struct mt6833_conn *conn)
{
	void __iomem *infra = conn->win[MT6833_CONN_WIN_INFRA_AO];
	u32 axi_before, ahb_before, axi_after, ahb_after;

	if (!infra)
		return;

	axi_before = readl(infra + CONN_INFRA_AXI_PROT_STA);
	ahb_before = readl(infra + CONN_INFRA_AHB_PROT_STA);

	/* write-1-to-clear side: release all four directions */
	writel(CONN_PROT_AXI_RX, infra + CONN_INFRA_AXI_PROT_EN_CLR);
	writel(CONN_PROT_AXI_TX, infra + CONN_INFRA_AXI_PROT_EN_CLR);
	writel(CONN_PROT_AHB_TX, infra + CONN_INFRA_AXI_PROT_EN_CLR);
	writel(CONN_PROT_AHB_RX, infra + CONN_INFRA_AHB_PROT_EN_CLR);
	mdelay(5);

	axi_after = readl(infra + CONN_INFRA_AXI_PROT_STA);
	ahb_after = readl(infra + CONN_INFRA_AHB_PROT_STA);

	dev_info(conn->dev,
		 "bus protect: AXI %#010x -> %#010x (rx bit14 %d->%d, tx bit18 %d->%d)\n",
		 axi_before, axi_after,
		 !!(axi_before & CONN_PROT_AXI_RX), !!(axi_after & CONN_PROT_AXI_RX),
		 !!(axi_before & CONN_PROT_AXI_TX), !!(axi_after & CONN_PROT_AXI_TX));
	dev_info(conn->dev,
		 "bus protect: AHB %#010x -> %#010x (rx bit10 %d->%d, tx bit13 %d->%d)\n",
		 ahb_before, ahb_after,
		 !!(ahb_before & CONN_PROT_AHB_RX), !!(ahb_after & CONN_PROT_AHB_RX),
		 !!(ahb_before & CONN_PROT_AHB_TX), !!(ahb_after & CONN_PROT_AHB_TX));
}

static void conn_log_idle_loop(struct mt6833_conn *conn)
{
	u32 val = 0;
	int i;

	if (!conn->win[MT6833_CONN_WIN_ON])
		return;

	/*
	 * The vendor budgets 10 x 20 ms and the signature appears ~85 ms after the
	 * reset is released, so polling for only a few tens of milliseconds misses
	 * it.  idle_poll_ms defaults to 400.
	 */
	for (i = 0; i < idle_poll_ms; i++) {
		val = readl(conn->win[MT6833_CONN_WIN_ON] + CONN_IDLE_LOOP_OFFSET);
		if (val == CONN_IDLE_LOOP_MAGIC)
			break;
		usleep_range(1000, 1000);
	}

	dev_info(conn->dev, "CONN CPU idle check: CONN_ON+%#x = %#010x (%s)\n",
		 CONN_IDLE_LOOP_OFFSET, val,
		 val == CONN_IDLE_LOOP_MAGIC ? "idle loop reached" :
					       "NOT in idle loop");

	if (val != CONN_IDLE_LOOP_MAGIC) {
		dev_info(conn->dev, "CONN CPU state: CONN_ON+%#x = %#010x, CONN_OFF+%#x = %#010x\n",
			 CONN_PWR_STATUS_OFFSET,
			 readl(conn->win[MT6833_CONN_WIN_ON] + CONN_PWR_STATUS_OFFSET),
			 CONN_CPUPCR_OFFSET,
			 conn->win[MT6833_CONN_WIN_OFF] ?
			 readl(conn->win[MT6833_CONN_WIN_OFF] + CONN_CPUPCR_OFFSET) :
			 0);
	}
}

static void conn_stp_run(struct mt6833_conn *conn);
static int conn_hw_on(struct mt6833_conn *conn);
static void conn_hw_off(struct mt6833_conn *conn);

static struct mt6833_conn *conn_cycle_dev;

static ssize_t conn_cycle_write(struct file *f, const char __user *buf,
				size_t len, loff_t *off)
{
	struct mt6833_conn *conn = conn_cycle_dev;

	if (!conn)
		return -ENODEV;
	dev_info(conn->dev, "conn_cycle: minimal=%d steps=%#x, power cycling\n",
		 minimal, steps);
	conn_hw_off(conn);
	msleep(50);
	conn_hw_on(conn);
	dev_info(conn->dev, "conn_cycle: done, CONN_ON+0x600=%#010x CPUPCR=%#010x\n",
		 readl(conn->win[MT6833_CONN_WIN_ON] + 0x600),
		 readl(conn->win[MT6833_CONN_WIN_OFF] + 0x104));

	if (replay_probe) {
		if (stp_before_probe) {
			dev_info(conn->dev, "conn_cycle: running WMT/STP first\n");
			conn_stp_run(conn);
		}
		msleep(50);
		dev_info(conn->dev, "conn_cycle: replaying the WLAN probe\n");
		if (conn_replay_probe_hook)
			conn_replay_probe_hook();
		else
			dev_warn(conn->dev, "conn_cycle: no probe hook installed\n");
	}
	return len;
}


/* Run the power-on once per `steps` value in a space-separated list. */
static ssize_t conn_sweep_write(struct file *f, const char __user *buf,
				size_t len, loff_t *off)
{
	struct mt6833_conn *conn = conn_cycle_dev;
	char kbuf[256];
	char *tok, *cur;

	if (!conn)
		return -ENODEV;
	if (len >= sizeof(kbuf))
		len = sizeof(kbuf) - 1;
	if (copy_from_user(kbuf, buf, len))
		return -EFAULT;
	kbuf[len] = 0;

	for (cur = kbuf; (tok = strsep(&cur, " \n\t")); ) {
		unsigned int v;
		u32 cpu;

		if (!*tok)
			continue;
		if (kstrtouint(tok, 0, &v))
			continue;
		steps = v;
		minimal = 1;

		conn_hw_off(conn);
		msleep(40);
		conn_hw_on(conn);
		cpu = readl(conn->win[MT6833_CONN_WIN_OFF] + 0x104);
		dev_info(conn->dev,
			 "SWEEP steps=%#x -> idle600=%#010x cpupcr=%#010x\n",
			 v, readl(conn->win[MT6833_CONN_WIN_ON] + 0x600), cpu);
	}
	return len;
}

static const struct proc_ops conn_sweep_pops = {
	.proc_write	= conn_sweep_write,
};


/* Raw register access for bring-up: "r <addr>" or "w <addr> <val>". */
static ssize_t conn_reg_write(struct file *f, const char __user *buf,
			      size_t len, loff_t *off)
{
	struct mt6833_conn *conn = conn_cycle_dev;
	char kbuf[128];
	char op;
	unsigned long addr;
	unsigned int val = 0;
	void __iomem *p;

	if (!conn)
		return -ENODEV;
	if (len >= sizeof(kbuf))
		len = sizeof(kbuf) - 1;
	if (copy_from_user(kbuf, buf, len))
		return -EFAULT;
	kbuf[len] = 0;

	if (sscanf(kbuf, " %c %lx %x", &op, &addr, &val) < 2)
		return -EINVAL;
	p = ioremap(addr & ~3UL, 4);
	if (!p)
		return -ENOMEM;
	if (op == 'w') {
		writel(val, p);
		dev_info(conn->dev, "reg w %#lx = %#010x (reads %#010x)\n",
			 addr, val, readl(p));
	} else {
		dev_info(conn->dev, "reg r %#lx = %#010x\n", addr, readl(p));
	}
	iounmap(p);
	return len;
}

static const struct proc_ops conn_reg_pops = {
	.proc_write	= conn_reg_write,
};


/* ------------------------------------------------------------------ */
/* Minimal BTIF + WMT_RESET: kick the CONN MCU out of its ROM idle loop. */
/* ------------------------------------------------------------------ */

#define CONN_BTIF_BASE		0x1100C000UL
#define CONN_BTIF_THR		0x00
#define CONN_BTIF_IER		0x04
#define CONN_BTIF_FIFOCTRL	0x08
#define CONN_BTIF_FAKELCR	0x0C
#define CONN_BTIF_DMA_EN	0x4C
#define CONN_BTIF_TRI_LVL	0x60
#define CONN_BTIF_HANDSHAKE	0x6C
#define CONN_BTIF_WAK		0x64
#define CONN_BTIF_STAT		0x10

/*
 * BTIF line-status bits and the BTIF_TRI_LVL field layout, taken from vendor
 * plat_inc/{btif_priv.h,plat_common.h}.  conn_wmt_kick() sits above the WMT
 * frame block that used to own these, so they are defined here instead.
 */
#define BTIF_LSR		0x14
#define BTIF_LSR_DR		BIT(0)
#define BTIF_LSR_THRE		BIT(5)
#define BTIF_LSR_TEMT		BIT(6)
#define BTIF_TRI_LVL_TXVAL(x)	(((x) & 0xf) << 0)
#define BTIF_TRI_LVL_RXVAL(x)	(((x) & 0x7) << 4)

/* The WMT payloads now live in conn_wmt_kick(); see the p_* tables there. */


/* BTIF needs CLK_IFRAO_BTIF (and friends) before its registers accept writes. */
static struct clk *conn_btif_clks[4];
static int conn_btif_nclk;

static void conn_btif_clocks_on(struct device *dev)
{
	struct device_node *np;
	int i;

	if (conn_btif_nclk)
		return;

	np = of_find_compatible_node(NULL, NULL, "mediatek,btif");
	if (!np) {
		dev_warn(dev, "wmt: no mediatek,btif node\n");
		return;
	}

	for (i = 0; i < 4; i++) {
		struct clk *c = of_clk_get(np, i);
		int ret;

		if (IS_ERR(c))
			break;
		ret = clk_prepare_enable(c);
		dev_info(dev, "wmt: btif clock[%d] -> %d\n", i, ret);
		if (!ret)
			conn_btif_clks[conn_btif_nclk++] = c;
	}
	of_node_put(np);
}

/* Defined with the rest of the WMT frame helpers, far below. */
static int btif_tx(void __iomem *b, const u8 *payload, int len);

/*
 * Send one WMT command over the already-initialised BTIF link and collect the
 * reply.  A live WMT service answers with group 2 (event), the same opcode and
 * a status byte, so all three are printed.
 */
static int conn_wmt_xfer(struct mt6833_conn *conn, void __iomem *b,
			 const char *name, const u8 *pl, int len)
{
	u8 buf[64];
	int i, n, rx = 0;

	n = btif_tx(b, pl, len);
	if (n < 0) {
		dev_err(conn->dev, "wmt: %s tx failed (%d)\n", name, n);
		return n;
	}

	/*
	 * OPCODE_FUNC_CTRL (0x06) initialises a whole subsystem, so give the
	 * MCU far longer than the millisecond-scale replies of the earlier
	 * negotiation commands.  vendor wmt_core_rx() blocks instead of
	 * polling, and the BT core can take well over a second to come up.
	 */
	for (i = 0; i < 1500; i++) {
		u32 lsr = readl(b + BTIF_LSR);

		if (lsr & BTIF_LSR_DR) {
			if (rx < (int)sizeof(buf))
				buf[rx++] = readl(b + CONN_BTIF_THR) & 0xff;
			continue;
		}
		if (rx)
			break;
		udelay(1000);
	}

	if (!rx) {
		dev_info(conn->dev, "wmt: %-8s no reply\n", name);
		return 0;
	}

	dev_info(conn->dev, "wmt: %-8s rx %d grp=%u op=%#04x status=%u\n",
		 name, rx, buf[4], buf[5], rx > 8 ? buf[8] : 0);
	for (i = 0; i < rx; i += 16) {
		int left = rx - i > 16 ? 16 : rx - i;

		dev_info(conn->dev, "wmt:   [%02d] %*phN\n", i, left, buf + i);
	}
	return rx;
}

static int conn_wmt_kick(struct mt6833_conn *conn)
{
	conn_btif_clocks_on(conn->dev);

	void __iomem *b;
	u32 v;
	int ret = 0;

	b = ioremap(CONN_BTIF_BASE, 0x1000);
	if (!b) {
		dev_err(conn->dev, "wmt: cannot map BTIF\n");
		return -ENOMEM;
	}

	dev_info(conn->dev, "wmt: BTIF stat=%#010x ier=%#010x fifo=%#010x\n",
		 readl(b + CONN_BTIF_STAT), readl(b + CONN_BTIF_IER),
		 readl(b + CONN_BTIF_FIFOCTRL));

	writel(0, b + CONN_BTIF_FAKELCR);
	dev_info(conn->dev, "wmt: fakeLCR=%#010x stat=%#010x\n",
		 readl(b + CONN_BTIF_FAKELCR), readl(b + CONN_BTIF_STAT));

	v = readl(b + CONN_BTIF_HANDSHAKE);
	writel(v | 1, b + CONN_BTIF_HANDSHAKE);
	dev_info(conn->dev, "wmt: handshake %#010x -> %#010x\n", v,
		 readl(b + CONN_BTIF_HANDSHAKE));

/* FIFO flush in the vendor's order: RX set->clear, then TX set->clear. */
        v = readl(b + CONN_BTIF_FIFOCTRL);
        writel(v | BIT(1), b + CONN_BTIF_FIFOCTRL);
        writel(v & ~BIT(1), b + CONN_BTIF_FIFOCTRL);
        v = readl(b + CONN_BTIF_FIFOCTRL);
        writel(v | BIT(2), b + CONN_BTIF_FIFOCTRL);
        writel(v & ~BIT(2), b + CONN_BTIF_FIFOCTRL);
        dev_info(conn->dev, "wmt: fifoctrl=%#010x\n", readl(b + CONN_BTIF_FIFOCTRL));

        /*
         * Trigger levels: TX 8 (half of the 16-byte FIFO), RX 1, internal
         * loopback off - exactly what vendor hal_btif_hw_init() programs.
         * Writing the raw register back left every threshold at zero.
         */
        writel(BTIF_TRI_LVL_TXVAL(8) | BTIF_TRI_LVL_RXVAL(1),
               b + CONN_BTIF_TRI_LVL);
        dev_info(conn->dev, "wmt: tri_lvl=%#010x\n",
                 readl(b + CONN_BTIF_TRI_LVL));

	v = readl(b + CONN_BTIF_DMA_EN);
	writel(v | BIT(2), b + CONN_BTIF_DMA_EN);
	writel(1, b + CONN_BTIF_IER);

	/* wake the peer */
	v = readl(b + CONN_BTIF_WAK);
	writel(v | 1, b + CONN_BTIF_WAK);
	udelay(50);
	dev_info(conn->dev, "wmt: dma_en=%#010x ier=%#010x stat=%#010x\n",
		 readl(b + CONN_BTIF_DMA_EN), readl(b + CONN_BTIF_IER),
		 readl(b + CONN_BTIF_STAT));

	/*
	 * RESET is acknowledged, so the MCU's WMT service is alive.  Walk the rest
	 * of the handshake the vendor WMT core runs before any firmware download;
	 * every step prints the group/opcode/status the MCU answered with.
	 */
	{
		static const u8 p_reset[]   = { 0x01, 0x07, 0x01, 0x00, 0x04 };
		/*
		 * vendor init_table_1_2: right after RESET the WMT core negotiates
		 * the HIF link using HIF(4) sub-commands in param[0] - QUERY_BAUD
		 * (0x02) then QUERY_STP (0x04), one byte of payload each.  We used
		 * to jump straight to FUNC_CTRL without this, which is likely why
		 * the MCU never answered it.
		 */
		static const u8 p_q_baud[]  = { 0x01, 0x04, 0x01, 0x00, 0x02 };
		static const u8 p_q_stp[]   = { 0x01, 0x04, 0x01, 0x00, 0x04 };
		static const u8 p_chipid[]  = { 0x01, 0x02, 0x05, 0x00,
						0x11, 0x00, 0x00, 0x00, 0x00 };
		static const u8 p_adie[]    = { 0x01, 0x02, 0x01, 0x00, 0x12 };
		static const u8 p_allint[]  = { 0x01, 0x08, 0x10, 0x00,
						0x01, 0x01, 0x00, 0x01,
						0x00, 0x03, 0x05, 0x80,
						0x00, 0xC4, 0x00, 0x00,
						0x00, 0xC4, 0x00, 0x00 };
		static const u8 p_wk_gate[] = { 0x01, 0x03, 0x01, 0x00, 0x04 };
		static const u8 p_btp2[]    = { 0x01, 0x10, 0x03, 0x00,
						0x01, 0x03, 0x01 };
		static const u8 p_strap[]   = { 0x01, 0x05, 0x02, 0x00, 0x02, 0x02 };
		static const u8 p_set_stp[] = { 0x01, 0x04, 0x05, 0x00,
						0x03, 0xdb, 0x0e, 0x68, 0x01 };
		/*
		 * WMT_CTRL_ON = OPCODE_FUNC_CTRL(6) carrying a drv type and on/off.
		 * Types are from vendor ENUM_WMTDRV_TYPE_T: BT=0, GPS=2, WIFI=3,
		 * WMT=4.  OPCODE 0x14 (the MT6895 ANT_SWAP/RF_CAL pair) is not
		 * answered by this MCU at all, so it is dropped.
		 */
		static const u8 p_on_wmt[]  = { 0x01, 0x06, 0x02, 0x00, 0x04, 0x01 };
		/*
		 * Kept only to document what used to be sent: opcode 6 is EXIT in
		 * the real WMT table (FUNC_ON is 3), and FUNC_ON for Wi-Fi is not
		 * a command at all -- see the note at the end of this sequence.
		 */
		static const u8 p_on_wifi[] __maybe_unused = { 0x01, 0x06, 0x02, 0x00, 0x03, 0x01 };
		/* Kept for when the BT firmware assert is ever sorted out. */
		static const u8 p_on_bt[] __maybe_unused = { 0x01, 0x06, 0x02, 0x00, 0x00, 0x01 };

		/*
		 * vendor wmt_ic_soc.c init table, in its original order: the very
		 * first thing the WMT core does is write the interrupt mask into the
		 * MCU (INT 0x08 -> reg 0x80050300), then disable wakeup gating and
		 * negotiate the HIF link.  FUNC_CTRL is not part of this table.
		 */
		conn_wmt_xfer(conn, b, "ALLINT", p_allint, sizeof(p_allint));
		conn_wmt_xfer(conn, b, "WK_GATE", p_wk_gate, sizeof(p_wk_gate));
		conn_wmt_xfer(conn, b, "Q_STP", p_q_stp, sizeof(p_q_stp));
		conn_wmt_xfer(conn, b, "Q_BAUD", p_q_baud, sizeof(p_q_baud));
		conn_wmt_xfer(conn, b, "RESET", p_reset, sizeof(p_reset));
		conn_wmt_xfer(conn, b, "BTP2", p_btp2, sizeof(p_btp2));
		conn_wmt_xfer(conn, b, "CHIPID", p_chipid, sizeof(p_chipid));
		/*
		 * SET_STP is where the MCU falls silent: everything the vendor
		 * table puts after it (STRAP, ADIE, the func-ctrl trio) stops
		 * answering.  So drive the subsystem enable while the WMT control
		 * channel is still known good, and only then configure STP.
		 */
		conn_wmt_xfer(conn, b, "ON_WMT", p_on_wmt, sizeof(p_on_wmt));
		conn_wmt_xfer(conn, b, "SET_STP", p_set_stp, sizeof(p_set_stp));
		conn_wmt_xfer(conn, b, "STRAP", p_strap, sizeof(p_strap));
		conn_wmt_xfer(conn, b, "ADIE", p_adie, sizeof(p_adie));
		/*
		 * No ON_WIFI / ON_BT here on purpose.
		 *
		 * Reading the vendor WMT engine (common_main/core/wmt_core.c)
		 * settles what these really are:
		 *   - the opcode table is HIF_CONF=0, PWR_ON=1, PWR_OFF=2,
		 *     FUNC_ON=3, FUNC_OFF=4, REG_RW=5, EXIT=6, ...  so the bytes
		 *     this driver used to send as "ON_WIFI" (opcode 6) were EXIT,
		 *     and "ADIE" (opcode 2) was PWR_OFF -- which is why the MCU
		 *     kept asserting and why ADIE answered 0xff.
		 *   - opfunc_func_on() for WMTDRV_TYPE_WIFI never emits a command
		 *     at all: it does PWR_ON and then turns the op into a local
		 *     WMT_OPID_WLAN_PROBE, i.e. it just calls hifAxiProbe().
		 * So there is nothing for us to send to enable Wi-Fi: the WLAN
		 * driver downloads WIFI_RAM_CODE and starts the WLAN MCU itself.
		 * The CONN-side power-on is already done above.
		 */
	}

	/* give the MCU a moment, then report what it says about itself */
	msleep(200);
	dev_info(conn->dev, "wmt: after reset: CONN_ON+0x600=%#010x CPUPCR=%#010x\n",
		 readl(conn->win[MT6833_CONN_WIN_ON] + 0x600),
		 readl(conn->win[MT6833_CONN_WIN_OFF] + 0x104));
	/* WFSYS ready bit: without it the WLAN driver cannot own the bus. */
	dev_info(conn->dev, "wmt: WFSYS pwr_status=%#010x (rdy bit2=%lu)\n",
		 readl(conn->win[MT6833_CONN_WIN_ON] + 0x2cc),
		 (unsigned long)((readl(conn->win[MT6833_CONN_WIN_ON] + 0x2cc) >> 2) & 1));

	iounmap(b);
	return ret;
}

static ssize_t conn_wmt_write(struct file *f, const char __user *buf,
			      size_t len, loff_t *off)
{
	struct mt6833_conn *conn = conn_cycle_dev;

	if (!conn)
		return -ENODEV;
	dev_info(conn->dev, "wmt: kick requested\n");
	conn_wmt_kick(conn);
	return len;
}

static const struct proc_ops conn_wmt_pops = {
	.proc_write	= conn_wmt_write,
};


/* ------------------------------------------------------------------ */
/* Probe the WLAN interrupt line.                                      */
/* ------------------------------------------------------------------ */

static irqreturn_t conn_irq_probe_handler(int irq, void *data)
{
	struct mt6833_conn *conn = data;

	dev_info(conn->dev, "irqprobe: fired irq=%d WFDMA_INT=%#010x SW_IRQ=%#010x\n",
		 irq,
		 readl(conn->win[MT6833_CONN_WIN_MCU_EMI] + 0x200),
		 readl(conn->win[MT6833_CONN_WIN_ON] + 0x150));
	return IRQ_HANDLED;
}

static int conn_irq_probe_setup(struct mt6833_conn *conn)
{
	struct device_node *np;
	int irq, ret;
	u32 hwirq = 0;

	np = of_find_compatible_node(NULL, NULL, "mediatek,wifi");
	if (!np) {
		dev_warn(conn->dev, "irqprobe: no mediatek,wifi node\n");
		return -ENODEV;
	}

	irq = irq_of_parse_and_map(np, 0);
	of_node_put(np);
	dev_info(conn->dev, "irqprobe: wifi node maps to virq %d\n", irq);

	if (!irq) {
		dev_warn(conn->dev, "irqprobe: no irq mapped\n");
		return -EINVAL;
	}

	ret = request_irq(irq, conn_irq_probe_handler,
			  IRQF_SHARED | IRQF_NO_THREAD,
			  "conn-irqprobe", conn);
	dev_info(conn->dev, "irqprobe: request_irq(%d) -> %d\n", irq, ret);
	return ret;
}


/* ------------------------------------------------------------------ */
/* Minimal WMT/STP over BTIF.                                          */
/* ------------------------------------------------------------------ */

#define WMT_FRAG_SIZE	1000

static const u8 wmt_clk_en[] = {
	0x01, 0x08, 0x10, 0x00, 0x01, 0x01, 0x00, 0x01, 0x34, 0x03,
	0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff
};
static const u8 wmt_clk_138[] = {
	0x01, 0x08, 0x10, 0x00, 0x01, 0x01, 0x00, 0x01, 0x0c, 0x01,
	0x00, 0x80, 0x59, 0x4d, 0x84, 0x00, 0xff, 0xff, 0xff, 0xff
};
static const u8 wmt_reset[] = { 0x01, 0x07, 0x01, 0x00, 0x04 };
static const u8 wmt_clk_26[] = {
	0x01, 0x08, 0x10, 0x00, 0x01, 0x01, 0x00, 0x01, 0x0c, 0x01,
	0x00, 0x80, 0x00, 0x4d, 0x84, 0x00, 0xff, 0xff, 0xff, 0xff
};
static const u8 wmt_clk_dis[] = {
	0x01, 0x08, 0x10, 0x00, 0x01, 0x01, 0x00, 0x01, 0x34, 0x03,
	0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
};
static const u8 wmt_chipid[] = { 0x01, 0x02, 0x05, 0x00, 0x11, 0x33, 0x68, 0x00, 0x00 };
static const u8 wmt_adie[] = { 0x01, 0x02, 0x01, 0x00, 0x12 };
static const u8 wmt_set_stp[] = {
	0x01, 0x04, 0x05, 0x00, 0x03, 0xdb, 0x0e, 0x68, 0x01
};
static const u8 wmt_ant_swap[] = {
	0x01, 0x14, 0x04, 0x00, 0x07, 0x02, 0x00, 0x00
};
static const u8 wmt_rf_cal[] = { 0x01, 0x14, 0x01, 0x00, 0x01 };

#define WMT_FRAME_MAX  (WMT_FRAG_SIZE + 16)

static int btif_tx(void __iomem *b, const u8 *payload, int len)
{
	u8 *frame;
	int n = 0, i, guard;

	if (len < 0 || len + 16 > WMT_FRAME_MAX)
		return -EINVAL;

	frame = kmalloc(WMT_FRAME_MAX, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;

	frame[n++] = 0x80;
	frame[n++] = (u8)((4 << 4) | ((len >> 8) & 0xF));
	frame[n++] = (u8)(len & 0xFF);
	frame[n++] = 0x00;
	for (i = 0; i < len; i++)
		frame[n++] = payload[i];
	frame[n++] = 0x00;
	frame[n++] = 0x00;

	for (i = 0; i < n; i++) {
		guard = 20000;
		while (!(readl(b + BTIF_LSR) & (BTIF_LSR_THRE | BTIF_LSR_TEMT))) {
			if (!--guard) {
				kfree(frame);
				return -ETIMEDOUT;
			}
			udelay(1);
		}
		writeb(frame[i], b + 0x00);
	}
	kfree(frame);
	return n;
}

static int btif_rx(void __iomem *b, u8 *buf, int max, int ms)
{
	int got = 0, i;

	for (i = 0; i < ms * 100; i++) {
		if (readl(b + BTIF_LSR) & BTIF_LSR_DR) {
			buf[got++] = readb(b + 0x00);
			if (got >= max)
				break;
		} else if (got) {
			udelay(10);
		} else {
			udelay(10);
		}
	}
	return got;
}


/*
 * Port of the vendor's hal_btif_hw_init().  Without it the MCU only answers when
 * some earlier boot stage happened to leave the BTIF block configured.
 */
#define BTIF_IER	0x04
#define BTIF_FIFOCTRL	0x08
#define BTIF_FAKELCR	0x0c
#define BTIF_DMA_EN	0x4c
#define BTIF_TRI_LVL	0x60
#define BTIF_HANDSHAKE	0x6c

static void btif_hw_init(void __iomem *b)
{
	u32 v;

	/* fake LCR: normal mode */
	writel(0, b + BTIF_FAKELCR);

	/* new handshake mode */
	v = readl(b + BTIF_HANDSHAKE);
	writel(v | 0x1, b + BTIF_HANDSHAKE);

	/* flush the RX and TX FIFOs (set the clear bit, then drop it) */
	v = readl(b + BTIF_FIFOCTRL);
	writel(v | 0x2, b + BTIF_FIFOCTRL);
	writel(readl(b + BTIF_FIFOCTRL) & ~0x2u, b + BTIF_FIFOCTRL);
	writel(readl(b + BTIF_FIFOCTRL) | 0x4, b + BTIF_FIFOCTRL);
	writel(readl(b + BTIF_FIFOCTRL) & ~0x4u, b + BTIF_FIFOCTRL);

	/* TX threshold 8, RX threshold 1, no loopback */
	writel(((8 & 0xf) << 0) | ((1 & 0x7) << 4), b + BTIF_TRI_LVL);

	/* PIO mode, auto reset on */
	v = readl(b + BTIF_DMA_EN);
	writel(v | 0x4, b + BTIF_DMA_EN);

	/* TX interrupt off, RX interrupt on */
	writel(0, b + BTIF_IER);
	writel(readl(b + BTIF_IER) | 0x1, b + BTIF_IER);
}


static void hexdump_stp(const char *tag, const u8 *p, int n)
{
	char line[128];
	int i, off = 0;

	if (n <= 0) {
		pr_info("stp: %s <empty>\n", tag);
		return;
	}
	off += scnprintf(line + off, sizeof(line) - off, "stp: %s [%d]:", tag, n);
	for (i = 0; i < n && off < (int)sizeof(line) - 5; i++)
		off += scnprintf(line + off, sizeof(line) - off, " %02x", p[i]);
	pr_info("%s\n", line);
}

static void conn_stp_run(struct mt6833_conn *conn)
{
	void __iomem *b = ioremap(CONN_BTIF_BASE, 0x1000);
	u8 rx[64];
	int n;

	if (!b) {
		dev_err(conn->dev, "stp: no BTIF\n");
		return;
	}

	conn_btif_clocks_on(conn->dev);

	btif_hw_init(b);
	dev_info(conn->dev, "stp: btif hw init done, TRI_LVL=%#x HANDSHAKE=%#x\n",
		 readl(b + BTIF_TRI_LVL), readl(b + BTIF_HANDSHAKE));
	dev_info(conn->dev, "stp: LSR=%#x\n", readl(b + BTIF_LSR));

	n = btif_tx(b, wmt_clk_en, sizeof(wmt_clk_en));
	dev_info(conn->dev, "stp: clk_en sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 200);
	dev_info(conn->dev, "stp: clk_en resp %d bytes, first %#x\n", n, n ? rx[0] : 0);
	hexdump_stp("clk_en", rx, n);

	n = btif_tx(b, wmt_clk_138, sizeof(wmt_clk_138));
	dev_info(conn->dev, "stp: clk_138 sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 200);
	dev_info(conn->dev, "stp: clk_138 resp %d bytes, first %#x\n", n, n ? rx[0] : 0);
	hexdump_stp("clk_138", rx, n);


	n = btif_tx(b, wmt_reset, sizeof(wmt_reset));
	dev_info(conn->dev, "stp: reset sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 300);
	dev_info(conn->dev, "stp: reset resp %d bytes\n", n);
	hexdump_stp("reset", rx, n);
	if (n >= 5) {
		dev_info(conn->dev, "stp: reset evt %02x %02x %02x %02x %02x\n",
			 rx[0], rx[1], rx[2], rx[3], rx[4]);
	}

	n = btif_tx(b, wmt_clk_26, sizeof(wmt_clk_26));
	dev_info(conn->dev, "stp: clk_26 sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 200);
	dev_info(conn->dev, "stp: clk_26 resp %d bytes\n", n);
	hexdump_stp("clk_26", rx, n);

	n = btif_tx(b, wmt_clk_dis, sizeof(wmt_clk_dis));
	dev_info(conn->dev, "stp: clk_dis sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 200);
	dev_info(conn->dev, "stp: clk_dis resp %d bytes\n", n);
	hexdump_stp("clk_dis", rx, n);

	n = btif_tx(b, wmt_chipid, sizeof(wmt_chipid));
	dev_info(conn->dev, "stp: chipid sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 300);
	dev_info(conn->dev, "stp: chipid resp %d bytes\n", n);
	hexdump_stp("chipid", rx, n);
	n = btif_rx(b, rx, sizeof(rx), 200);

	n = btif_tx(b, wmt_adie, sizeof(wmt_adie));
	dev_info(conn->dev, "stp: adie sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 300);
	dev_info(conn->dev, "stp: adie resp %d bytes\n", n);
	hexdump_stp("adie", rx, n);
	n = btif_rx(b, rx, sizeof(rx), 200);

	n = btif_tx(b, wmt_set_stp, sizeof(wmt_set_stp));
	dev_info(conn->dev, "stp: set_stp sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 200);
	dev_info(conn->dev, "stp: set_stp resp %d bytes\n", n);
	hexdump_stp("set_stp", rx, n);

	n = btif_tx(b, wmt_ant_swap, sizeof(wmt_ant_swap));
	dev_info(conn->dev, "stp: ant_swap sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 300);
	dev_info(conn->dev, "stp: ant_swap resp %d bytes\n", n);
	hexdump_stp("ant_swap", rx, n);

	n = btif_tx(b, wmt_rf_cal, sizeof(wmt_rf_cal));
	dev_info(conn->dev, "stp: rf_cal sent (%d)\n", n);
	n = btif_rx(b, rx, sizeof(rx), 4000);
	dev_info(conn->dev, "stp: rf_cal resp %d bytes, first %#x %#x %#x\n",
		 n, n > 0 ? rx[0] : 0, n > 1 ? rx[1] : 0, n > 2 ? rx[2] : 0);
	hexdump_stp("rf_cal", rx, n);

	dev_info(conn->dev, "stp: done, CONN_ON+0x600=%#010x CPUPCR=%#010x ON_MISC=%#010x\n",
		 readl(conn->win[MT6833_CONN_WIN_ON] + 0x600),
		 readl(conn->win[MT6833_CONN_WIN_OFF] + 0x104),
		 readl(conn->win[MT6833_CONN_WIN_MCU_CFG] + 0x1140 - 0x1000));
	iounmap(b);
}

static ssize_t conn_stp_write(struct file *f, const char __user *buf,
			      size_t len, loff_t *off)
{
	struct mt6833_conn *conn = conn_cycle_dev;

	if (!conn)
		return -ENODEV;
	dev_info(conn->dev, "stp: sequence requested\n");
	conn_stp_run(conn);
	return len;
}

static const struct proc_ops conn_stp_pops = {
	.proc_write	= conn_stp_write,
};


static const struct proc_ops conn_cycle_pops = {
	.proc_write	= conn_cycle_write,
};

static void conn_hw_off(struct mt6833_conn *conn)
{
	/* Clear CON_PWR_ON first: the block must see the request go away. */
	writel(0, conn->win[MT6833_CONN_WIN_PERICFG] + CONN_PERICFG_CON_STA_OFFSET);

	if (conn->powered)
		pm_runtime_put_sync(conn->dev);

	udelay(CONN_PWR_OFF_SETTLE_US);

	/* Drop any stale CCIF4 ACK so the next power-on starts clean. */
	if (conn->win[MT6833_CONN_WIN_AP_CCIF4])
		writel(CONN_AP_CCIF4_ACK_CLEAR,
		       conn->win[MT6833_CONN_WIN_AP_CCIF4] +
		       CONN_AP_CCIF4_ACK_OFFSET);

	clk_disable_unprepare(conn->ccif);

	if (conn->vcn33_1)
		regulator_disable(conn->vcn33_1);
	if (conn->vcn18)
		regulator_disable(conn->vcn18);

	conn->powered = false;
	conn->chip_id = 0;
}

int mt6833_conn_power_on(struct mt6833_conn *conn)
{
	int ret = 0;

	if (!conn)
		return -EINVAL;

	mutex_lock(&conn->lock);

	if (conn->dead) {
		ret = -ENODEV;
		goto unlock;
	}

	if (conn->power_users) {
		conn->power_users++;
		goto unlock;
	}

	ret = conn_hw_on(conn);
	if (!ret)
		conn->power_users = 1;

unlock:
	mutex_unlock(&conn->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(mt6833_conn_power_on);

void mt6833_conn_power_off(struct mt6833_conn *conn)
{
	if (!conn)
		return;

	mutex_lock(&conn->lock);

	if (!conn->dead && conn->power_users && !--conn->power_users)
		conn_hw_off(conn);

	mutex_unlock(&conn->lock);
}
EXPORT_SYMBOL_GPL(mt6833_conn_power_off);

/* Same sequence as /proc/conn_cycle: a real off/on, ignoring the refcount. */
void mt6833_conn_force_cycle(struct mt6833_conn *conn)
{
	if (!conn || conn->dead)
		return;

	mutex_lock(&conn->lock);
	conn_hw_off(conn);
	conn->power_users = 0;
	msleep(50);
	if (!conn_hw_on(conn))
		conn->power_users = 1;
	/* force_cycle runs STP: the MCU only serves the driver's commands once the
	 * WMT/STP initialisation has been done, and the vendor does it before
	 * wlanProbe too.
	 */
	if (stp_before_probe)
		conn_stp_run(conn);
	mutex_unlock(&conn->lock);
}
EXPORT_SYMBOL_GPL(mt6833_conn_force_cycle);

unsigned int mt6833_conn_power_users(struct mt6833_conn *conn)
{
	return conn ? READ_ONCE(conn->power_users) : 0;
}
EXPORT_SYMBOL_GPL(mt6833_conn_power_users);

bool mt6833_conn_is_up(struct mt6833_conn *conn)
{
	u32 val;

	if (!conn || !conn->win[MT6833_CONN_WIN_MCU_CFG])
		return false;

	if (!READ_ONCE(conn->powered))
		return false;

	val = readl(conn->win[MT6833_CONN_WIN_MCU_CFG] + CONN_CHIP_ID_OFFSET);

	return mt6833_conn_chip_id_valid(val);
}
EXPORT_SYMBOL_GPL(mt6833_conn_is_up);

/* ------------------------------------------------------------------------- */
/* Probe / remove							     */
/* ------------------------------------------------------------------------- */

static int mt6833_conn_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mt6833_conn *conn;
	unsigned int i;
	int ret;

	conn = kzalloc_obj(*conn, GFP_KERNEL);
	if (!conn)
		return -ENOMEM;

	dev_info(dev, "probe: enter\n");
	conn->dev = dev;
	mutex_init(&conn->lock);
	kref_init(&conn->ref);

	for (i = 0; i < MT6833_CONN_WIN_MAX; i++) {
		conn->win[i] = of_iomap(np, i);
		if (conn->win[i])
			continue;

		if (conn_window_required(i)) {
			dev_err(dev, "cannot map the %s window (reg index %u)\n",
				conn_window_name(i), i);
			ret = -ENOMEM;
			goto err_free;
		}

		dev_dbg(dev, "optional %s window (reg index %u) not present\n",
			conn_window_name(i), i);
	}

	conn->ccif = devm_clk_get(dev, "ccif");
	if (IS_ERR(conn->ccif)) {
		ret = dev_err_probe(dev, PTR_ERR(conn->ccif),
				    "no ccif clock\n");
		goto err_free;
	}

	/*
	 * Optional supply lookup: -ENODEV means "this board does not wire the
	 * rail to us", which is a supported configuration (see conn_hw_on).
	 */
	conn->vcn18 = devm_regulator_get_optional(dev, "vcn18");
	if (IS_ERR(conn->vcn18)) {
		ret = PTR_ERR(conn->vcn18);
		if (ret != -ENODEV) {
			dev_err_probe(dev, ret, "cannot get vcn18\n");
			goto err_free;
		}
		conn->vcn18 = NULL;
	}

	conn->vcn33_1 = devm_regulator_get_optional(dev, "vcn33_1");
	conn->vcn33_2_wifi = devm_regulator_get_optional(dev, "vcn33_2_wifi");
	if (IS_ERR(conn->vcn33_2_wifi)) {
		if (PTR_ERR(conn->vcn33_2_wifi) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		conn->vcn33_2_wifi = NULL;
	}
	if (IS_ERR(conn->vcn33_1)) {
		ret = PTR_ERR(conn->vcn33_1);
		if (ret != -ENODEV) {
			dev_err_probe(dev, ret, "cannot get vcn33_1\n");
			goto err_free;
		}
		conn->vcn33_1 = NULL;
	}

	pm_runtime_enable(dev);

	conn_cycle_dev = conn;
	conn_irq_probe_setup(conn);
	proc_create("conn_cycle", 0200, NULL, &conn_cycle_pops);
	proc_create("conn_stp", 0200, NULL, &conn_stp_pops);
	proc_create("conn_wmt", 0200, NULL, &conn_wmt_pops);
	proc_create("conn_reg", 0200, NULL, &conn_reg_pops);
	proc_create("conn_sweep", 0200, NULL, &conn_sweep_pops);
	platform_set_drvdata(pdev, conn);

	mutex_lock(&mt6833_conn_instance_lock);
	if (mt6833_conn_instance) {
		mutex_unlock(&mt6833_conn_instance_lock);
		dev_err(dev, "a second CONN instance is not supported\n");
		ret = -EBUSY;
		goto err_pm;
	}
	mt6833_conn_instance = conn;
	mutex_unlock(&mt6833_conn_instance_lock);

	dev_info(dev, "MT6833 CONN subsystem, windows mapped%s%s\n",
		 conn->vcn18 ? ", vcn18" : "", conn->vcn33_1 ? ", vcn33_1" : "");

	/*
	 * Opt-in bring-up self test: power the block, print the id, power it off
	 * again.  Off by default because it costs a full on/off cycle at boot;
	 * enable with "mediatek,probe-selftest" when debugging a new board.
	 */
	if (device_property_read_bool(dev, "mediatek,probe-selftest")) {
		ret = mt6833_conn_power_on(conn);
		if (ret) {
			dev_err(dev, "self test: power-on failed: %d\n", ret);
		} else {
			dev_info(dev, "self test: chip id %#010x\n", conn->chip_id);
			mt6833_conn_check_clock(conn);
			/*
			 * BRING-UP ACCOMMODATION, not the finished behaviour: the
			 * reference is deliberately NOT dropped here.  A cold power-on
			 * works, but power-off followed by power-on reads the chip id
			 * back as 0x00030001 (measured), because this sequence does not
			 * re-arm the CONN MCU reset the way the bootloader/vendor
			 * sequence does.  Keeping the block up hands this reference to
			 * the first consumer (the WLAN driver), so its power_on() only
			 * bumps the count.  The block therefore stays powered for the
			 * life of the system until a real off->reset->on sequence is
			 * implemented.
			 */
			dev_info(dev, "self test: keeping the CONN block powered (bring-up accommodation)\n");
		}
	}

	/*
	 * Hand the MCU its WMT bring-up right here rather than waiting for
	 * someone to poke /proc/conn_wmt.  Without it the Bluetooth core
	 * answers a bare HCI Reset with an EXCEPTION and the HCI driver times
	 * out, because WMT is what switches the individual subsystems on.
	 */
	conn_wmt_kick(conn);

	return 0;

err_pm:
	pm_runtime_disable(dev);
err_free:
	mt6833_conn_unmap_all(conn);
	kfree(conn);

	return ret;
}

static void mt6833_conn_remove(struct platform_device *pdev)
{
	struct mt6833_conn *conn = platform_get_drvdata(pdev);

	if (!conn)
		return;

	mutex_lock(&mt6833_conn_instance_lock);
	if (mt6833_conn_instance == conn)
		mt6833_conn_instance = NULL;
	mutex_unlock(&mt6833_conn_instance_lock);

	mutex_lock(&conn->lock);
	conn->dead = true;
	if (conn->power_users) {
		conn->power_users = 0;
		conn_hw_off(conn);
	}
	mutex_unlock(&conn->lock);

	if (kref_read(&conn->ref) > 1)
		dev_warn(&pdev->dev,
			 "removed with %u outstanding CONN handles\n",
			 kref_read(&conn->ref) - 1);

	pm_runtime_disable(&pdev->dev);
	mt6833_conn_put(conn);
}

static const struct of_device_id mt6833_conn_of_match[] = {
	{ .compatible = "mediatek,mt6833-consys" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6833_conn_of_match);

static struct platform_driver mt6833_conn_driver = {
	.probe = mt6833_conn_probe,
	.remove = mt6833_conn_remove,
	.driver = {
		.name = "mt6833-conn",
		.of_match_table = mt6833_conn_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mt6833_conn_driver);

MODULE_DESCRIPTION("MediaTek MT6833 integrated CONN subsystem driver");
MODULE_LICENSE("GPL");
