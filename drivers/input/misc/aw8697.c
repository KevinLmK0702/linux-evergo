// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW8697 / AW8695 (AW869X family) LRA haptic driver.
 *
 * The evergo uses an AW8697 long-range actuator on i2c9 at address 0x5a,
 * with the reset line on GPIO132 and the interrupt on GPIO6.  The vendor
 * driver exposes the usual awinic "vib_*" tuneables from the device tree;
 * this driver keeps the subset needed to (a) bring the chip up and
 * (b) render EV_FF rumble/periodic effects through its continuous (CONT)
 * playback mode.  Waveform-RAM/RTP effects are not implemented.
 *
 * Register layout and the init sequence are taken from the vendor
 * AW869X driver (aw869x.c, GPL-2.0) used in the stock kernel.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define AW8697_I2C_NAME			"aw8697"

#define AW8697_REG_ID			0x00
#define AW8697_REG_SYSST		0x01
#define AW8697_REG_SYSINT		0x02
#define AW8697_REG_SYSINTM		0x03
#define AW8697_REG_SYSCTRL		0x04
#define AW8697_REG_GO			0x05
#define AW8697_REG_DATCTRL		0x2b
#define AW8697_REG_PWMPRC		0x2d
#define AW8697_REG_PWMDBG		0x2e
#define AW8697_REG_BSTDBG1		0x31
#define AW8697_REG_BSTDBG2		0x32
#define AW8697_REG_BSTDBG3		0x33
#define AW8697_REG_BSTCFG		0x34
#define AW8697_REG_ANADBG		0x35
#define AW8697_REG_ANACTRL		0x36
#define AW8697_REG_DATDBG		0x39
#define AW8697_REG_BSTDBG4		0x3a
#define AW8697_REG_PRLVL		0x3e
#define AW8697_REG_PRTIME		0x3f
#define AW8697_REG_GLB_STATE		0x46
#define AW8697_REG_BST_AUTO		0x47
#define AW8697_REG_CONT_CTRL		0x48
#define AW8697_REG_F_PRE_H		0x49
#define AW8697_REG_F_PRE_L		0x4a
#define AW8697_REG_TD_H			0x4b
#define AW8697_REG_TD_L			0x4c
#define AW8697_REG_TSET			0x4d
#define AW8697_REG_R_SPARE		0x5d
#define AW8697_REG_DETCTRL		0x5f
#define AW8697_REG_VBATDET		0x62
#define AW8697_REG_ADCTEST		0x66
#define AW8697_REG_BEMF_VTHH_H		0x74
#define AW8697_REG_BEMF_VTHH_L		0x75
#define AW8697_REG_BEMF_VTHL_H		0x76
#define AW8697_REG_BEMF_VTHL_L		0x77
#define AW8697_REG_ZC_THRSH_H		0x72
#define AW8697_REG_ZC_THRSH_L		0x73
#define AW8697_REG_BEMF_NUM		0x78
#define AW8697_REG_TIME_NZC		0x7a
#define AW8697_REG_DRV_LVL		0x7b
#define AW8697_REG_DRV_LVL_OV		0x7c

/* SYSCTRL: reg 0x04 */
#define SYSCTRL_WORK_MODE_MASK		GENMASK(0, 0)
#define SYSCTRL_ACTIVE			0
#define SYSCTRL_STANDBY			BIT(0)
#define SYSCTRL_BST_MODE_MASK		GENMASK(1, 1)
#define SYSCTRL_BST_MODE_BYPASS		0
#define SYSCTRL_BST_MODE_BOOST		BIT(1)
#define SYSCTRL_PLAY_MODE_MASK		GENMASK(3, 2)
#define SYSCTRL_PLAY_MODE_RAM		0
#define SYSCTRL_PLAY_MODE_CONT		(2 << 2)
#define SYSCTRL_RAMINIT_MASK		GENMASK(5, 5)
#define SYSCTRL_RAMINIT_EN		BIT(5)

/* SYSINTM: reg 0x03 */
#define SYSINTM_UVLO_MASK		GENMASK(5, 5)
#define SYSINTM_UVLO_OFF		BIT(5)
#define SYSINTM_UVLO_EN			0

/* GO: reg 0x05 */
#define GO_MASK				GENMASK(0, 0)
#define GO_ENABLE			BIT(0)

/* DATCTRL: reg 0x2b */
#define DATCTRL_FC_MASK			GENMASK(7, 6)
#define DATCTRL_FC_1000HZ		(3 << 6)
#define DATCTRL_LPF_ENABLE_MASK		GENMASK(5, 5)
#define DATCTRL_LPF_ENABLE		BIT(5)

/* PWMDBG: reg 0x2e */
#define PWMDBG_PWM_MODE_MASK		GENMASK(6, 5)
#define PWMDBG_PWM_24K			(2 << 5)

/* BSTCFG: reg 0x34 */
#define BSTCFG_PEAKCUR_MASK		GENMASK(2, 0)
#define BSTCFG_PEAKCUR_3P5A		(5 << 0)

/* ANACTRL: reg 0x36 */
#define ANACTRL_LRA_SRC_MASK		GENMASK(5, 5)
#define ANACTRL_LRA_SRC_REG		BIT(5)

/* ANADBG: reg 0x35 */
#define ANADBG_IOC_MASK			GENMASK(3, 2)
#define ANADBG_IOC_4P65A		(3 << 2)

/* BSTDBG4: reg 0x3a */
#define BSTDBG4_BSTVOL_MASK		GENMASK(5, 1)

/* PWMPRC / PRLVL: reg 0x2d / 0x3e */
#define PWMPRC_PRC_MASK			GENMASK(7, 7)
#define PWMPRC_PRC_ENABLE		BIT(7)
#define PRLVL_PR_MASK			GENMASK(7, 7)
#define PRLVL_PR_ENABLE			BIT(7)

/* BST_AUTO: reg 0x47 */
#define BST_AUTO_AUTOSW_MASK		GENMASK(2, 2)
#define BST_AUTO_MANUAL_BOOST		0
#define BST_AUTO_RTP_MASK		GENMASK(1, 1)
#define BST_AUTO_RTP_DISABLE		0
#define BST_AUTO_RAM_MASK		GENMASK(0, 0)
#define BST_AUTO_RAM_DISABLE		0

/* CONT_CTRL: reg 0x48 */
#define CONT_CTRL_ZC_DETEC_MASK		GENMASK(7, 7)
#define CONT_CTRL_ZC_DETEC_ENABLE	BIT(7)
#define CONT_CTRL_WAIT_PERIOD_MASK	GENMASK(6, 5)
#define CONT_CTRL_WAIT_1PERIOD		0
#define CONT_CTRL_MODE_MASK		GENMASK(4, 4)
#define CONT_CTRL_BY_GO_SIGNAL		0
#define CONT_CTRL_EN_CLOSE_MASK		GENMASK(3, 3)
#define CONT_CTRL_CLOSE_PLAYBACK	BIT(3)
#define CONT_CTRL_F0_DETECT_MASK	GENMASK(2, 2)
#define CONT_CTRL_F0_DETECT_DISABLE	0
#define CONT_CTRL_O2C_MASK		GENMASK(1, 1)
#define CONT_CTRL_O2C_DISABLE		0
#define CONT_CTRL_AUTO_BRK_MASK		GENMASK(0, 0)
#define CONT_CTRL_AUTO_BRK_ENABLE	BIT(0)

/* DETCTRL: reg 0x5f */
#define DETCTRL_PROTECT_MASK		GENMASK(5, 5)
#define DETCTRL_PROTECT_SHUTDOWN	0
#define DETCTRL_DIAG_GO_MASK		GENMASK(0, 0)
#define DETCTRL_DIAG_GO_ENABLE		BIT(0)

/* ADCTEST: reg 0x66 */
#define ADCTEST_VBAT_MODE_MASK		GENMASK(6, 6)
#define ADCTEST_VBAT_HW_COMP		BIT(6)

/* BEMF_NUM: reg 0x78 */
#define BEMF_NUM_BRK_MASK		GENMASK(3, 0)

#define AW8697_CHIP_ID			0x97
#define AW8697_CHIP_ID_8695		0x95
#define AW8697_CHIP_ID_86907		0x04

#define AW8697_DEFAULT_F0_PRE		1700
#define AW8697_DEFAULT_F0_COEFF		260
#define AW8697_DEFAULT_CONT_DRV_LVL	0x35
#define AW8697_DEFAULT_CONT_DRV_LVL_OV	0x35
#define AW8697_DEFAULT_CONT_TD		0x009a
#define AW8697_DEFAULT_CONT_ZC_THR	0x0ff1
#define AW8697_DEFAULT_CONT_NUM_BRK	3
#define AW8697_DEFAULT_TSET		0x12
#define AW8697_DEFAULT_R_SPARE		0x68
#define AW8697_DEFAULT_BST_VOL		0x10
#define AW8697_MAX_F0_REG		0xffff

struct aw8697 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct input_dev *input;
	struct mutex lock;
	struct work_struct play_work;
	unsigned int level;

	u32 f0_pre;
	u32 f0_coeff;
	u32 cont_drv_lvl;
	u32 cont_drv_lvl_ov;
	u32 cont_td;
	u32 cont_zc_thr;
	u32 cont_num_brk;
	u32 tset;
	u32 r_spare;
	u32 bst_vol;
	u32 bstdbg[3];
	u32 bemf_cfg[4];
};

static const struct regmap_config aw8697_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static int aw8697_write_bits(struct aw8697 *h, unsigned int reg,
			     unsigned int mask, unsigned int val)
{
	return regmap_update_bits(h->regmap, reg, mask, val);
}

static int aw8697_play_mode(struct aw8697 *h, unsigned int mode)
{
	int ret;

	switch (mode) {
	case 0:					/* standby */
		ret = aw8697_write_bits(h, AW8697_REG_SYSINTM,
					SYSINTM_UVLO_MASK, SYSINTM_UVLO_OFF);
		if (ret)
			return ret;
		return aw8697_write_bits(h, AW8697_REG_SYSCTRL,
					 SYSCTRL_WORK_MODE_MASK, SYSCTRL_STANDBY);
	case 2:					/* continuous */
		ret = aw8697_write_bits(h, AW8697_REG_SYSCTRL,
					SYSCTRL_PLAY_MODE_MASK,
					SYSCTRL_PLAY_MODE_CONT);
		if (ret)
			return ret;
		ret = aw8697_write_bits(h, AW8697_REG_SYSCTRL,
					SYSCTRL_BST_MODE_MASK,
					SYSCTRL_BST_MODE_BYPASS);
		if (ret)
			return ret;
		ret = aw8697_write_bits(h, AW8697_REG_SYSCTRL,
					SYSCTRL_WORK_MODE_MASK, SYSCTRL_ACTIVE);
		if (ret)
			return ret;
		return aw8697_write_bits(h, AW8697_REG_SYSINTM,
					 SYSINTM_UVLO_MASK, SYSINTM_UVLO_EN);
	default:
		return -EINVAL;
	}
}

static int aw8697_wait_standby(struct aw8697 *h)
{
	unsigned int val;
	int ret, i;

	for (i = 0; i < 100; i++) {
		ret = regmap_read(h->regmap, AW8697_REG_GLB_STATE, &val);
		if (ret)
			return ret;
		if ((val & 0x0f) == 0)
			return 0;
		usleep_range(2000, 2500);
	}
	return -ETIMEDOUT;
}

static int aw8697_stop(struct aw8697 *h)
{
	int ret;

	ret = aw8697_write_bits(h, AW8697_REG_GO, GO_MASK, 0);
	if (ret)
		return ret;
	ret = aw8697_wait_standby(h);
	if (ret)
		dev_dbg(h->dev, "did not reach standby: %d\n", ret);
	return aw8697_play_mode(h, 0);
}

static int aw8697_start_cont(struct aw8697 *h, unsigned int level)
{
	unsigned int drv;
	int ret;

	ret = aw8697_stop(h);
	if (ret)
		return ret;

	ret = aw8697_play_mode(h, 2);
	if (ret)
		return ret;

	/* low-pass filter and zero-cross settings */
	ret = aw8697_write_bits(h, AW8697_REG_DATCTRL, DATCTRL_FC_MASK,
				DATCTRL_FC_1000HZ);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_DATCTRL,
				DATCTRL_LPF_ENABLE_MASK, DATCTRL_LPF_ENABLE);
	if (ret)
		return ret;

	ret = aw8697_write_bits(h, AW8697_REG_CONT_CTRL,
				CONT_CTRL_ZC_DETEC_MASK, CONT_CTRL_ZC_DETEC_ENABLE);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_CONT_CTRL,
				CONT_CTRL_WAIT_PERIOD_MASK, CONT_CTRL_WAIT_1PERIOD);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_CONT_CTRL, CONT_CTRL_MODE_MASK,
				CONT_CTRL_BY_GO_SIGNAL);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_CONT_CTRL,
				CONT_CTRL_EN_CLOSE_MASK, CONT_CTRL_CLOSE_PLAYBACK);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_CONT_CTRL,
				CONT_CTRL_F0_DETECT_MASK,
				CONT_CTRL_F0_DETECT_DISABLE);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_CONT_CTRL, CONT_CTRL_O2C_MASK,
				CONT_CTRL_O2C_DISABLE);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_CONT_CTRL,
				CONT_CTRL_AUTO_BRK_MASK, CONT_CTRL_AUTO_BRK_ENABLE);
	if (ret)
		return ret;

	ret = regmap_write(h->regmap, AW8697_REG_TD_H, h->cont_td >> 8);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_TD_L, h->cont_td & 0xff);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_TSET, h->tset);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_ZC_THRSH_H,
			   h->cont_zc_thr >> 8);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_ZC_THRSH_L,
			   h->cont_zc_thr & 0xff);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_BEMF_NUM, BEMF_NUM_BRK_MASK,
				h->cont_num_brk);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_TIME_NZC, 0x23);
	if (ret)
		return ret;

	/* driver level follows the requested rumble magnitude */
	/*
	 * Keep a useful floor for a rumble/periodic event: an effect at half
	 * magnitude (0x7fff) otherwise maps to half drive and is barely felt.
	 */
	drv = 0x38 + (h->cont_drv_lvl - 0x38) * min(level, 0xffffU) / 0xffff;
	if (drv < 0x18)
		drv = 0x18;
	else if (drv > h->cont_drv_lvl)
		drv = h->cont_drv_lvl;
	ret = regmap_write(h->regmap, AW8697_REG_DRV_LVL, drv);
	if (ret)
		return ret;
	return regmap_write(h->regmap, AW8697_REG_DRV_LVL_OV, drv);
}

static void aw8697_play_work(struct work_struct *work)
{
	struct aw8697 *h = container_of(work, struct aw8697, play_work);
	int ret;

	mutex_lock(&h->lock);
	if (h->level)
		ret = aw8697_start_cont(h, h->level);
	else
		ret = aw8697_stop(h);
	mutex_unlock(&h->lock);

	if (ret)
		dev_err(h->dev, "haptic playback failed: %d\n", ret);
}

static int aw8697_play_effect(struct input_dev *dev, void *data,
			      struct ff_effect *effect)
{
	struct aw8697 *h = input_get_drvdata(dev);
	unsigned int level;

	if (effect->type != FF_RUMBLE && effect->type != FF_PERIODIC)
		return 0;

	if (effect->type == FF_RUMBLE) {
		level = max(effect->u.rumble.strong_magnitude,
			    effect->u.rumble.weak_magnitude);
	} else {
		level = effect->u.periodic.magnitude;
	}

	if (level == h->level)
		return 0;

	h->level = level;
	schedule_work(&h->play_work);

	return 0;
}

static int aw8697_hw_reset(struct aw8697 *h)
{
	if (!h->reset_gpio)
		return 0;

	/* active-low reset: logical 1 asserts, 0 releases */
	gpiod_set_value_cansleep(h->reset_gpio, 1);
	usleep_range(5000, 5500);
	gpiod_set_value_cansleep(h->reset_gpio, 0);
	usleep_range(8000, 8500);

	return 0;
}

static int aw8697_soft_reset(struct aw8697 *h)
{
	int ret;

	ret = regmap_write(h->regmap, AW8697_REG_ID, 0xaa);
	if (ret)
		return ret;
	usleep_range(2000, 2500);
	return 0;
}

static int aw8697_detect(struct aw8697 *h)
{
	unsigned int id;
	int ret;

	ret = regmap_read(h->regmap, AW8697_REG_ID, &id);
	if (ret)
		return ret;

	switch (id) {
	case AW8697_CHIP_ID:
	case AW8697_CHIP_ID_8695:
	case AW8697_CHIP_ID_86907:
		dev_info(h->dev, "detected AW869X chip id 0x%02x\n", id);
		return 0;
	default:
		dev_err(h->dev, "unsupported chip id 0x%02x\n", id);
		return -ENODEV;
	}
}

static int aw8697_offset_calibration(struct aw8697 *h)
{
	unsigned int val;
	int ret, i;

	ret = aw8697_write_bits(h, AW8697_REG_SYSCTRL, SYSCTRL_RAMINIT_MASK,
				SYSCTRL_RAMINIT_EN);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_DETCTRL, DETCTRL_DIAG_GO_MASK,
				DETCTRL_DIAG_GO_ENABLE);
	if (ret)
		return ret;
	for (i = 0; i < 100; i++) {
		ret = regmap_read(h->regmap, AW8697_REG_DETCTRL, &val);
		if (ret)
			return ret;
		if (!(val & 0x01))
			break;
		usleep_range(1000, 1500);
	}
	return aw8697_write_bits(h, AW8697_REG_SYSCTRL, SYSCTRL_RAMINIT_MASK, 0);
}

static int aw8697_init_chip(struct aw8697 *h)
{
	unsigned int f0_reg;
	int ret;

	/* standby + 24 kHz PWM */
	ret = aw8697_play_mode(h, 0);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_PWMDBG, PWMDBG_PWM_MODE_MASK,
				PWMDBG_PWM_24K);
	if (ret)
		return ret;

	/* LRA trim source = register, analog settings from the vendor init */
	ret = aw8697_write_bits(h, AW8697_REG_ANACTRL, ANACTRL_LRA_SRC_MASK,
				ANACTRL_LRA_SRC_REG);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_BSTDBG1, h->bstdbg[0]);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_BSTDBG2, h->bstdbg[1]);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_BSTDBG3, h->bstdbg[2]);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_TSET, h->tset);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_R_SPARE, h->r_spare);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_ANADBG, ANADBG_IOC_MASK,
				ANADBG_IOC_4P65A);
	if (ret)
		return ret;

	/* boost peak current 3.5 A (AW8697 default) */
	ret = aw8697_write_bits(h, AW8697_REG_BSTCFG, BSTCFG_PEAKCUR_MASK,
				BSTCFG_PEAKCUR_3P5A);
	if (ret)
		return ret;

	/* motor protection: shutdown on over-current */
	ret = aw8697_write_bits(h, AW8697_REG_DETCTRL, DETCTRL_PROTECT_MASK,
				DETCTRL_PROTECT_SHUTDOWN);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_PWMPRC, PWMPRC_PRC_MASK,
				PWMPRC_PRC_ENABLE);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_PRLVL, PRLVL_PR_MASK,
				PRLVL_PR_ENABLE);
	if (ret)
		return ret;

	/* no automatic boost switching */
	ret = aw8697_write_bits(h, AW8697_REG_BST_AUTO, BST_AUTO_AUTOSW_MASK,
				BST_AUTO_MANUAL_BOOST);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_BST_AUTO, BST_AUTO_RTP_MASK,
				BST_AUTO_RTP_DISABLE);
	if (ret)
		return ret;
	ret = aw8697_write_bits(h, AW8697_REG_BST_AUTO, BST_AUTO_RAM_MASK,
				BST_AUTO_RAM_DISABLE);
	if (ret)
		return ret;

	ret = aw8697_offset_calibration(h);
	if (ret)
		return ret;

	/* hardware VBAT compensation in continuous mode */
	ret = aw8697_write_bits(h, AW8697_REG_ADCTEST, ADCTEST_VBAT_MODE_MASK,
				ADCTEST_VBAT_HW_COMP);
	if (ret)
		return ret;

	/* BEMF thresholds from the board tuneables */
	ret = regmap_write(h->regmap, AW8697_REG_BEMF_VTHH_H, h->bemf_cfg[0]);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_BEMF_VTHH_L, h->bemf_cfg[1]);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_BEMF_VTHL_H, h->bemf_cfg[2]);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_BEMF_VTHL_L, h->bemf_cfg[3]);
	if (ret)
		return ret;

	/* preset resonance frequency: f0_reg = 1e9 / (f0_pre * f0_coeff) */
	f0_reg = 1000000000ULL / ((u64)h->f0_pre * h->f0_coeff);
	if (f0_reg > AW8697_MAX_F0_REG)
		f0_reg = AW8697_MAX_F0_REG;
	ret = regmap_write(h->regmap, AW8697_REG_F_PRE_H, f0_reg >> 8);
	if (ret)
		return ret;
	ret = regmap_write(h->regmap, AW8697_REG_F_PRE_L, f0_reg & 0xff);
	if (ret)
		return ret;

	/* boost voltage */
	return aw8697_write_bits(h, AW8697_REG_BSTDBG4, BSTDBG4_BSTVOL_MASK,
				 (h->bst_vol << 1) & BSTDBG4_BSTVOL_MASK);
}

static int aw8697_parse_dt(struct aw8697 *h)
{
	struct device_node *np = h->dev->of_node;

	h->f0_pre = AW8697_DEFAULT_F0_PRE;
	h->f0_coeff = AW8697_DEFAULT_F0_COEFF;
	h->cont_drv_lvl = AW8697_DEFAULT_CONT_DRV_LVL;
	h->cont_drv_lvl_ov = AW8697_DEFAULT_CONT_DRV_LVL_OV;
	h->cont_td = AW8697_DEFAULT_CONT_TD;
	h->cont_zc_thr = AW8697_DEFAULT_CONT_ZC_THR;
	h->cont_num_brk = AW8697_DEFAULT_CONT_NUM_BRK;
	h->tset = AW8697_DEFAULT_TSET;
	h->r_spare = AW8697_DEFAULT_R_SPARE;
	h->bst_vol = AW8697_DEFAULT_BST_VOL;
	h->bstdbg[0] = 0x30;
	h->bstdbg[1] = 0xeb;
	h->bstdbg[2] = 0xd4;
	h->bemf_cfg[0] = 0x10;
	h->bemf_cfg[1] = 0x08;
	h->bemf_cfg[2] = 0x03;
	h->bemf_cfg[3] = 0xf8;

	device_property_read_u32(h->dev, "vib_f0_pre", &h->f0_pre);
	device_property_read_u32(h->dev, "vib_f0_coeff", &h->f0_coeff);
	device_property_read_u32(h->dev, "vib_cont_drv_lev",
				 &h->cont_drv_lvl);
	device_property_read_u32(h->dev, "vib_cont_drv_lvl_ov",
				 &h->cont_drv_lvl_ov);
	device_property_read_u32(h->dev, "vib_cont_td", &h->cont_td);
	device_property_read_u32(h->dev, "vib_cont_zc_thr",
				 &h->cont_zc_thr);
	device_property_read_u32(h->dev, "vib_cont_num_brk",
				 &h->cont_num_brk);
	device_property_read_u32(h->dev, "vib_tset", &h->tset);
	device_property_read_u32(h->dev, "vib_r_spare", &h->r_spare);
	device_property_read_u32(h->dev, "vib_bst_vol_default",
				 &h->bst_vol);
	device_property_read_u32_array(h->dev, "vib_bstdbg", h->bstdbg, 3);
	device_property_read_u32_array(h->dev, "vib_bemf_config",
				       h->bemf_cfg, 4);

	if (!h->f0_pre || !h->f0_coeff)
		return -EINVAL;
	if (!np)
		dev_warn(h->dev, "no device tree node, using defaults\n");
	return 0;
}

static int aw8697_probe(struct i2c_client *client)
{
	struct aw8697 *h;
	int ret;

	h = devm_kzalloc(&client->dev, sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	h->dev = &client->dev;
	i2c_set_clientdata(client, h);
	mutex_init(&h->lock);
	INIT_WORK(&h->play_work, aw8697_play_work);

	h->regmap = devm_regmap_init_i2c(client, &aw8697_regmap_config);
	if (IS_ERR(h->regmap))
		return dev_err_probe(h->dev, PTR_ERR(h->regmap),
				     "failed to allocate regmap\n");

	h->reset_gpio = devm_gpiod_get_optional(h->dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(h->reset_gpio))
		return dev_err_probe(h->dev, PTR_ERR(h->reset_gpio),
				     "failed to get reset gpio\n");

	ret = aw8697_parse_dt(h);
	if (ret)
		return ret;

	ret = aw8697_hw_reset(h);
	if (ret)
		return ret;
	ret = aw8697_soft_reset(h);
	if (ret)
		return dev_err_probe(h->dev, ret, "soft reset failed\n");
	ret = aw8697_detect(h);
	if (ret)
		return ret;
	ret = aw8697_init_chip(h);
	if (ret)
		return dev_err_probe(h->dev, ret, "chip init failed\n");

	h->input = devm_input_allocate_device(h->dev);
	if (!h->input)
		return -ENOMEM;

	h->input->name = "aw8697-haptics";
	h->input->phys = "aw8697/input0";
	h->input->id.bustype = BUS_I2C;
	input_set_drvdata(h->input, h);
	input_set_capability(h->input, EV_FF, FF_RUMBLE);
	input_set_capability(h->input, EV_FF, FF_PERIODIC);

	ret = input_ff_create_memless(h->input, NULL, aw8697_play_effect);
	if (ret)
		return dev_err_probe(h->dev, ret, "failed to create FF device\n");

	ret = input_register_device(h->input);
	if (ret)
		return dev_err_probe(h->dev, ret, "failed to register input\n");

	dev_info(h->dev, "AW8697 haptic driver ready (f0 %u Hz)\n", h->f0_pre);
	return 0;
}

static void aw8697_remove(struct i2c_client *client)
{
	struct aw8697 *h = i2c_get_clientdata(client);

	cancel_work_sync(&h->play_work);
	mutex_lock(&h->lock);
	aw8697_stop(h);
	mutex_unlock(&h->lock);
}

static const struct of_device_id aw8697_of_match[] = {
	{ .compatible = "awinic,aw8697" },
	{ .compatible = "awinic,aw8697_haptic" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw8697_of_match);

static const struct i2c_device_id aw8697_i2c_id[] = {
	{ "aw8697" },
	{ "aw8697_haptic" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw8697_i2c_id);

static struct i2c_driver aw8697_driver = {
	.driver = {
		.name = AW8697_I2C_NAME,
		.of_match_table = aw8697_of_match,
	},
	.probe = aw8697_probe,
	.remove = aw8697_remove,
	.id_table = aw8697_i2c_id,
};
module_i2c_driver(aw8697_driver);

MODULE_DESCRIPTION("Awinic AW8697 LRA haptic driver");
MODULE_LICENSE("GPL");
