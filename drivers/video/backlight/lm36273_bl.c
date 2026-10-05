// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight + LCD bias driver for the LM36273 ("I2C_LCD_BIAS" in the vendor
 * device tree) used on the Xiaomi evergo with the K16A panels.
 *
 * Ported from the vendor kernel's drivers/gpu/drm/panel/lcm_cust_common.c.
 *
 * The chip owns both the panel bias rails (VPOS/VNEG) and the backlight
 * current, which is why power transitions - not just brightness - go through
 * this driver: the panel's prepare path turns the bias on before it releases
 * the VSP/VSN GPIOs and starts the DSI link, and its unprepare path ramps
 * them down again.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>

/* register map, from the vendor's lcm_cust_common.h */
#define LM36273_DISP_BC1	0x02
#define LM36273_DISP_BC2	0x03
#define LM36273_DISP_BB_LSB	0x04
#define LM36273_DISP_BB_MSB	0x05
#define LM36273_DISP_BL_ENABLE	0x08
#define LM36273_DISP_BIAS_CONF1	0x09
#define LM36273_DISP_BIAS_VPOS	0x0d
#define LM36273_DISP_BIAS_VNEG	0x0e

#define LM36273_MAX_BRIGHTNESS	255

struct lm36273 {
	struct i2c_client *client;
	struct backlight_device *bl;
	struct mutex lock;	/* serialises register access */
	bool bias_on;
	bool bl_on;
};

static int lm36273_write(struct lm36273 *lm, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(lm->client, reg, val);

	if (ret < 0)
		dev_err(&lm->client->dev, "write %#x = %#x failed: %d\n",
			reg, val, ret);

	return ret;
}

/* vendor order: ramp VPOS first, then VNEG */
static void lm36273_bias_enable(struct lm36273 *lm)
{
	lm36273_write(lm, LM36273_DISP_BIAS_CONF1, 0x9c);
	msleep(2);
	lm36273_write(lm, LM36273_DISP_BIAS_CONF1, 0x9e);
	lm->bias_on = true;
}

static void lm36273_bias_disable(struct lm36273 *lm)
{
	lm36273_write(lm, LM36273_DISP_BIAS_CONF1, 0x9c);
	msleep(2);
	lm36273_write(lm, LM36273_DISP_BIAS_CONF1, 0x98);
	lm36273_write(lm, LM36273_DISP_BIAS_CONF1, 0x18);
	lm->bias_on = false;
}

static void lm36273_bl_on(struct lm36273 *lm)
{
	if (lm->bl_on)
		return;

	lm36273_write(lm, LM36273_DISP_BL_ENABLE, 0x17);
	lm36273_write(lm, LM36273_DISP_BC2, 0xcd);
	lm->bl_on = true;
}

static void lm36273_bl_off(struct lm36273 *lm)
{
	if (!lm->bl_on)
		return;

	lm36273_write(lm, LM36273_DISP_BL_ENABLE, 0x00);
	lm->bl_on = false;
}

static int lm36273_update_status(struct backlight_device *bd)
{
	struct lm36273 *lm = bl_get_data(bd);
	int brightness = backlight_get_brightness(bd);
	u32 level;

	mutex_lock(&lm->lock);

	if (backlight_is_blank(bd)) {
		/* the panel is powering down: backlight first, then bias */
		lm36273_bl_off(lm);
		if (lm->bias_on)
			lm36273_bias_disable(lm);
		mutex_unlock(&lm->lock);
		return 0;
	}

	if (!lm->bias_on) {
		/* vendor lm36273_bl_bias_conf() + lm36273_bias_enable(1, 1) */
		lm36273_write(lm, LM36273_DISP_BC1, 0x38);	/* disable PWM */
		lm36273_write(lm, LM36273_DISP_BC2, 0x85);
		lm36273_write(lm, LM36273_DISP_BIAS_VPOS, 0x1e);	/* +5.5 V */
		lm36273_write(lm, LM36273_DISP_BIAS_VNEG, 0x1e);	/* -5.5 V */
		lm36273_bias_enable(lm);
	}

	if (!brightness) {
		/* bias stays on, the backlight simply stays dark */
		lm36273_bl_off(lm);
		mutex_unlock(&lm->lock);
		return 0;
	}

	/* the vendor scales the 0..255 level onto the chip's 0..180 range */
	level = brightness * 180 / LM36273_MAX_BRIGHTNESS;
	lm36273_write(lm, LM36273_DISP_BB_LSB, level & 0x7);
	lm36273_write(lm, LM36273_DISP_BB_MSB, (level >> 3) & 0xff);
	lm36273_bl_on(lm);

	mutex_unlock(&lm->lock);
	return 0;
}

static const struct backlight_ops lm36273_bl_ops = {
	.update_status = lm36273_update_status,
};

static int lm36273_probe(struct i2c_client *client)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = LM36273_MAX_BRIGHTNESS,
	};
	struct lm36273 *lm;

	lm = devm_kzalloc(&client->dev, sizeof(*lm), GFP_KERNEL);
	if (!lm)
		return -ENOMEM;

	lm->client = client;
	mutex_init(&lm->lock);

	lm->bl = devm_backlight_device_register(&client->dev, "lcd-backlight",
						&client->dev, lm,
						&lm36273_bl_ops, &props);
	if (IS_ERR(lm->bl))
		return dev_err_probe(&client->dev, PTR_ERR(lm->bl),
				     "failed to register backlight\n");

	i2c_set_clientdata(client, lm);

	return 0;
}

static const struct of_device_id lm36273_of_match[] = {
	{ .compatible = "mediatek,i2c_lcd_bias" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm36273_of_match);

static const struct i2c_device_id lm36273_id[] = {
	{ "i2c_lcd_bias" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm36273_id);

static struct i2c_driver lm36273_driver = {
	.driver = {
		.name = "lm36273-backlight",
		.of_match_table = lm36273_of_match,
	},
	.probe = lm36273_probe,
	.id_table = lm36273_id,
};
module_i2c_driver(lm36273_driver);

MODULE_DESCRIPTION("LM36273 backlight and LCD bias driver");
MODULE_LICENSE("GPL");
