// SPDX-License-Identifier: GPL-2.0-only
/*
 * "K16A" 1080x2400 C-PHY video mode panels as used on the Xiaomi evergo
 * (Redmi Note 11 5G): the Tianma k16a_36_02_0a_vdo and the CSOT
 * k16a_42_02_0b_vdo, both built around a Novatek NT36672C controller.
 *
 * Ported from the vendor kernel's panel-k16a-36-02-0a-vdo.c and
 * panel-k16a-42-02-0b-vdo.c.  Three things about this panel are unusual:
 *
 *  - it is driven over a 3-trio C-PHY link with no clock lane, so the DSI
 *    node carries "mediatek,cphy" and the PHY is the mt6833-mipi-tx-cphy
 *    driver.  The LP/HS cycle counts the C-PHY blanking is computed from
 *    are the panel module's own (11/35/26) and come from the DSI node;
 *  - its bias rails and backlight are supplied by an LM36273 which is
 *    driven through the "backlight" phandle - the bias has to be up before
 *    the VSP/VSN GPIOs are released and the DCS init starts;
 *  - the two modules differ in their DCS init sequence and in the vertical
 *    sync split (VSA 4/VBP 16 vs VSA 8/VBP 12), so both are described here.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

#define K16A_DEFAULT_BRIGHTNESS		128
#define K16A_LCD_DVDD_UV		1300000

struct k16a;

struct k16a_desc {
	const char *name;
	const struct drm_display_mode *mode;
	void (*init)(struct k16a *ctx);
};

struct k16a {
	struct device *dev;
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct backlight_device *backlight;
	struct regulator *dvdd;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pm_gpio;
	struct gpio_desc *bl_en_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	const struct k16a_desc *desc;
	bool prepared;
};

static inline struct k16a *to_k16a(struct drm_panel *panel)
{
	return container_of(panel, struct k16a, panel);
}

/*
 * Manufacturer commands (0xb0 and above) go out as generic writes and
 * everything else as DCS - the split the vendor driver uses.  This relies on
 * the ARM unsigned-char default: 0xff is *not* below 0xb0.
 */
static void k16a_dcs_write(struct k16a *ctx, const void *data, size_t len)
{
	const u8 *d = data;
	ssize_t ret;

	if (d[0] < 0xb0)
		ret = mipi_dsi_dcs_write_buffer(ctx->dsi, data, len);
	else
		ret = mipi_dsi_generic_write(ctx->dsi, data, len);

	if (ret < 0)
		dev_err(ctx->dev, "DCS write %#x failed: %zd\n", d[0], ret);
}

#define K16A_DCS(ctx, seq...)				\
	do {						\
		static const u8 d[] = { seq };		\
		k16a_dcs_write(ctx, d, sizeof(d));	\
	} while (0)

/* 1080x2400@60: HFP 228 / HSA 20 / HBP 36, VFP 1290 / VSA 4 / VBP 16 */
static const struct drm_display_mode k16a_36_mode = {
	.clock = 303626,
	.hdisplay = 1080,
	.hsync_start = 1308,
	.hsync_end = 1328,
	.htotal = 1364,
	.vdisplay = 2400,
	.vsync_start = 3690,
	.vsync_end = 3694,
	.vtotal = 3710,
};

/* the CSOT module keeps the same glass timings but splits VSA/VBP as 8/12 */
static const struct drm_display_mode k16a_42_mode = {
	.clock = 303626,
	.hdisplay = 1080,
	.hsync_start = 1308,
	.hsync_end = 1328,
	.htotal = 1364,
	.vdisplay = 2400,
	.vsync_start = 3690,
	.vsync_end = 3698,
	.vtotal = 3710,
};

static void k16a_36_init(struct k16a *ctx)
{
	K16A_DCS(ctx, 0xff, 0x10);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x3b, 0x03, 0x14, 0x36, 0x04, 0x04);
	K16A_DCS(ctx, 0xb0, 0x00);
	K16A_DCS(ctx, 0xc0, 0x00);
	K16A_DCS(ctx, 0xc1, 0x89, 0x28, 0x00, 0x14, 0x00, 0xaa, 0x02,
		 0x0e, 0x00, 0x71, 0x00, 0x07, 0x05, 0x0e, 0x05, 0x16);
	K16A_DCS(ctx, 0xc2, 0x1b, 0xa0);

	K16A_DCS(ctx, 0xff, 0xe0);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x35, 0x82);

	K16A_DCS(ctx, 0xff, 0xf0);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x1c, 0x01);
	K16A_DCS(ctx, 0x33, 0x01);
	K16A_DCS(ctx, 0x5a, 0x00);

	K16A_DCS(ctx, 0xff, 0xd0);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x53, 0x22);
	K16A_DCS(ctx, 0x54, 0x02);

	K16A_DCS(ctx, 0xff, 0xc0);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x9c, 0x11);
	K16A_DCS(ctx, 0x9d, 0x11);

	K16A_DCS(ctx, 0xff, 0x10);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0xc0, 0x00);
	K16A_DCS(ctx, 0x35, 0x00);
}

static void k16a_42_init(struct k16a *ctx)
{
	K16A_DCS(ctx, 0xff, 0x10);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0xc0, 0x00);

	K16A_DCS(ctx, 0xff, 0xe0);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x35, 0x82);

	K16A_DCS(ctx, 0xff, 0xf0);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x1c, 0x01);
	K16A_DCS(ctx, 0x33, 0x01);
	K16A_DCS(ctx, 0x5a, 0x00);

	K16A_DCS(ctx, 0xff, 0xd0);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x53, 0x22);
	K16A_DCS(ctx, 0x54, 0x02);

	K16A_DCS(ctx, 0xff, 0xc0);
	K16A_DCS(ctx, 0xfb, 0x01);
	K16A_DCS(ctx, 0x9c, 0x11);
	K16A_DCS(ctx, 0x9d, 0x11);

	K16A_DCS(ctx, 0xff, 0x10);
	K16A_DCS(ctx, 0x35, 0x00);
}

static const struct k16a_desc k16a_36_desc = {
	.name = "k16a_36_02_0a_vdo",
	.mode = &k16a_36_mode,
	.init = k16a_36_init,
};

static const struct k16a_desc k16a_42_desc = {
	.name = "k16a_42_02_0b_vdo",
	.mode = &k16a_42_mode,
	.init = k16a_42_init,
};

static void k16a_reset(struct k16a *ctx)
{
	/* the vendor sequence: high, then two low/high pulses, 10 ms apart */
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 11 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 11 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 11 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 11 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 15 * 1000);
}

static int k16a_prepare(struct drm_panel *panel)
{
	struct k16a *ctx = to_k16a(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	if (ctx->dvdd) {
		ret = regulator_set_voltage(ctx->dvdd, K16A_LCD_DVDD_UV,
					    K16A_LCD_DVDD_UV);
		if (ret)
			dev_err(ctx->dev, "failed to set lcd_dvdd: %d\n", ret);

		ret = regulator_enable(ctx->dvdd);
		if (ret)
			dev_err(ctx->dev, "failed to enable lcd_dvdd: %d\n",
				ret);
		usleep_range(2000, 2001);
	}

	gpiod_set_value(ctx->pm_gpio, 1);
	gpiod_set_value(ctx->bl_en_gpio, 1);
	usleep_range(2000, 2001);

	/*
	 * Bias on: the LM36273 configures itself and ramps VPOS/VNEG while
	 * the backlight stays dark (brightness is still 0), and only then are
	 * the panel module's VSP/VSN gate GPIOs released.
	 */
	if (ctx->backlight)
		backlight_enable(ctx->backlight);
	usleep_range(2000, 2001);

	gpiod_set_value(ctx->bias_pos, 1);
	usleep_range(2000, 2001);
	gpiod_set_value(ctx->bias_neg, 1);
	usleep_range(2000, 2001);

	k16a_reset(ctx);
	ctx->desc->init(ctx);
	K16A_DCS(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	usleep_range(70000, 70001);
	K16A_DCS(ctx, MIPI_DCS_SET_DISPLAY_ON);

	ctx->prepared = true;

	return 0;
}

static int k16a_unprepare(struct drm_panel *panel)
{
	struct k16a *ctx = to_k16a(panel);

	if (!ctx->prepared)
		return 0;

	K16A_DCS(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(20000, 20001);
	K16A_DCS(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(100000, 100001);

	/* and reverse of prepare: VSP first, then the chip, then VSN */
	gpiod_set_value(ctx->bias_pos, 0);
	usleep_range(2000, 2001);
	if (ctx->backlight)
		backlight_disable(ctx->backlight);
	usleep_range(2000, 2001);
	gpiod_set_value(ctx->bias_neg, 0);
	usleep_range(2000, 2001);

	gpiod_set_value(ctx->bl_en_gpio, 0);
	gpiod_set_value(ctx->pm_gpio, 0);
	if (ctx->dvdd)
		regulator_disable(ctx->dvdd);

	ctx->prepared = false;

	return 0;
}

static int k16a_enable(struct drm_panel *panel)
{
	struct k16a *ctx = to_k16a(panel);

	if (!ctx->backlight)
		return 0;

	/* bring-up default, so the panel lights up without userspace help */
	if (!ctx->backlight->props.brightness)
		ctx->backlight->props.brightness = K16A_DEFAULT_BRIGHTNESS;

	return backlight_enable(ctx->backlight);
}

static int k16a_disable(struct drm_panel *panel)
{
	struct k16a *ctx = to_k16a(panel);

	if (!ctx->backlight)
		return 0;

	return backlight_disable(ctx->backlight);
}

static int k16a_get_modes(struct drm_panel *panel,
			  struct drm_connector *connector)
{
	struct k16a *ctx = to_k16a(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, ctx->desc->mode);
	if (!mode) {
		dev_err(ctx->dev, "failed to add mode %ux%u@%u\n",
			ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
			drm_mode_vrefresh(ctx->desc->mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = 69;
	connector->display_info.height_mm = 153;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs k16a_panel_funcs = {
	.prepare = k16a_prepare,
	.unprepare = k16a_unprepare,
	.enable = k16a_enable,
	.disable = k16a_disable,
	.get_modes = k16a_get_modes,
};

static int k16a_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct k16a *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct k16a, panel, &k16a_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dev = dev;
	ctx->dsi = dsi;
	ctx->desc = of_device_get_match_data(dev);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset gpio\n");

	ctx->pm_gpio = devm_gpiod_get_optional(dev, "pm-enable",
					       GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->pm_gpio),
				     "failed to get pm-enable gpio\n");

	ctx->bl_en_gpio = devm_gpiod_get(dev, "lcm-bl-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bl_en_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->bl_en_gpio),
				     "failed to get lcm-bl-enable gpio\n");

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bias_pos))
		return dev_err_probe(dev, PTR_ERR(ctx->bias_pos),
				     "failed to get bias gpio 0\n");

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bias_neg))
		return dev_err_probe(dev, PTR_ERR(ctx->bias_neg),
				     "failed to get bias gpio 1\n");

	/*
	 * The 1.3 V DVDD comes from the MT6359 (VCN13) on the real board; it
	 * is optional here so the panel can come up while the PMIC side is
	 * still being brought up.
	 */
	ctx->dvdd = devm_regulator_get_optional(dev, "lcd_dvdd");
	if (IS_ERR(ctx->dvdd)) {
		if (PTR_ERR(ctx->dvdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ctx->dvdd),
					     "failed to get lcd_dvdd\n");
		ctx->dvdd = NULL;
	}

	ctx->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(ctx->backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->backlight),
				     "failed to get backlight\n");

	dsi->lanes = 3;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			  MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM |
			  MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	mipi_dsi_set_drvdata(dsi, ctx);

	ret = devm_drm_panel_add(dev, &ctx->panel);
	if (ret)
		return ret;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0)
		return ret;

	dev_info(dev, "%s panel ready\n", ctx->desc->name);

	return 0;
}

static const struct of_device_id k16a_of_match[] = {
	{ .compatible = "k16a_36_02_0a_vdo,lcm", .data = &k16a_36_desc },
	{ .compatible = "k16a_42_02_0b_vdo,lcm", .data = &k16a_42_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, k16a_of_match);

static struct mipi_dsi_driver k16a_driver = {
	.probe = k16a_probe,
	.driver = {
		.name = "panel-k16a-vdo",
		.of_match_table = k16a_of_match,
	},
};
module_mipi_dsi_driver(k16a_driver);

MODULE_DESCRIPTION("K16A (NT36672C) 1080x2400 C-PHY panels");
MODULE_LICENSE("GPL");
