// SPDX-License-Identifier: GPL-2.0-only
/*
 * ams OSRAM TSL2522 ambient light sensor (ALS only).
 *
 * The evergo wires the TSL2522 to i2c1 @0x39 next to the AK09918
 * magnetometer.  The vendor kernel does not drive it from the AP: the
 * MediaTek SCP sensor-hub firmware owns it (see the Android kernel's
 * sensors-1.0/alsps/alspshub driver, which only forwards IPI requests).
 *
 * The register sequence below was recovered from the vendor scp.img
 * (evergo_images V14.0.4.0, "init_tsl2522"): the photodiode/modulator
 * multiplexer programming (0xDC..0xDF), FIFO data config (0xF9/0xFA),
 * modulator calibration (0xCF/0xE4/0xE6), thresholds (0x8A..0x8F) and the
 * sample/measurement timing (0x83..0x86/0x89/0xAC/0xAD) are all explicit
 * there, and the defaults alone leave the ALS data registers at zero.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>

#define TSL2522_REG_ENABLE		0x80
#define TSL2522_REG_MEAS_MODE0		0x81
#define TSL2522_REG_ID			0x92
#define TSL2522_REG_STATUS		0x93
#define TSL2522_REG_ALS_STATUS		0x94
#define TSL2522_REG_ALS_DATA0_LSB	0x95
#define TSL2522_REG_ALS_DATA0_MSB	0x96
#define TSL2522_REG_ALS_DATA1_LSB	0x97
#define TSL2522_REG_ALS_DATA1_MSB	0x98
#define TSL2522_REG_STATUS2		0x9d
#define TSL2522_REG_CONTROL		0xb1

#define TSL2522_ENABLE_PON		BIT(0)
#define TSL2522_ENABLE_AEN		BIT(1)
#define TSL2522_CONTROL_SOFT_RESET	BIT(3)
#define TSL2522_STATUS2_ALS_DATA_VALID	BIT(6)

#define TSL2522_ID			0x5c

struct tsl2522_data {
	struct regmap *regmap;
};

static const struct regmap_config tsl2522_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static int tsl2522_update_bits(struct tsl2522_data *data, u8 reg, u8 mask,
			       u8 val)
{
	unsigned int old;
	int ret;

	ret = regmap_read(data->regmap, reg, &old);
	if (ret)
		return ret;

	return regmap_write(data->regmap, reg, (old & ~mask) | (val & mask));
}

/*
 * One entry of the vendor's tsl2522_als_cfg() sequence.  mask 0xff means a
 * plain write; anything else is a read-modify-write of those bits.
 */
static const struct {
	u8 reg;
	u8 mask;
	u8 val;
} tsl2522_init_seq[] = {
	{ 0x80, 0x01, 0x01 },	/* PON */
	{ 0xdc, 0xff, 0x56 },	/* step0 modulator/photodiode mapping */
	{ 0xdd, 0xff, 0x09 },
	{ 0xde, 0xff, 0x56 },	/* step1 mapping */
	{ 0xdf, 0xff, 0x09 },
	{ 0xf9, 0xff, 0x0f },	/* modulator FIFO data config 0 */
	{ 0xfa, 0xff, 0x0f },	/* modulator FIFO data config 1 */
	{ 0xd0, 0x0f, 0x01 },	/* ALS sequencer pattern: step 0 */
	{ 0xcf, 0x0f, 0x02 },	/* flicker sequencer pattern: step 1 */
	{ 0x83, 0xff, 0xb3 },	/* sample time */
	{ 0x84, 0xff, 0x00 },
	{ 0x89, 0xff, 0x00 },	/* wait time */
	{ 0xac, 0xff, 0x13 },	/* AGC number of samples */
	{ 0xad, 0xff, 0x00 },
	{ 0xdf, 0xf0, 0xf0 },	/* vendor sets the high nibble too */
	{ 0xe1, 0xf0, 0xf0 },
	{ 0xe4, 0xff, 0x01 },	/* modulator calibration */
	{ 0xe6, 0x20, 0x20 },
	{ 0x8a, 0xff, 0x00 },	/* ALS thresholds */
	{ 0x8b, 0xff, 0x00 },
	{ 0x8c, 0xff, 0x00 },
	{ 0x8d, 0xff, 0xff },
	{ 0x8e, 0xff, 0x00 },
	{ 0x8f, 0xff, 0x00 },
	{ 0xa6, 0x0f, 0x01 },	/* APERS */
	{ 0x85, 0xff, 0x8f },	/* ALS measurement time */
	{ 0x86, 0xff, 0x01 },
};

static int tsl2522_init(struct device *dev, struct tsl2522_data *data)
{
	unsigned int id;
	int ret, i;

	ret = regmap_write(data->regmap, TSL2522_REG_CONTROL,
			   TSL2522_CONTROL_SOFT_RESET);
	if (ret)
		return ret;
	usleep_range(5000, 6000);

	ret = regmap_read(data->regmap, TSL2522_REG_ID, &id);
	if (ret)
		return ret;
	if (id != TSL2522_ID)
		return dev_err_probe(dev, -ENODEV, "unexpected chip id %#x\n", id);

	for (i = 0; i < ARRAY_SIZE(tsl2522_init_seq); i++) {
		ret = tsl2522_update_bits(data, tsl2522_init_seq[i].reg,
					  tsl2522_init_seq[i].mask,
					  tsl2522_init_seq[i].val);
		if (ret)
			return ret;
	}

	/* enable ALS and clear the status flags, like the vendor firmware */
	ret = regmap_write(data->regmap, TSL2522_REG_ENABLE,
			   TSL2522_ENABLE_PON | TSL2522_ENABLE_AEN);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, TSL2522_REG_STATUS, 0xff);

	return ret;
}

static int tsl2522_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct tsl2522_data *data = iio_priv(indio_dev);
	u8 buf[8];
	unsigned int status2;
	int ret, tries;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	/* wait for the measurement engine to latch a valid ALS result */
	for (tries = 0; tries < 20; tries++) {
		ret = regmap_read(data->regmap, TSL2522_REG_STATUS2, &status2);
		if (ret)
			return ret;
		if (status2 & TSL2522_STATUS2_ALS_DATA_VALID)
			break;
		usleep_range(10000, 11000);
	}
	if (tries == 20)
		return -EAGAIN;

	ret = regmap_bulk_read(data->regmap, TSL2522_REG_ALS_STATUS, buf,
			       ARRAY_SIZE(buf));
	if (ret)
		return ret;

	if (chan->channel == 0)
		*val = buf[1] | (buf[2] << 8);
	else
		*val = buf[3] | (buf[4] << 8);

	return IIO_VAL_INT;
}

static const struct iio_info tsl2522_info = {
	.read_raw = tsl2522_read_raw,
};

static const struct iio_chan_spec tsl2522_channels[] = {
	{
		.type = IIO_LIGHT,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_INTENSITY,
		.channel = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int tsl2522_probe(struct i2c_client *client)
{
	struct tsl2522_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->regmap = devm_regmap_init_i2c(client, &tsl2522_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(data->regmap),
				     "failed to init regmap\n");

	ret = tsl2522_init(&client->dev, data);
	if (ret)
		return ret;

	indio_dev->name = "tsl2522";
	indio_dev->info = &tsl2522_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = tsl2522_channels;
	indio_dev->num_channels = ARRAY_SIZE(tsl2522_channels);

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to register IIO device\n");

	dev_info(&client->dev, "TSL2522 ALS ready (vendor scp init sequence)\n");
	return 0;
}

static const struct of_device_id tsl2522_of_match[] = {
	{ .compatible = "amstaos,tsl2522" },
	{ }
};
MODULE_DEVICE_TABLE(of, tsl2522_of_match);

static const struct i2c_device_id tsl2522_id[] = {
	{ "tsl2522", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tsl2522_id);

static struct i2c_driver tsl2522_driver = {
	.driver = {
		.name = "tsl2522",
		.of_match_table = tsl2522_of_match,
	},
	.probe = tsl2522_probe,
	.id_table = tsl2522_id,
};
module_i2c_driver(tsl2522_driver);

MODULE_DESCRIPTION("ams OSRAM TSL2522 ambient light sensor");
MODULE_LICENSE("GPL");
