// SPDX-License-Identifier: GPL-2.0-only
/*
 * ams OSRAM TSL2522 ambient light sensor (ALS only).
 *
 * The evergo wires the TSL2522 to i2c1 @0x39 next to the AK09918
 * magnetometer.  This driver brings the chip up in its continuous ALS mode
 * and exposes the two ALS channels as raw IIO illuminance values; the
 * flicker-detection/modulator engine and proximity-style MOD channels are
 * not implemented.
 *
 * Register map from the public "Ambient 25" driver for the same part.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>

#define TSL2522_REG_ENABLE		0x80
#define TSL2522_REG_WTIME		0x89
#define TSL2522_REG_MEAS_MODE1		0x82
#define TSL2522_REG_ID			0x92
#define TSL2522_REG_ALS_DATA0_LSB	0x95
#define TSL2522_REG_ALS_DATA0_MSB	0x96
#define TSL2522_REG_ALS_DATA1_LSB	0x97
#define TSL2522_REG_ALS_DATA1_MSB	0x98
#define TSL2522_REG_STATUS2		0x9d
#define TSL2522_REG_CFG2		0xa3
#define TSL2522_REG_CONTROL		0xb1
#define TSL2522_REG_INTENAB		0xba
#define TSL2522_REG_FIFO_THR		0xfc
#define TSL2522_REG_FIFO_STATUS0	0xfd
#define TSL2522_REG_FIFO_STATUS1	0xfe
#define TSL2522_REG_FIFO_DATA		0xff
#define TSL2522_REG_TRIGGER_MODE	0xae
#define TSL2522_REG_MEAS_SEQR_STEP0_MOD_GAINX_L	0xd4

#define TSL2522_ENABLE_PON		BIT(0)
#define TSL2522_ENABLE_AEN		BIT(1)
#define TSL2522_CONTROL_SOFT_RESET	BIT(3)
#define TSL2522_STATUS2_ALS_DATA_VALID	BIT(6)
#define TSL2522_TRIGGER_MODE_NORMAL	0x01
#define TSL2522_WTIME_200MS		0x46
#define TSL2522_MEAS_MODE1_ALS_MSB_8	0x08
#define TSL2522_GAIN_MOD0_32X		0x06
#define TSL2522_GAIN_MOD1_4X		0x30
#define TSL2522_INTENAB_FIEN		BIT(2)
#define TSL2522_FIFO_THR_5		5

struct tsl2522_data {
	struct regmap *regmap;
};

static const struct regmap_config tsl2522_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static int tsl2522_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct tsl2522_data *data = iio_priv(indio_dev);
	unsigned int s0, s1, data_buf[6];
	unsigned int fifo_size;
	int ret, i;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	/* The measurement engine feeds a 6-byte FIFO record: ALS0 then ALS1. */
	ret = regmap_read(data->regmap, TSL2522_REG_FIFO_STATUS0, &s0);
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, TSL2522_REG_FIFO_STATUS1, &s1);
	if (ret)
		return ret;
	fifo_size = (s0 << 2) | (s1 & 0x03);
	if (fifo_size < 6)
		return -EAGAIN;

	for (i = 0; i < 6; i++) {
		ret = regmap_read(data->regmap, TSL2522_REG_FIFO_DATA, &data_buf[i]);
		if (ret)
			return ret;
	}

	if (chan->channel == 0)
		*val = (data_buf[1] << 8) | data_buf[0];
	else
		*val = (data_buf[3] << 8) | data_buf[2];

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
	unsigned int id;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->regmap = devm_regmap_init_i2c(client, &tsl2522_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(data->regmap),
				     "failed to allocate regmap\n");

	/* soft reset, then wait for the oscillator */
	ret = regmap_write(data->regmap, TSL2522_REG_CONTROL,
			   TSL2522_CONTROL_SOFT_RESET);
	if (ret)
		return ret;
	msleep(10);

	ret = regmap_read(data->regmap, TSL2522_REG_ID, &id);
	if (ret)
		return ret;
	if (id == 0x00 || id == 0xff)
		return dev_err_probe(&client->dev, -ENODEV,
				     "TSL2522 not responding (id %#x)\n", id);
	dev_info(&client->dev, "TSL2522 detected (id %#x)\n", id);

	ret = regmap_write(data->regmap, TSL2522_REG_ENABLE, TSL2522_ENABLE_PON);
	if (ret)
		return ret;
	msleep(20);

	ret = regmap_write(data->regmap, TSL2522_REG_WTIME, TSL2522_WTIME_200MS);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, TSL2522_REG_TRIGGER_MODE,
			   TSL2522_TRIGGER_MODE_NORMAL);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, TSL2522_REG_MEAS_MODE1,
			   TSL2522_MEAS_MODE1_ALS_MSB_8);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, TSL2522_REG_MEAS_SEQR_STEP0_MOD_GAINX_L,
			   TSL2522_GAIN_MOD0_32X | TSL2522_GAIN_MOD1_4X);
	if (ret)
		return ret;
	/* FIFO mode: 6-byte ALS records, IRQ on threshold */
	ret = regmap_write(data->regmap, TSL2522_REG_INTENAB,
			   TSL2522_INTENAB_FIEN);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, TSL2522_REG_FIFO_THR,
			   TSL2522_FIFO_THR_5 >> 1);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, TSL2522_REG_CFG2,
			   TSL2522_FIFO_THR_5 & 0x01);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, TSL2522_REG_ENABLE,
			   TSL2522_ENABLE_PON | TSL2522_ENABLE_AEN);
	if (ret)
		return ret;

	indio_dev->name = "tsl2522";
	indio_dev->info = &tsl2522_info;
	indio_dev->channels = tsl2522_channels;
	indio_dev->num_channels = ARRAY_SIZE(tsl2522_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id tsl2522_of_match[] = {
	{ .compatible = "amstaos,tsl2522" },
	{ }
};
MODULE_DEVICE_TABLE(of, tsl2522_of_match);

static const struct i2c_device_id tsl2522_id[] = {
	{ "tsl2522" },
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
