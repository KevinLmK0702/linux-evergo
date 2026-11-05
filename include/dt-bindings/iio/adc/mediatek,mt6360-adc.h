/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * MT6360 PMIC ADC channel indices.
 *
 * Keep the order in sync with mt6360_adc_channels[] in
 * drivers/iio/adc/mt6360-adc.c: the DT cell of an io-channels specifier is
 * the channel number, so board files should use these names instead of
 * hard-coded numbers.
 */
#ifndef _DT_BINDINGS_IIO_ADC_MEDIATEK_MT6360_ADC_H
#define _DT_BINDINGS_IIO_ADC_MEDIATEK_MT6360_ADC_H

#define MT6360_ADC_USBID		0
#define MT6360_ADC_VBUSDIV5		1
#define MT6360_ADC_VBUSDIV2		2
#define MT6360_ADC_VSYS			3
#define MT6360_ADC_VBAT			4
#define MT6360_ADC_IBUS			5
#define MT6360_ADC_IBAT			6
#define MT6360_ADC_CHG_VDDP		7
#define MT6360_ADC_TEMP_JC		8

#endif /* _DT_BINDINGS_IIO_ADC_MEDIATEK_MT6360_ADC_H */
