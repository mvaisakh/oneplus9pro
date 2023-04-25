// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2021 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include "kookong_ir_spi.h"


#define MASK_WRITE    0x00

#define KOOKONG_IR_MAX_SIZE 2048*1024u
#define KOOKONG_IR_MIN_SIZE 8u
#define ONESECOND	1000000
#define ONESECOND_X100000	100000000000
#define LARGE_DATA_MALLOC_THRESHOLD	120000
#define SPIFREQUENCY	5000000
#define SPI_DEFAULT_FREQ	5000000
#define IR_DEFAULT_VDD_MIN_VOL	2950000
#define IR_DEFAULT_VDD_MAX_VOL	3050000

struct hw_config_t {
	int vdd_min_vol;
	int vdd_max_vol;
	int spi_frep;
	int spi_mode;
};

struct kookong_ir_t {
	dev_t devt;
	struct hw_config_t hw_config;
	struct miscdevice misc_dev;
	struct spi_device *spi;
	unsigned char *tx_buf;
	struct mutex kookong_tx_mutex;
	struct list_head device_entry;
};

static struct kookong_ir_t *kookong_ir;

struct spi_transfer spi_tx_xfer;

struct regulator *vdd_3v0 = NULL;

static int parse_hw_config(struct device *dev)
{
	int retval;
	u32 value;
	struct device_node *np = dev->of_node;

	if (!kookong_ir) {
		pr_err("kookong_ir: parse_hw_config error, kookong_ir is null!\n");
	} else {
		retval = of_property_read_u32(np, "spi-max-frequency", &value);
		if (retval < 0)
			kookong_ir->hw_config.spi_frep = SPI_DEFAULT_FREQ;
		else
			kookong_ir->hw_config.spi_frep = value;

		retval = of_property_read_u32(np, "ir-spi-mode", &value);
		if (retval < 0)
			kookong_ir->hw_config.spi_mode = SPI_MODE_0;
		else
			kookong_ir->hw_config.spi_mode = value;

		retval = of_property_read_u32(np, "vdd-min-vol", &value);
		if (retval < 0)
			kookong_ir->hw_config.vdd_min_vol = IR_DEFAULT_VDD_MIN_VOL;
		else
			kookong_ir->hw_config.vdd_min_vol = value;

		retval = of_property_read_u32(np, "vdd-max-vol", &value);
		if (retval < 0)
			kookong_ir->hw_config.vdd_max_vol = IR_DEFAULT_VDD_MAX_VOL;
		else
			kookong_ir->hw_config.vdd_max_vol = value;
		pr_info("kookong_ir: kookong_ir->hw_config.spi_frep=%d\n", kookong_ir->hw_config.spi_frep);
		pr_info("kookong_ir: kookong_ir->hw_config.spi_mode=%d\n", kookong_ir->hw_config.spi_mode);
		pr_info("kookong_ir: kookong_ir->hw_config.vdd_min_vol=%d\n", kookong_ir->hw_config.vdd_min_vol);
		pr_info("kookong_ir: kookong_ir->hw_config.vdd_max_vol=%d\n", kookong_ir->hw_config.vdd_max_vol);
	}

	return 0;
}

static void appendPulse(u8* spis, u32 spisnum, u32 *bytePos, u32 *bitPos, u32 bitNum, u8 isHigh)
{
	u8	b;
	u32	leftBits;
	u32	maskBits;
	u32	i;
	while (bitNum > 0) {
		if ((*bytePos) >= spisnum) {
		  return;
		}

		b = 0;
		if ((*bitPos) > 0) {
			b = spis[*bytePos];
		}

		leftBits	= 8 - (*bitPos);
		maskBits	= (leftBits < bitNum) ? leftBits : bitNum;
		leftBits	= 8 - maskBits - (*bitPos);
		if (isHigh == true) {
			for (i = 0; i < maskBits; i++) {
				b |= (1 << (leftBits + i));
			}
		}

		spis[*bytePos] = b;
		(*bitPos) += maskBits;
		(*bitPos) %= 8;
		if ((*bitPos) == 0) {
		  (*bytePos)++;
		}

		bitNum -= maskBits;
	}
}


static void appendPulse_HL(u8 *spis, u32 spisNum, u32 *bytePos, u32 *bitPos, u32 spiHighNum, u32 spiLowNum, u8 isHigh)
{
	if (isHigh == true) {
		appendPulse(spis, spisNum, bytePos, bitPos, spiHighNum, true);
		appendPulse(spis, spisNum, bytePos, bitPos, spiLowNum, false);
	} else {
		appendPulse(spis, spisNum, bytePos, bitPos, spiHighNum + spiLowNum, false);
	}
}


static int irToSpi(u8* spis, u32 spisNum, u32 spiFrequency, u32 irFrequency, const int *pattern, u32 pattern_length)
{
	u32	bitPos;
	u32	bytePos;
	u32	leftTime;
	u32	i;
	u32	j;
	u32	p;
	u32	irWaveNum;
	u32	addSpiLowNum;
	u8	divide;
	u32	highSpiNumPerIr;
	u32	lowSpiNumPerIr;

	u32	spiWaveLen_x10		= ONESECOND*10 / spiFrequency;
	u32	irWaveLen_x10		= ONESECOND*10 / irFrequency;
	u64	spiNumPerIrDouble	= ((u64)irWaveLen_x10) * 100000 / ((u64) spiWaveLen_x10);

	u32 spiNumPerIrInt = (u32) (spiNumPerIrDouble + 50000) / 100000;

	irWaveLen_x10 = spiNumPerIrInt * spiWaveLen_x10;

	highSpiNumPerIr = spiNumPerIrInt / 2;
	lowSpiNumPerIr	= spiNumPerIrInt - highSpiNumPerIr;


	bitPos		= 0;
	bytePos		= 0;
	leftTime	= 0;
	for (i = 0; i < pattern_length; i++) {
		if (i % 2 == 0) {
			p = pattern[i] * 10;
			divide = true;
		} else {
			p = pattern[i] * 10 + leftTime;
			leftTime = 0;
			divide = false;
		}

		irWaveNum = (u32) (p / irWaveLen_x10);
		for (j = 0; j < irWaveNum; j++) {
			appendPulse_HL(spis, spisNum, &bytePos, &bitPos, highSpiNumPerIr, lowSpiNumPerIr, divide);
		}

		leftTime += p - irWaveNum * irWaveLen_x10;
		if (i % 2 != 0) {
			addSpiLowNum = (u32) (leftTime / spiWaveLen_x10);
			if (addSpiLowNum > 0) {
				appendPulse(spis, spisNum, &bytePos, &bitPos, addSpiLowNum, false);
				leftTime -= addSpiLowNum * spiWaveLen_x10;
			}
		}
	}

	return(1);
}

static int kookong_ir_spi_write(struct kookong_ir_t *spidev, unsigned int length)
{
	int retval;
	struct spi_message msg;

	mutex_lock(&spidev->kookong_tx_mutex);
	spi_message_init(&msg);
	spi_tx_xfer.len = length;
	spi_tx_xfer.tx_buf = spidev->tx_buf;
	spi_tx_xfer.delay_usecs = 0;
	spi_tx_xfer.speed_hz = spidev->hw_config.spi_frep;
	spi_message_add_tail(&spi_tx_xfer, &msg);
	spidev->spi->mode = spidev->hw_config.spi_mode;
	retval = spi_sync(spidev->spi, &msg);
	if (retval == 0)
		retval = length;
	else {
		pr_err("kookong_ir: Failed to complete SPI transfer, error = %d\n", retval);
	}
	mutex_unlock(&spidev->kookong_tx_mutex);
	return retval;
}

static ssize_t kookong_ir_file_write(struct file *file, const char __user *ubuff, size_t count, loff_t *offset)
{
	char *user_wBuff;
	unsigned int byte_count;
	int retval, i;
	u32	spisNum;
	int *pattern;
	u32	spiTotalNum;
	u64	spiWaveLen;
	u32	spiFrequency = SPIFREQUENCY;
	int carrier_freq = 0;
	u32	totalTime = 0;

	mutex_lock(&kookong_ir->kookong_tx_mutex);
	if (vdd_3v0 != NULL) {
		regulator_set_voltage(vdd_3v0, kookong_ir->hw_config.vdd_min_vol, kookong_ir->hw_config.vdd_max_vol);
		retval = regulator_enable(vdd_3v0);
		if (retval) {
			pr_err("kookong_ir: file_write vdd_3v0 enable fail\n");
		}
	}
	mutex_unlock(&kookong_ir->kookong_tx_mutex);

	mdelay(20);

	if (*offset < 0)
		return -EINVAL;
	if (count == 0 || count > KOOKONG_IR_MAX_SIZE || count < KOOKONG_IR_MIN_SIZE)
		return 0;
	if (*offset != 0) {
		return 0;
	}
	user_wBuff = kzalloc(count, GFP_KERNEL);
	if (!user_wBuff) {
		return -ENOMEM;
	}
	if (copy_from_user(user_wBuff, ubuff, count)) {
		kfree(user_wBuff);
		return -EFAULT;
	}
	carrier_freq = (u32) user_wBuff[0] | ((u32) user_wBuff[1] << 8) | ((u32) user_wBuff[2] << 16) | ((u32) user_wBuff[3] << 24);

	for (i = 1; i < (count / 4); i ++) {
		totalTime = totalTime + (u32) user_wBuff[4 * i] + ((u32) user_wBuff[4 * i + 1] << 8) \
		+ ((u32) user_wBuff[4 * i + 2] << 16) + ((u32) user_wBuff[4 * i + 3] << 24);
	}

	pr_info("kookong_ir: transmit for %d uS at %d Hz", totalTime, carrier_freq);

	spiWaveLen = ONESECOND_X100000 / (u64) spiFrequency;
	pr_info("kookong_ir: spiWaveLen=%d\n", spiWaveLen);

	spiTotalNum = (u32) (((u64)totalTime * 100000) / spiWaveLen);
	pr_info("kookong_ir: spiTotalNum=%d\n", spiTotalNum);

	spisNum = (spiTotalNum / 8) + 1;
	pr_info("kookong_ir: spisNum=%d\n", spisNum);

	if (spisNum > KOOKONG_IR_MAX_SIZE) {
		pr_err("kookong_ir: spisNum is too large!\n");
		return(-1);
	}

	kookong_ir->tx_buf = kzalloc(spisNum + 2, GFP_KERNEL);
	pattern = (u32*) (&user_wBuff[4]);

	/*for ir debug*/
	/*for (i = 1; i < (count / 4); i ++) {
		pr_info("kookong_ir_file_write pattern[%d] = %d\n", i, pattern[i - 1]);
	}*/

	if (irToSpi(&kookong_ir->tx_buf[1], spisNum, spiFrequency, carrier_freq, pattern, (count / 4 - 1)) < 0) {
		kfree(kookong_ir->tx_buf);
		return(-1);
	}

	/*for ir debug*/
	/*if (spisNum != 0) {
		pr_info("kookong_ir_file_write data count is = %d\n", spisNum);
		for(i = 0; i < spisNum; i ++) {
			pr_info("kookong_ir_file_write data[%d] = %d\n", i, kookong_ir->tx_buf[i + 1]);
		}
	}*/

	kookong_ir->tx_buf[0] = MASK_WRITE;
	byte_count = spisNum + 1;
	retval = kookong_ir_spi_write(kookong_ir, byte_count);
	kfree(user_wBuff);
	kfree(kookong_ir->tx_buf);
	mdelay(5);
	mutex_lock(&kookong_ir->kookong_tx_mutex);
	regulator_disable(vdd_3v0);
	mutex_unlock(&kookong_ir->kookong_tx_mutex);
	return retval;
}

static int kookong_ir_file_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int kookong_ir_file_close(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t kookong_ir_file_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	return 0;
}

loff_t kookong_ir_file_llseek(struct file *file, loff_t offset, int whence)
{
	return 0;
}

static struct file_operations fops = {
	.open = kookong_ir_file_open,
	.release = kookong_ir_file_close,
	.read = kookong_ir_file_read,
	.write = kookong_ir_file_write,
	.llseek = kookong_ir_file_llseek,
};

static int kookong_ir_spi_probe(struct spi_device *spi)
{
	struct kookong_ir_t *kookong_ir_data;
	vdd_3v0 = regulator_get(&spi->dev, "vdd");

	pr_info("kookong_ir: spi_probe call\n");

	kookong_ir_data = kzalloc(sizeof(*kookong_ir_data), GFP_KERNEL);
	if (!kookong_ir_data)
		return -ENOMEM;
	kookong_ir_data->spi = spi;
	mutex_init(&kookong_ir_data->kookong_tx_mutex);
	INIT_LIST_HEAD(&kookong_ir_data->device_entry);
	kookong_ir_data->misc_dev.fops = &fops;
	kookong_ir_data->misc_dev.name = OPLUS_CONSUMER_IR_DEVICE_NAME;
	kookong_ir_data->misc_dev.minor = MISC_DYNAMIC_MINOR;
	misc_register(&kookong_ir_data->misc_dev);
	kookong_ir = kookong_ir_data;
	parse_hw_config(&spi->dev);
	if (vdd_3v0 != NULL) {
		regulator_set_voltage(vdd_3v0, kookong_ir->hw_config.vdd_min_vol, kookong_ir->hw_config.vdd_max_vol);
	}
	else
		pr_err("%s: vdd_3v0 is NULL\n", __func__);
	spi_set_drvdata(spi, kookong_ir_data);
	return 0;
}

static int kookong_ir_spi_remove(struct spi_device *spi)
{
	struct kookong_ir_t *kookong_ir_data = spi_get_drvdata(spi);
	misc_deregister(&kookong_ir_data->misc_dev);
	kfree(kookong_ir_data);
	return 0;
}

static const struct spi_device_id kookong_ir_id_table[] = {
	{SPI_MODULE_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(spi, kookong_ir_id_table);

static struct of_device_id kookong_ir_of_match_table[] = {
	{
		.compatible = "oplus,kookong_ir_spi",/*"kookong,ir-spi",*/
	},
	{},
};
MODULE_DEVICE_TABLE(of, kookong_ir_of_match_table);


static struct spi_driver kookong_ir_spi_driver = {
	.driver = {
		.name = SPI_MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = kookong_ir_of_match_table,
	},
	.probe = kookong_ir_spi_probe,
	.remove = kookong_ir_spi_remove,
	.id_table = kookong_ir_id_table,
};

module_spi_driver(kookong_ir_spi_driver);

MODULE_AUTHOR("oplus, Inc.");
MODULE_DESCRIPTION("oplus kookong SPI Bus Module");
MODULE_LICENSE("GPL");
