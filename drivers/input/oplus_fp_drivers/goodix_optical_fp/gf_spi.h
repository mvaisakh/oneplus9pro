/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>
#include "../include/oplus_fp_common.h"

/**********************************************************/
enum FP_MODE{
	GF_IMAGE_MODE = 0,
	GF_KEY_MODE,
	GF_SLEEP_MODE,
	GF_FF_MODE,
	GF_DEBUG_MODE = 0x56
};

struct gf_ioc_chip_info {
	unsigned char vendor_id;
	unsigned char mode;
	unsigned char operation;
	unsigned char reserved[5];
};

#define GF_IOC_MAGIC    'g'     //define magic number
#define GF_IOC_INIT             _IOR(GF_IOC_MAGIC, 0, uint8_t)
#define GF_IOC_EXIT             _IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET            _IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ       _IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ      _IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK   _IOW(GF_IOC_MAGIC, 5, uint32_t)
#define GF_IOC_DISABLE_SPI_CLK  _IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER     _IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER    _IO(GF_IOC_MAGIC, 8)
#define GF_IOC_INPUT_KEY_EVENT  _IOW(GF_IOC_MAGIC, 9, struct gf_key)
#define GF_IOC_ENTER_SLEEP_MODE _IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO      _IOR(GF_IOC_MAGIC, 11, uint8_t)
#define GF_IOC_REMOVE           _IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO        _IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)
#define GF_IOC_POWER_RESET      _IO(GF_IOC_MAGIC, 17)
#define GF_IOC_WAKELOCK_TIMEOUT_ENABLE        _IO(GF_IOC_MAGIC, 18 )
#define GF_IOC_WAKELOCK_TIMEOUT_DISABLE        _IO(GF_IOC_MAGIC, 19 )
#define GF_IOC_CLEAN_TOUCH_FLAG        _IO(GF_IOC_MAGIC, 20 )
#define GF_IOC_AUTO_SEND_TOUCHDOWN        _IO(GF_IOC_MAGIC, 21)
#define GF_IOC_AUTO_SEND_TOUCHUP        _IO(GF_IOC_MAGIC, 22)

#define  GF_IOC_MAXNR    14  /* THIS MACRO IS NOT USED NOW... */

#define  USE_PLATFORM_BUS     1
#define GF_NETLINK_ENABLE 1
#define GF_NET_EVENT_FB_BLACK 2
#define GF_NET_EVENT_FB_UNBLACK 3
#define NETLINK_TEST 25

enum NETLINK_CMD {
    GF_NET_EVENT_TEST = 0,
    GF_NET_EVENT_IRQ = 1,
    GF_NET_EVENT_SCR_OFF,
    GF_NET_EVENT_SCR_ON,
    GF_NET_EVENT_TP_TOUCHDOWN,
    GF_NET_EVENT_TP_TOUCHUP,
    GF_NET_EVENT_UI_READY,
    GF_NET_EVENT_MAX,
};

struct gf_dev {
	dev_t devt;
	struct list_head device_entry;
	struct platform_device *spi;
	struct clk *core_clk;
	struct clk *iface_clk;

	struct input_dev *input;
	/* buffer is NULL unless this device is open (users > 0) */
	unsigned users;
	signed irq_gpio;
	signed reset_gpio;
	signed pwr_gpio;
	int irq;
	int irq_enabled;
	int clk_enabled;
	struct notifier_block notifier;
	char device_available;
	char fb_black;

    /* jinrong add for power */
    unsigned power_num;
    fp_power_info_t pwr_list[FP_MAX_PWR_LIST_LEN];
    uint32_t notify_tpinfo_flag;
    uint32_t ftm_poweroff_flag;
};

#endif /*__GF_SPI_H*/
