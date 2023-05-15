// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#define pr_fmt(fmt)    KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/types.h>
#include <net/sock.h>
#include <net/netlink.h>

#include "../include/wakelock.h"
#include "gf_spi.h"
#include "../include/oplus_fp_common.h"
#include <linux/platform_device.h>
#if IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || IS_ENABLED(CONFIG_DRM_MSM)
#include <linux/msm_drm_notify.h>
#endif
#include <soc/oplus/system/boot_mode.h>
#include <linux/version.h>
#include <soc/oplus/system/oplus_project.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/uaccess.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
#define FB_EARLY_EVENT_BLANK    0x10
#endif

#define VER_MAJOR   1
#define VER_MINOR   2
#define PATCH_LEVEL 9

#ifndef MSM_DRM_ONSCREENFINGERPRINT_EVENT
#define MSM_DRM_ONSCREENFINGERPRINT_EVENT 0x10
#endif

#define WAKELOCK_HOLD_TIME 500 /* in ms */
#define SENDCMD_WAKELOCK_HOLD_TIME 1000 /* in ms */

#define GF_SPIDEV_NAME       "goodix,goodix_fp"
/*device name after register in charater*/
#define GF_DEV_NAME          "goodix_fp"

#define CHRD_DRIVER_NAME     "goodix_fp_spi"
#define CLASS_NAME           "goodix_fp"
#define GF_INPUT_NAME "qwerty"  /*"goodix_fp" */
#define N_SPI_MINORS         32	/* ... up to 256 */

struct fp_underscreen_info fp_tpinfo;
static unsigned int lasttouchmode = 0;

static int SPIDEV_MAJOR;

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct wake_lock fp_wakelock;
static struct wake_lock gf_cmd_wakelock;
struct gf_dev gf;

#define MAX_MSGSIZE 32

static int pid = -1;
struct sock *nl_sk = NULL;

extern struct fp_underscreen_info fp_tpinfo;
extern uint32_t notify_tpinfo_flag;
extern struct gf_dev gf;

static inline void sendnlmsg(char *msg)
{
    struct sk_buff *skb_1;
    struct nlmsghdr *nlh;
    struct netlink_msg_info fp_nl_msg;
    int len = NLMSG_SPACE(MAX_MSGSIZE);
    int ret = 0;
    struct gf_dev *gf_dev = &gf;

    if (!msg || !nl_sk || !pid) {
        return ;
    }
    skb_1 = alloc_skb(len, GFP_KERNEL);
    if (!skb_1) {
        pr_err("alloc_skb error\n");
        return;
    }

    nlh = nlmsg_put(skb_1, 0, 0, 0, MAX_MSGSIZE, 0);

    NETLINK_CB(skb_1).portid = 0;
    NETLINK_CB(skb_1).dst_group = 0;

    fp_nl_msg.netlink_cmd = *msg;
    if( (gf_dev->notify_tpinfo_flag != 0) && ((fp_nl_msg.netlink_cmd == GF_NET_EVENT_TP_TOUCHDOWN) || (fp_nl_msg.netlink_cmd == GF_NET_EVENT_TP_TOUCHUP))) {
        fp_nl_msg.tp_info = fp_tpinfo;
        memcpy(NLMSG_DATA(nlh), &fp_nl_msg , sizeof(struct netlink_msg_info));
        pr_err("send msg touch_state = %d\n", fp_tpinfo.touch_state);
        pr_err("send msg area_rate = %d\n", fp_tpinfo.area_rate);
        pr_err("send msg x = %d\n", fp_tpinfo.x);
        pr_err("send msg y = %d\n", fp_tpinfo.y);
    } else if (nlh != NULL) {
        memcpy(NLMSG_DATA(nlh), msg, sizeof(char));
        pr_debug("send message: %d\n", *(char *)NLMSG_DATA(nlh));
    }

    ret = netlink_unicast(nl_sk, skb_1, pid, MSG_DONTWAIT);
    if (!ret) {
        pr_err("send msg from kernel to usespace failed ret 0x%x\n", ret);
    }
}

static inline void nl_data_ready(struct sk_buff *__skb)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char str[100];
	skb = skb_get (__skb);
	if(skb->len >= NLMSG_SPACE(0))
	{
		nlh = nlmsg_hdr(skb);

		if (nlh != NULL) {
			memcpy(str, NLMSG_DATA(nlh), sizeof(str));
			pid = nlh->nlmsg_pid;
		}

		kfree_skb(skb);
	}

}


static inline int netlink_init(void)
{
	struct netlink_kernel_cfg netlink_cfg;
	memset(&netlink_cfg, 0, sizeof(struct netlink_kernel_cfg));

	netlink_cfg.groups = 0;
	netlink_cfg.flags = 0;
	netlink_cfg.input = nl_data_ready;
	netlink_cfg.cb_mutex = NULL;

	nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST,
			&netlink_cfg);

	if(!nl_sk){
		pr_err("create netlink socket error\n");
		return 1;
	}

	return 0;
}

static inline void netlink_exit(void)
{
	if(nl_sk != NULL){
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}

	pr_info("self module exited\n");
}

static inline int vreg_setup(struct gf_dev *goodix_fp, fp_power_info_t *pwr_info,
        bool enable)
{
    int rc;
    struct regulator *vreg;
    struct device *dev = &goodix_fp->spi->dev;
    const char *name = NULL;

    if (NULL == pwr_info || NULL == pwr_info->vreg_config.name) {
        pr_err("pwr_info is NULL\n");
        return -EINVAL;
    }
    vreg = pwr_info->vreg;
    name = pwr_info->vreg_config.name;
    pr_err("Regulator %s vreg_setup,enable=%d \n", name, enable);
    if (enable) {
        if (!vreg) {
            vreg = regulator_get(dev, name);
            if (IS_ERR(vreg)) {
                pr_err("Unable to get  %s\n", name);
                return PTR_ERR(vreg);
            }
        }
        if (regulator_count_voltages(vreg) > 0) {
            rc = regulator_set_voltage(vreg, pwr_info->vreg_config.vmin,
                    pwr_info->vreg_config.vmax);
            if (rc) {
                pr_err("Unable to set voltage on %s, %d\n", name, rc);
            }
        }
        rc = regulator_set_load(vreg, pwr_info->vreg_config.ua_load);
        if (rc < 0) {
            pr_err("Unable to set current on %s, %d\n", name, rc);
        }
        rc = regulator_enable(vreg);
        if (rc) {
            pr_err("error enabling %s: %d\n", name, rc);
            regulator_put(vreg);
            vreg = NULL;
        }
        pwr_info->vreg = vreg;
    } else {
        if (vreg) {
            if (regulator_is_enabled(vreg)) {
                regulator_disable(vreg);
                pr_err("disabled %s\n", name);
            }
            regulator_put(vreg);
            pwr_info->vreg = NULL;
        } else {
            pr_err("disable vreg is null \n");
        }
        rc = 0;
    }
    return rc;
}

static inline void gf_cleanup_pwr_list(struct gf_dev* gf_dev) {
    unsigned index = 0;
    pr_err("%s cleanup", __func__);
    for (index = 0; index < gf_dev->power_num; index++) {
        if (gf_dev->pwr_list[index].pwr_type == FP_POWER_MODE_GPIO) {
            gpio_free(gf_dev->pwr_list[index].pwr_gpio);
        }
        pr_debug("---- free pwr ok  with mode = %d, index = %d  ----\n",
                gf_dev->pwr_list[index].pwr_type, index);
        memset(&(gf_dev->pwr_list[index]), 0, sizeof(fp_power_info_t));
    }
}

static inline int gf_parse_pwr_list(struct gf_dev* gf_dev) {
    int ret = 0;
    struct device *dev = &gf_dev->spi->dev;
    struct device_node *np = dev->of_node;
    struct device_node *child = NULL;
    unsigned child_node_index = 0;
    int ldo_param_amount = 0;
    const char *node_name = NULL;
    fp_power_info_t *pwr_list = gf_dev->pwr_list;
    /* pwr list init */
    gf_cleanup_pwr_list(gf_dev);

    /* parse each node */
    for_each_available_child_of_node(np, child) {
        if (child_node_index >= FP_MAX_PWR_LIST_LEN) {
            pr_err("too many nodes");
            ret = -FP_ERROR_GENERAL;
            goto exit;
        }

        /* get type of this power */
        ret = of_property_read_u32(child, FP_POWER_NODE, &(pwr_list[child_node_index].pwr_type));
        if (ret) {
            pr_err("failed to request %s, ret = %d\n", FP_POWER_NODE, ret);
            goto exit;
        }

        pr_debug("read power type of index %d, type : %u\n", child_node_index, pwr_list[child_node_index].pwr_type);
        switch(pwr_list[child_node_index].pwr_type) {
        case FP_POWER_MODE_LDO:
            /* read ldo supply name */
            ret = of_property_read_string(child, FP_POWER_NAME_NODE, &(pwr_list[child_node_index].vreg_config.name));
            if (ret) {
                pr_err("the param %s is not found !\n", FP_POWER_NAME_NODE);
                ret = -FP_ERROR_GENERAL;
                goto exit;
            }
            pr_debug("get ldo node name %s", pwr_list[child_node_index].vreg_config.name);

            /* read ldo config name */
            ret = of_property_read_string(child, FP_POWER_CONFIG, &node_name);
            if (ret) {
                pr_err("the param %s is not found !\n", FP_POWER_CONFIG);
                ret = -FP_ERROR_GENERAL;
                goto exit;
            }
            pr_debug("get config node name %s", node_name);

            ldo_param_amount = of_property_count_elems_of_size(np, node_name, sizeof(u32));
            pr_debug("get ldo_param_amount %d", ldo_param_amount);
            if(ldo_param_amount != LDO_PARAM_AMOUNT) {
                pr_err("failed to request size %s\n", node_name);
                ret = -FP_ERROR_GENERAL;
                goto exit;
            }

            ret = of_property_read_u32_index(np, node_name, LDO_VMAX_INDEX, &(pwr_list[child_node_index].vreg_config.vmax));
            if (ret) {
                pr_err("failed to request %s(%d), rc = %u\n", node_name, LDO_VMAX_INDEX, ret);
                goto exit;
            }

            ret = of_property_read_u32_index(np, node_name, LDO_VMIN_INDEX, &(pwr_list[child_node_index].vreg_config.vmin));
            if (ret) {
                pr_err("failed to request %s(%d), rc = %u\n", node_name, LDO_VMIN_INDEX, ret);
                goto exit;
            }

            ret = of_property_read_u32_index(np, node_name, LDO_UA_INDEX, &(pwr_list[child_node_index].vreg_config.ua_load));
            if (ret) {
                pr_err("failed to request %s(%d), rc = %u\n", node_name, LDO_UA_INDEX, ret);
                goto exit;
            }

            pr_debug("%s size = %d, ua=%d, vmax=%d, vmin=%d\n", node_name, ldo_param_amount,
                    pwr_list[child_node_index].vreg_config.ua_load,
                    pwr_list[child_node_index].vreg_config.vmax,
                    pwr_list[child_node_index].vreg_config.vmin);
            break;
        case FP_POWER_MODE_GPIO:
            /* read GPIO name */
            ret = of_property_read_string(child, FP_POWER_NAME_NODE, &node_name);
            if (ret) {
                pr_err("the param %s is not found !\n", FP_POWER_NAME_NODE);
                ret = -FP_ERROR_GENERAL;
                goto exit;
            }
            pr_debug("get config node name %s", node_name);

            /* get gpio by name */
            gf_dev->pwr_list[child_node_index].pwr_gpio = of_get_named_gpio(np, node_name, 0);
            pr_debug("end of_get_named_gpio %s, pwr_gpio: %d!\n", node_name, pwr_list[child_node_index].pwr_gpio);
            if (gf_dev->pwr_list[child_node_index].pwr_gpio < 0) {
                pr_err("falied to get goodix_pwr gpio!\n");
                ret = -FP_ERROR_GENERAL;
                goto exit;
            }

            /* get poweron-level of gpio */
            pr_debug("get poweron level: %s", FP_POWERON_LEVEL_NODE);
            ret = of_property_read_u32(child, FP_POWERON_LEVEL_NODE, &pwr_list[child_node_index].poweron_level);
            if (ret) {
                /* property of poweron-level is not config, by default set to 1 */
                pwr_list[child_node_index].poweron_level = 1;
            } else {
                if (pwr_list[child_node_index].poweron_level != 0) {
                    pwr_list[child_node_index].poweron_level = 1;
                }
            }
            pr_debug("gpio poweron level: %d\n", pwr_list[child_node_index].poweron_level);

            ret = devm_gpio_request(dev, pwr_list[child_node_index].pwr_gpio, node_name);
            if (ret) {
                pr_err("failed to request %s gpio, ret = %d\n", node_name, ret);
                goto exit;
            }
            gpio_direction_output(pwr_list[child_node_index].pwr_gpio, (pwr_list[child_node_index].poweron_level == 0 ? 1: 0));
            pr_err("set goodix_pwr %u output %d \n", child_node_index, pwr_list[child_node_index].poweron_level);
            break;
        default:
            pr_err("unknown type %u\n", pwr_list[child_node_index].pwr_type);
            ret = -FP_ERROR_GENERAL;
            goto exit;
        }

        /* get delay time of this power */
        ret = of_property_read_u32(child, FP_POWER_DELAY_TIME, &pwr_list[child_node_index].delay);
        if (ret) {
            pr_err("failed to request %s, ret = %d\n", FP_POWER_DELAY_TIME, ret);
            goto exit;
        }
        child_node_index++;
    }
    gf_dev->power_num = child_node_index;
exit:
    if (ret) {
        gf_cleanup_pwr_list(gf_dev);
    }
    return ret;
}

static inline void gf_cleanup_ftm_poweroff_flag(struct gf_dev* gf_dev) {
    gf_dev->ftm_poweroff_flag = 0;
    pr_err("%s cleanup", __func__);
}

static inline int gf_parse_ftm_poweroff_flag(struct gf_dev* gf_dev) {
    int ret = 0;
    struct device *dev = &gf_dev->spi->dev;
    struct device_node *np = dev->of_node;
    uint32_t ftm_poweroff_flag = 0;

    gf_cleanup_ftm_poweroff_flag(gf_dev);

    ret = of_property_read_u32(np, FP_FTM_POWEROFF_FLAG, &ftm_poweroff_flag);
    if (ret) {
        pr_err("failed to request %s, ret = %d\n", FP_FTM_POWEROFF_FLAG, ret);
        goto exit;
    }
    gf_dev->ftm_poweroff_flag = ftm_poweroff_flag;
    pr_err("gf_dev->ftm_poweroff_flag = %d\n", gf_dev->ftm_poweroff_flag);

exit:
    if (ret) {
        gf_cleanup_ftm_poweroff_flag(gf_dev);
    }
    return ret;
}


static inline void gf_cleanup_notify_tpinfo_flag(struct gf_dev* gf_dev) {
    gf_dev->notify_tpinfo_flag = 0;
    pr_err("%s cleanup", __func__);
}

static inline int gf_parse_notify_tpinfo_flag(struct gf_dev* gf_dev) {
    int ret = 0;
    struct device *dev = &gf_dev->spi->dev;
    struct device_node *np = dev->of_node;
    uint32_t notify_tpinfo_flag = 0;

    gf_cleanup_notify_tpinfo_flag(gf_dev);

    ret = of_property_read_u32(np, FP_NOTIFY_TPINFO_FLAG, &notify_tpinfo_flag);
    if (ret) {
        pr_err("failed to request %s, ret = %d\n", FP_NOTIFY_TPINFO_FLAG, ret);
        goto exit;
    }
    gf_dev->notify_tpinfo_flag = notify_tpinfo_flag;
    pr_err("gf_dev->notify_tpinfo_flag = %d\n", gf_dev->notify_tpinfo_flag);

exit:
    if (ret) {
        gf_cleanup_notify_tpinfo_flag(gf_dev);
    }
    return ret;
}

static inline int gf_parse_dts(struct gf_dev* gf_dev)
{
    int rc = 0;
    struct device *dev = &gf_dev->spi->dev;
    struct device_node *np = dev->of_node;

    gf_dev->reset_gpio = of_get_named_gpio(np, "goodix,gpio_reset", 0);
    if (gf_dev->reset_gpio < 0) {
        pr_err("falied to get reset gpio!\n");
        return gf_dev->reset_gpio;
    }

    rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
    if (rc) {
        pr_err("failed to request reset gpio, rc = %d\n", rc);
        goto err_reset;
    }
    gpio_direction_output(gf_dev->reset_gpio, 0);

    gf_dev->irq_gpio = of_get_named_gpio(np, "goodix,gpio_irq", 0);
    if (gf_dev->irq_gpio < 0) {
        pr_err("falied to get irq gpio!\n");
        return gf_dev->irq_gpio;
    }

    rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
    if (rc) {
        pr_err("failed to request irq gpio, rc = %d\n", rc);
        goto err_irq;
    }
    gpio_direction_input(gf_dev->irq_gpio);

    rc = gf_parse_pwr_list(gf_dev);
    if (rc) {
        pr_err("failed to parse power list, rc = %d\n", rc);
        goto err_pwr;
    }
    pr_err("end gf_parse_dts !\n");

    gf_parse_notify_tpinfo_flag(gf_dev);

    pr_err("end gf_parse_dts !\n");

    return rc;

err_pwr:
    gf_cleanup_pwr_list(gf_dev);
err_irq:
    devm_gpio_free(dev, gf_dev->reset_gpio);
err_reset:
    return rc;
}

static inline void gf_cleanup(struct gf_dev *gf_dev)
{
    pr_debug("[info] %s\n",__func__);
    if (gpio_is_valid(gf_dev->irq_gpio))
    {
        gpio_free(gf_dev->irq_gpio);
        pr_debug("remove irq_gpio success\n");
    }
    if (gpio_is_valid(gf_dev->reset_gpio))
    {
        gpio_free(gf_dev->reset_gpio);
        pr_debug("remove reset_gpio success\n");
    }

    gf_cleanup_pwr_list(gf_dev);
}

/*power on auto during boot, no need fp driver power on*/
static inline int gf_power_on(struct gf_dev* gf_dev)
{
    int rc = 0;
    unsigned index = 0;

    for (index = 0; index < gf_dev->power_num; index++) {
        switch (gf_dev->pwr_list[index].pwr_type) {
        case FP_POWER_MODE_LDO:
            rc = vreg_setup(gf_dev, &(gf_dev->pwr_list[index]), true);
            pr_debug("---- power on ldo ----\n");
            break;
        case FP_POWER_MODE_GPIO:
            gpio_set_value(gf_dev->pwr_list[index].pwr_gpio, gf_dev->pwr_list[index].poweron_level);
            pr_debug("set pwr_gpio %d\n", gf_dev->pwr_list[index].poweron_level);
            break;
        case FP_POWER_MODE_AUTO:
            pr_debug("[%s] power on auto, no need power on again\n", __func__);
            break;
        case FP_POWER_MODE_NOT_SET:
        default:
            rc = -1;
            pr_debug("---- power on mode not set !!! ----\n");
            break;
        }
        if (rc) {
            pr_err("---- power on failed with mode = %d, index = %d, rc = %d ----\n",
                    gf_dev->pwr_list[index].pwr_type, index, rc);
            break;
        } else {
            pr_debug("---- power on ok with mode = %d, index = %d  ----\n",
                    gf_dev->pwr_list[index].pwr_type, index);
        }
        msleep(gf_dev->pwr_list[index].delay);
    }

    msleep(30);
    return rc;
}

/*power off auto during shut down, no need fp driver power off*/
static inline int gf_power_off(struct gf_dev* gf_dev)
{
    int rc = 0;
    unsigned index = 0;

    for (index = 0; index < gf_dev->power_num; index++) {
        switch (gf_dev->pwr_list[index].pwr_type) {
        case FP_POWER_MODE_LDO:
            rc = vreg_setup(gf_dev, &(gf_dev->pwr_list[index]), false);
            pr_debug("---- power on ldo ----\n");
            break;
        case FP_POWER_MODE_GPIO:
            gpio_set_value(gf_dev->pwr_list[index].pwr_gpio, (gf_dev->pwr_list[index].poweron_level == 0 ? 1 : 0));
            pr_debug("set pwr_gpio %d\n", (gf_dev->pwr_list[index].poweron_level == 0 ? 1 : 0));
            break;
        case FP_POWER_MODE_AUTO:
            pr_debug("[%s] power on auto, no need power on again\n", __func__);
            break;
        case FP_POWER_MODE_NOT_SET:
        default:
            rc = -1;
            pr_debug("---- power on mode not set !!! ----\n");
            break;
        }
        if (rc) {
            pr_err("---- power off failed with mode = %d, index = %d, rc = %d ----\n",
                    gf_dev->pwr_list[index].pwr_type, index, rc);
            break;
        } else {
            pr_debug("---- power off ok with mode = %d, index = %d  ----\n",
                    gf_dev->pwr_list[index].pwr_type, index);
        }
    }

    msleep(30);
    return rc;
}

static inline int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
    if (gf_dev == NULL)
    {
        pr_debug("Input buff is NULL.\n");
        return -1;
    }
    //gpio_direction_output(gf_dev->reset_gpio, 1);
    gpio_set_value(gf_dev->reset_gpio, 0);
    mdelay(5);
    gpio_set_value(gf_dev->reset_gpio, 1);
    mdelay(delay_ms);
    return 0;
}

static inline int gf_power_reset(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
        pr_debug("Input buff is NULL.\n");
        return -1;
	}
	gpio_set_value(gf_dev->reset_gpio, 0);
	gf_power_off(gf_dev);
	mdelay(50);
	gf_power_on(gf_dev);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(3);
	return 0;
}

static inline int gf_irq_num(struct gf_dev *gf_dev)
{
    if(gf_dev == NULL) {
        pr_debug("Input buff is NULL.\n");
        return -1;
    } else {
        return gpio_to_irq(gf_dev->irq_gpio);
    }
}


static inline int gf_opticalfp_irq_handler(struct fp_underscreen_info *tp_info);

static inline void gf_enable_irq(struct gf_dev *gf_dev)
{
    if (gf_dev->irq_enabled) {
        pr_warn("IRQ has been enabled.\n");
    } else {
        enable_irq(gf_dev->irq);
        gf_dev->irq_enabled = 1;
    }
}

static inline void gf_disable_irq(struct gf_dev *gf_dev)
{
    if (gf_dev->irq_enabled) {
        gf_dev->irq_enabled = 0;
        disable_irq(gf_dev->irq);
    } else {
        pr_warn("IRQ has been disabled.\n");
    }
}

static inline irqreturn_t gf_irq(int irq, void *handle)
{
#if defined(GF_NETLINK_ENABLE)
    char msg = GF_NET_EVENT_IRQ;
    wake_lock_timeout(&fp_wakelock, msecs_to_jiffies(WAKELOCK_HOLD_TIME));
    sendnlmsg(&msg);
#elif defined (GF_FASYNC)
    struct gf_dev *gf_dev = &gf;
    if (gf_dev->async) {
        kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
    }
#endif

    return IRQ_HANDLED;
}

static inline int irq_setup(struct gf_dev *gf_dev)
{
    int status;

    gf_dev->irq = gf_irq_num(gf_dev);
    status = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT, "gf", gf_dev);

    if (status) {
        pr_err("failed to request IRQ:%d\n", gf_dev->irq);
        return status;
    }
    enable_irq_wake(gf_dev->irq);
    gf_dev->irq_enabled = 1;

    return status;
}

static inline void irq_cleanup(struct gf_dev *gf_dev)
{
    gf_dev->irq_enabled = 0;
    disable_irq(gf_dev->irq);
    disable_irq_wake(gf_dev->irq);
    free_irq(gf_dev->irq, gf_dev);//need modify
}

static inline void gf_auto_send_touchdown(void)
{
	struct fp_underscreen_info tp_info;
	tp_info.touch_state = 1;
	gf_opticalfp_irq_handler(&tp_info);
}

static inline void gf_auto_send_touchup(void)
{
	struct fp_underscreen_info tp_info;
	tp_info.touch_state = 0;
	gf_opticalfp_irq_handler(&tp_info);
}

static inline long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gf_dev *gf_dev = &gf;
    int retval = 0;
    u8 netlink_route = NETLINK_TEST;
    struct gf_ioc_chip_info info;

    if (_IOC_TYPE(cmd) != GF_IOC_MAGIC) {
        return -ENODEV;
    }

    if (_IOC_DIR(cmd) & _IOC_READ) {
        #if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
        retval = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
        #else
        retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
        #endif
    } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
        #if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
        retval = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
        #else
        retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
        #endif
    }
    if (retval) {
        return -EFAULT;
    }

    if (gf_dev->device_available == 0) {
        if ((cmd == GF_IOC_ENABLE_POWER) || (cmd == GF_IOC_DISABLE_POWER) || (cmd == GF_IOC_POWER_RESET)) {
            pr_info("power cmd\n");
        } else {
            pr_info("Sensor is power off currently. \n");
            return -ENODEV;
        }
    }

	switch (cmd) {
        case GF_IOC_INIT:
            pr_debug("%s GF_IOC_INIT\n", __func__);
            if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
                retval = -EFAULT;
                break;
            }
            break;
        case GF_IOC_EXIT:
            pr_debug("%s GF_IOC_EXIT\n", __func__);
            break;
        case GF_IOC_DISABLE_IRQ:
            pr_debug("%s GF_IOC_DISABEL_IRQ\n", __func__);
            gf_disable_irq(gf_dev);
            break;
        case GF_IOC_ENABLE_IRQ:
            pr_debug("%s GF_IOC_ENABLE_IRQ\n", __func__);
            gf_enable_irq(gf_dev);
            break;
        case GF_IOC_RESET:
            pr_info("%s GF_IOC_RESET. \n", __func__);
            gf_hw_reset(gf_dev, 10);
            break;
	case GF_IOC_POWER_RESET:
		pr_info("%s GF_IOC_POWER_RESET. \n", __func__);
		gf_power_reset(gf_dev);
		gf_dev->device_available = 1;
		break;
        case GF_IOC_ENABLE_SPI_CLK:
            pr_debug("%s GF_IOC_ENABLE_SPI_CLK\n",  __func__);
            pr_debug("Doesn't support control clock.\n");
            break;
        case GF_IOC_DISABLE_SPI_CLK:
            pr_debug("%s GF_IOC_DISABLE_SPI_CLK\n", __func__);
            pr_debug("Doesn't support control clock\n");
            break;
        case GF_IOC_ENABLE_POWER:
            pr_debug("%s GF_IOC_ENABLE_POWER\n", __func__);
            if (gf_dev->device_available == 1)
                pr_info("Sensor has already powered-on.\n");
            else
                gf_power_on(gf_dev);
            gf_dev->device_available = 1;
            break;
        case GF_IOC_DISABLE_POWER:
            pr_debug("%s GF_IOC_DISABLE_POWER\n", __func__);
            if (gf_dev->device_available == 0)
                pr_info("Sensor has already powered-off.\n");
            else
                gf_power_off(gf_dev);
            gf_dev->device_available = 0;
            break;
        case GF_IOC_ENTER_SLEEP_MODE:
            pr_debug("%s GF_IOC_ENTER_SLEEP_MODE\n", __func__);
            break;
        case GF_IOC_GET_FW_INFO:
            pr_debug("%s GF_IOC_GET_FW_INFO\n", __func__);
            break;

        case GF_IOC_REMOVE:
            irq_cleanup(gf_dev);
            gf_cleanup(gf_dev);
            pr_debug("%s GF_IOC_REMOVE\n", __func__);
            break;

        case GF_IOC_CHIP_INFO:
            pr_debug("%s GF_IOC_CHIP_INFO\n", __func__);
            if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg, sizeof(struct gf_ioc_chip_info))) {
                retval = -EFAULT;
                break;
            }
            pr_info("vendor_id : 0x%x\n", info.vendor_id);
            pr_info("mode : 0x%x\n", info.mode);
            pr_info("operation: 0x%x\n", info.operation);
            break;
        case GF_IOC_WAKELOCK_TIMEOUT_ENABLE:
            pr_debug("%s GF_IOC_WAKELOCK_TIMEOUT_ENABLE\n", __func__);
            wake_lock_timeout(&gf_cmd_wakelock, msecs_to_jiffies(SENDCMD_WAKELOCK_HOLD_TIME));
            break;
        case GF_IOC_WAKELOCK_TIMEOUT_DISABLE:
            pr_debug("%s GF_IOC_WAKELOCK_TIMEOUT_DISABLE\n", __func__);
            wake_unlock(&gf_cmd_wakelock);
            break;
        case GF_IOC_CLEAN_TOUCH_FLAG:
            lasttouchmode = 0;
            pr_debug("%s GF_IOC_CLEAN_TOUCH_FLAG\n", __func__);
            break;
	case GF_IOC_AUTO_SEND_TOUCHDOWN:
		pr_info("%s GF_IOC_AUTO_SEND_TOUCHDOWN\n", __func__);
		gf_auto_send_touchdown();
		break;
	case GF_IOC_AUTO_SEND_TOUCHUP:
		pr_info("%s GF_IOC_AUTO_SEND_TOUCHUP\n", __func__);
		gf_auto_send_touchup();
		break;
        default:
            pr_warn("unsupport cmd:0x%x\n", cmd);
            break;
    }

    return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/


static inline int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev = &gf;
    int status = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
        if (gf_dev->devt == inode->i_rdev) {
            pr_info("Found\n");
            status = 0;
            break;
        }
    }

    if (status == 0) {
        if (status == 0) {
            gf_dev->users++;
            filp->private_data = gf_dev;
            nonseekable_open(inode, filp);
            pr_info("Succeed to open device. irq = %d\n",
                    gf_dev->irq);
            if (gf_dev->users == 1) {
                status = gf_parse_dts(gf_dev);
                if (status)
                    goto err_parse_dt;

                status = irq_setup(gf_dev);
                if (status)
                    goto err_irq;
            }
        }
    } else {
        pr_info("No device for minor %d\n", iminor(inode));
    }
    mutex_unlock(&device_list_lock);

    return status;
err_irq:
    gf_cleanup(gf_dev);
err_parse_dt:
    return status;
}

static inline int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev = &gf;
    int status = 0;

    mutex_lock(&device_list_lock);
    gf_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close?? */
    gf_dev->users--;
    if (!gf_dev->users) {
        irq_cleanup(gf_dev);
        gf_cleanup(gf_dev);
        /*power off the sensor*/
        gf_dev->device_available = 0;
    }
    mutex_unlock(&device_list_lock);
    return status;
}

static const struct file_operations gf_fops = {
    .owner = THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
    .open = gf_open,
    .release = gf_release,
};

static inline int goodix_fb_state_chg_callback(struct notifier_block *nb,
        unsigned long val, void *data)
{
    struct gf_dev *gf_dev;
    struct fb_event *evdata = data;
    unsigned int blank;
    char msg = 0;
    int retval = 0;

    gf_dev = container_of(nb, struct gf_dev, notifier);

    if (val == MSM_DRM_ONSCREENFINGERPRINT_EVENT) {
        uint8_t op_mode = 0x0;
        op_mode = *(uint8_t *)evdata->data;

        switch (op_mode) {
            case 0:
                pr_info("[%s] UI disappear\n", __func__);
                break;
            case 1:
                pr_info("[%s] UI ready \n", __func__);
                msg = GF_NET_EVENT_UI_READY;
                sendnlmsg(&msg);
                break;
            default:
                pr_info("[%s] Unknown MSM_DRM_ONSCREENFINGERPRINT_EVENT\n", __func__);
                break;
        }
        return retval;
    }

    if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf_dev) {
        blank = *(int *)(evdata->data);
        switch (blank) {
            case FB_BLANK_POWERDOWN:
                if (gf_dev->device_available == 1) {
                    gf_dev->fb_black = 1;
                    msg = GF_NET_EVENT_FB_BLACK;
                    sendnlmsg(&msg);
                }
                break;
            case FB_BLANK_UNBLANK:
                if (gf_dev->device_available == 1) {
                    gf_dev->fb_black = 0;
                    msg = GF_NET_EVENT_FB_UNBLACK;
                    sendnlmsg(&msg);
                }
                break;
            default:
                pr_info("%s defalut\n", __func__);
                break;
        }
    }
    return NOTIFY_OK;
}

static struct notifier_block goodix_noti_block = {
    .notifier_call = goodix_fb_state_chg_callback,
};

static inline int gf_opticalfp_irq_handler(struct fp_underscreen_info *tp_info)
{
    char msg = 0;
    fp_tpinfo = *tp_info;
    if(tp_info->touch_state== lasttouchmode){
        return IRQ_HANDLED;
    }
    wake_lock_timeout(&fp_wakelock, msecs_to_jiffies(WAKELOCK_HOLD_TIME));
    if (1 == tp_info->touch_state) {
        msg = GF_NET_EVENT_TP_TOUCHDOWN;
        sendnlmsg(&msg);
        lasttouchmode = tp_info->touch_state;
    } else {
        msg = GF_NET_EVENT_TP_TOUCHUP;
        sendnlmsg(&msg);
        lasttouchmode = tp_info->touch_state;
    }

    return IRQ_HANDLED;
}


static struct class *gf_class;
static inline int gf_probe(struct platform_device *pdev)
{
    struct gf_dev *gf_dev = &gf;
    int status = -EINVAL;
    unsigned long minor;
    int boot_mode = 0;
    int i;
    /* Initialize the driver data */
    INIT_LIST_HEAD(&gf_dev->device_entry);
    gf_dev->spi = pdev;
    gf_dev->irq_gpio = -EINVAL;
    gf_dev->reset_gpio = -EINVAL;
    gf_dev->pwr_gpio = -EINVAL;
    gf_dev->device_available = 0;
    gf_dev->fb_black = 0;

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
        dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
                gf_dev, GF_DEV_NAME);
        status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
        dev_dbg(&gf_dev->spi->dev, "no minor number available!\n");
        status = -ENODEV;
        mutex_unlock(&device_list_lock);
        goto error_hw;
    }

    if (status == 0) {
        set_bit(minor, minors);
        list_add(&gf_dev->device_entry, &device_list);
    } else {
        gf_dev->devt = 0;
    }
    mutex_unlock(&device_list_lock);

    if (status == 0)
    {
        /*input device subsystem */
        gf_dev->input = input_allocate_device();
        if (gf_dev->input == NULL)
        {
            pr_err("%s, failed to allocate input device\n", __func__);
            status = -ENOMEM;
            goto error_dev;
        }

        gf_dev->input->name = GF_INPUT_NAME;
        status = input_register_device(gf_dev->input);
        if (status)
        {
            pr_err("failed to register input device\n");
            goto error_input;
        }
    }

    gf_dev->notifier = goodix_noti_block;
#if IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY) || IS_ENABLED(CONFIG_DRM_MSM)
    status = msm_drm_register_client(&gf_dev->notifier);
    if (status == -1) {
        return status;
    }
#elif defined(CONFIG_FB)
    status = fb_register_client(&gf_dev->notifier);
    if (status == -1) {
        return status;
    }
#endif
    wake_lock_init(&fp_wakelock, WAKE_LOCK_SUSPEND, "fp_wakelock");
    wake_lock_init(&gf_cmd_wakelock, WAKE_LOCK_SUSPEND, "gf_cmd_wakelock");
    pr_err("register goodix_fp_ok\n");
    pr_info("version V%d.%d.%02d\n", VER_MAJOR, VER_MINOR, PATCH_LEVEL);

    gf_parse_ftm_poweroff_flag(gf_dev);
    if (gf_dev->ftm_poweroff_flag) {
        boot_mode = get_boot_mode();
        if (MSM_BOOT_MODE__FACTORY == boot_mode)
        {
            pr_err("enter to fastbootmode,and power off\n");
            status = gf_parse_pwr_list(gf_dev);
            if (status) {
                pr_err("failed to parse power list, status = %d\n", status);
                gf_cleanup_pwr_list(gf_dev);
            } else {
                pr_err("enter to power off\n");
                gf_power_on(gf_dev);
                gf_power_off(gf_dev);
            }
        }
    }

    return status;

error_input:
    if (gf_dev->input != NULL)
        input_free_device(gf_dev->input);
error_dev:
    if (gf_dev->devt != 0)
    {
        pr_info("Err: status = %d\n", status);
        mutex_lock(&device_list_lock);
        list_del(&gf_dev->device_entry);
        device_destroy(gf_class, gf_dev->devt);
        clear_bit(MINOR(gf_dev->devt), minors);
        mutex_unlock(&device_list_lock);
    }
error_hw:
    gf_dev->device_available = 0;
    boot_mode = get_boot_mode();
    if (MSM_BOOT_MODE__FACTORY == boot_mode)
    {
        gf_power_off(gf_dev);
    }

    return status;
}

static inline int gf_remove(struct platform_device *pdev)
{
    struct gf_dev *gf_dev = &gf;
    wake_lock_destroy(&fp_wakelock);
    wake_lock_destroy(&gf_cmd_wakelock);

    fb_unregister_client(&gf_dev->notifier);
    if (gf_dev->input)
        input_unregister_device(gf_dev->input);
    input_free_device(gf_dev->input);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&gf_dev->device_entry);
    device_destroy(gf_class, gf_dev->devt);
    clear_bit(MINOR(gf_dev->devt), minors);
    mutex_unlock(&device_list_lock);

    return 0;
}

static struct of_device_id gx_match_table[] = {
    { .compatible = GF_SPIDEV_NAME },
    {},
};

static struct platform_driver gf_driver = {
    .driver = {
        .name = GF_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = gx_match_table,
    },
    .probe = gf_probe,
    .remove = gf_remove,
};

static int __init gf_init(void)
{
    int status;
    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */

    if ((FP_GOODIX_3268 != get_fpsensor_type())
            && (FP_GOODIX_5288 != get_fpsensor_type())
            && (FP_GOODIX_5228 != get_fpsensor_type())
            && (FP_GOODIX_5658 != get_fpsensor_type())
            && (FP_GOODIX_OPTICAL_95 != get_fpsensor_type())
            && (FP_GOODIX_3626 != get_fpsensor_type())) {
        pr_err("%s, found not goodix sensor\n", __func__);
        status = -EINVAL;
        return status;
    }

    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0) {
        pr_warn("Failed to register char device!\n");
        return status;
    }
    SPIDEV_MAJOR = status;
    gf_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gf_class)) {
        unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
        pr_warn("Failed to create class.\n");
        return PTR_ERR(gf_class);
    }

    status = platform_driver_register(&gf_driver);

    if (status < 0) {
        class_destroy(gf_class);
        unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
        pr_warn("Failed to register SPI driver.\n");
        return status;
    }

    netlink_init();

    /*Register for receiving tp touch event.
     * Must register after get_fpsensor_type filtration as only one handler can be registered.
     */
    opticalfp_irq_handler_register(gf_opticalfp_irq_handler);
    pr_info("status = 0x%x\n", status);
    return 0;
}
late_initcall(gf_init);

static void __exit gf_exit(void)
{
    platform_driver_unregister(&gf_driver);
    class_destroy(gf_class);
    unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
}
module_exit(gf_exit);

MODULE_SOFTDEP("pre:oplus_fp_common");
MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_AUTHOR("Jandy Gou, <gouqingsong@goodix.com>");
MODULE_DESCRIPTION("goodix fingerprint sensor device driver");
MODULE_LICENSE("GPL");
