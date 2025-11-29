/*
 * Copyright (c) 2025 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ft_ft90_wdt

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>

#include <wdt_drv.h>

LOG_MODULE_REGISTER(ft_ft90_wdt, CONFIG_WDT_LOG_LEVEL);

struct wdt_ft_config
{
    WDT_TypeDef *base;
    uint32_t clkid;
    struct reset_dt_spec reset;
};

struct wdt_ft_data
{
    bool en_flag;
    bool wmr_flag;
};

static int wdt_ft_setup(const struct device *dev, uint8_t options)
{
    ARG_UNUSED(dev);
    struct wdt_ft_data *wdt_datas = dev->data;
    const struct wdt_ft_config *cfg = dev->config;
    if ((options & WDT_OPT_PAUSE_HALTED_BY_DBG) != 0U)
    {
        Wdt_SetMode(cfg->base, WDT_DBG);
    }
    else
    {
        wdt_datas->en_flag = false;
        if ((options & WDT_OPT_PAUSE_IN_SLEEP) != 0U)
        {
            Wdt_SetMode(cfg->base, WDT_DOZE);
        }
    }

    Wdt_EnableFunc(cfg->base);

    return 0;
}

/*
 *Disable the watchdog.
 *In debug (test) mode, multiple writes are allowed.
 *In normal mode, only the first write is effective; further writes are ignored.
 */
static int wdt_ft_disable(const struct device *dev)
{
    /* watchdog cannot be stopped once started */
    ARG_UNUSED(dev);
    struct wdt_ft_data *wdt_datas = dev->data;
    const struct wdt_ft_config *cfg = dev->config;
    bool ret = false;
    ret = Wdt_IsInDbg(cfg->base);
    if (ret)
    {
        Wdt_DisableFunc(cfg->base);
        return 0;
    }
    else if (!Wdt_IsEnabled(cfg->base))
    {
        Wdt_DisableFunc(cfg->base);
        return 0;
    }
    else if (wdt_datas->en_flag)
    {
        wdt_datas->en_flag = false;
        Wdt_DisableFunc(cfg->base);
        return 0;
    }
    else
    {
        return -EPERM;
    }
}

static int wdt_ft_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *config)
{
    uint32_t prescaler = 0U;
    uint32_t reload = 0U;
    uint16_t divider = 4096U;
    uint32_t ticks = (uint64_t)(80 * NSEC_PER_MSEC) * (config->window.max) / MSEC_PER_SEC;
    struct wdt_ft_data *wdt_datas = dev->data;
    const struct wdt_ft_config *cfg = dev->config;

    if (!(wdt_datas->wmr_flag))
    {
        LOG_ERR("Can't write wmr secondly!");
        return -EPERM;
    }
    else if (config->callback != NULL)
    {
        LOG_ERR("callback not supported by WDT");
        return -ENOTSUP;
    }
    else if (((ticks / divider) > 0xFFFF) || ((config->window.max) == 0))
    {
        LOG_ERR("window max is out of range");
        return -EINVAL;
    }
    else
    {
        wdt_datas->wmr_flag = false;
        Wdt_SetCnt(cfg->base, (ticks / divider));
    }

    return 0;
}

/**
 * @brief Feed (refresh) the watchdog timer
 *
 * This function reloads the watchdog counter to prevent it from timing out
 * and resetting the system.
 *
 * @param dev Pointer to the watchdog device structure
 * @param channel_id Channel ID (unused for this implementation)
 *
 * @return 0 on success
 */
static int wdt_ft_feed(const struct device *dev, int channel_id)
{
    ARG_UNUSED(channel_id);
    const struct wdt_ft_config *cfg = dev->config;
    Wdt_FeedDog(cfg->base);
    return 0;
}

static const struct wdt_driver_api wdt_ft_api = {
    .setup = wdt_ft_setup,
    .disable = wdt_ft_disable,
    .install_timeout = wdt_ft_install_timeout,
    .feed = wdt_ft_feed,
};

/**
 * @brief Initialize the watchdog (WDT) device
 *
 * This function enables the clock and toggles the reset line for the watchdog
 * using configuration data provided via device tree.
 *
 * @param dev Pointer to the device structure
 *
 * @return 0 on success, negative error code on failure
 */
static int wdt_ft_init(const struct device *dev)
{
    const struct wdt_ft_config *cfg = dev->config;
    const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
    int ret = 0;
    ret = clock_control_on(clk, (clock_control_subsys_t *)&cfg->clkid);
    if (ret < 0)
    {
        return ret;
    }
    ret = reset_line_toggle_dt(&cfg->reset);
    if (ret < 0)
    {
        return ret;
    }
    return ret;
}

static const struct wdt_ft_config wdt_cfg = {
    .base = (WDT_TypeDef *)DT_REG_ADDR(DT_INST(0, ft_ft90_wdt)),
    .clkid = DT_INST_CLOCKS_CELL(0, id),
    .reset = RESET_DT_SPEC_INST_GET(0),
};
static struct wdt_ft_data wdt_data = {
    .en_flag = true,
    .wmr_flag = true,
};

DEVICE_DT_INST_DEFINE(0, wdt_ft_init, NULL, &wdt_data, &wdt_cfg, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                      &wdt_ft_api);
