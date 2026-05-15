/*
 * Copyright (c) 2021 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Gigadevice SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_SOC_FOCALTECH_COMMON_H_
#define ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_SOC_FOCALTECH_COMMON_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#ifdef CONFIG_HAS_FT9002LIB
#include <zephyr/dt-bindings/pinctrl/ft9002s-pinctrl.h>
#endif
#ifdef CONFIG_HAS_FT9001LIB
#include <zephyr/dt-bindings/pinctrl/ft9001-pinctrl.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

/** @brief Type for FT90 pin.
 *
 * Bits (PINMUX model):
 * - 0-25: FT90_PINMUX bit field.
 * - 26-31: Pin configuration bit field (@ref FT90_PINCFG).
 */
typedef uint32_t pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)			       \
	(DT_PROP_BY_IDX(node_id, prop, idx) |				       \
	 ((FT90_PUPD_PULLUP * DT_PROP(node_id, bias_pull_up))		       \
	  << FT90_PUPD_POS) |						       \
	 ((FT90_PUPD_PULLDOWN * DT_PROP(node_id, bias_pull_down))	       \
	  << FT90_PUPD_POS) |						       \
	 ((FT90_OTYPE_OD * DT_PROP(node_id, drive_open_drain))		       \
	  << FT90_OTYPE_POS) |						       \
	 (DT_ENUM_IDX(node_id, slew_rate) << FT90_OSPEED_POS)),

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			       \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),		       \
				DT_FOREACH_PROP_ELEM, pinmux,		       \
				Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

/**
 * @name FT90 PUPD (values match the ones in the HAL for AF model).
 * @{
 */

/** No pull-up/down */
#define FT90_PUPD_NONE 0U
/** Pull-up */
#define FT90_PUPD_PULLUP 1U
/** Pull-down */
#define FT90_PUPD_PULLDOWN 2U

/** @} */

/**
 * @name FT90 OTYPE (values match the ones in the HAL for AF model).
 * @{
 */

/** Push-pull */
#define FT90_OTYPE_PP 0U
/** Open-drain */
#define FT90_OTYPE_OD 1U

/** @} */

/**
 * @name FT90 OSPEED (values match the ones in the HAL for AF model, mode minus
 * one for AFIO model).
 * @{
 */

#ifdef CONFIG_PINCTRL_FT90_AF
/** Maximum 2MHz */
#define FT90_OSPEED_2MHZ 0U
#if defined(CONFIG_SOC_SERIES_FT90F3X0) || \
	defined(CONFIG_SOC_SERIES_FT90A50X) || \
	defined(CONFIG_SOC_SERIES_FT90L23X)
/** Maximum 10MHz */
#define FT90_OSPEED_10MHZ 1U
/** Maximum 50MHz */
#define FT90_OSPEED_50MHZ 3U
#else
/** Maximum 25MHz */
#define FT90_OSPEED_25MHZ 1U
/** Maximum 50MHz */
#define FT90_OSPEED_50MHZ 2U
/** Maximum speed */
#define FT90_OSPEED_MAX 3U
#endif

#else /* CONFIG_PINCTRL_FT90_AF */
/** Maximum 10MHz */
#define FT90_OSPEED_10MHZ 0U
/** Maximum 2MHz */
#define FT90_OSPEED_2MHZ 1U
/** Maximum 50MHz */
#define FT90_OSPEED_50MHZ 2U
/** Maximum speed */
#define FT90_OSPEED_MAX 3U
#endif /* CONFIG_PINCTRL_FT90_AF */

/** @} */

/**
 * @name FT90 pin configuration bit field mask and positions.
 * @anchor FT90_PINCFG
 *
 * Fields:
 *
 * - 31..29: Pull-up/down
 * - 28:     Output type
 * - 27..26: Output speed
 *
 * @{
 */

/** PUPD field mask. */
#define FT90_PUPD_MSK 0x3U
/** PUPD field position. */
#define FT90_PUPD_POS 29U
/** OTYPE field mask. */
#define FT90_OTYPE_MSK 0x1U
/** OTYPE field position. */
#define FT90_OTYPE_POS 28U
/** OSPEED field mask. */
#define FT90_OSPEED_MSK 0x3U
/** OSPEED field position. */
#define FT90_OSPEED_POS 26U

/** @} */

/**
 * Obtain PUPD field from pinctrl_soc_pin_t configuration.
 *
 * @param pincfg pinctrl_soc_pin_t bit field value.
 */
#define FT90_PUPD_GET(pincfg) \
	(((pincfg) >> FT90_PUPD_POS) & FT90_PUPD_MSK)

/**
 * Obtain OTYPE field from pinctrl_soc_pin_t configuration.
 *
 * @param pincfg pinctrl_soc_pin_t bit field value.
 */
#define FT90_OTYPE_GET(pincfg) \
	(((pincfg) >> FT90_OTYPE_POS) & FT90_OTYPE_MSK)

/**
 * Obtain OSPEED field from pinctrl_soc_pin_t configuration.
 *
 * @param pincfg pinctrl_soc_pin_t bit field value.
 */
#define FT90_OSPEED_GET(pincfg) \
	(((pincfg) >> FT90_OSPEED_POS) & FT90_OSPEED_MSK)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_SOC_FT90_COMMON_H_ */
