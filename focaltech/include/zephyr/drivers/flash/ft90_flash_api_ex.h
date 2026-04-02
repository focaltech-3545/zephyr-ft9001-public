/*
 * Copyright (c) 2025 Focaltech
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __ZEPHYR_INCLUDE_DRIVERS_FT90_FLASH_API_EX_H__
#define __ZEPHYR_INCLUDE_DRIVERS_FT90_FLASH_API_EX_H__

/**
 * @brief Enumeration for Focaltech flash extended operations.
 */
enum flash_ft_xip_ex_ops {
	/**
	 * @brief Get the three status registers (SR1, SR2, SR3).
	 *
	 * This operation reads the contents of the three status registers from the flash device.
	 *
	 * @param out Pointer to a @ref andes_xip_ex_ops_get_out structure to store the register
	 *            values.
	 */
	FLASH_FT_XIP_EX_OP_GET_STATUS_REGS = FLASH_EX_OP_VENDOR_BASE,
	/**
	 * @brief Set the three status registers (SR1, SR2, SR3).
	 *
	 * This operation writes new values to the status registers, applying a mask to modify only
	 * specific bits.
	 * This operation is protected by a software lock that can be controlled with
	 * @ref FLASH_FT_XIP_EX_OP_LOCK.
	 *
	 * @param in Pointer to a @ref ft_xip_ex_ops_set_in structure containing the values and
	 *           masks to write.
	 */
	FLASH_FT_XIP_EX_OP_SET_STATUS_REGS,
	/**
	 * @brief Set a software lock to prevent status register modification.
	 *
	 * This operation enables or disables a software lock that prevents the
	 * @ref FLASH_FT_XIP_EX_OP_SET_STATUS_REGS operation from executing.
	 *
	 * @param in Pointer to a @ref ft_xip_ex_ops_lock_in structure specifying whether to
	 *           enable or disable the lock.
	 */
	FLASH_FT_XIP_EX_OP_LOCK,
	/**
	 * @brief Get the current state of the software status register lock.
	 *
	 * @param out Pointer to a @ref ft_xip_ex_ops_lock_state_out structure to store the
	 *            current lock state.
	 */
	FLASH_FT_XIP_EX_OP_LOCK_STATE,
	/**
	 * @brief Set the SPI command for memory-mapped read mode.
	 *
	 * This operation configures the command used by the hardware for Execute-In-Place (XIP) or
	 * memory-mapped reads.
	 *
	 * @param in Pointer to a @ref ft_xip_ex_ops_mem_read_cmd_in structure specifying the
	 *           read command to use.
	 */
	FLASH_FT_XIP_EX_OP_MEM_READ_CMD,
};

/**
 * @brief Output parameters for @ref FLASH_FT_XIP_EX_OP_GET_STATUS_REGS operation.
 */
struct ft_xip_ex_ops_get_out {
	/** Buffer for read status registers. */
	uint8_t regs[3];
};

/**
 * @brief Input parameters for @ref FLASH_FT_XIP_EX_OP_SET_STATUS_REGS operation.
 */
struct ft_xip_ex_ops_set_in {
	/** Status registers to write. */
	uint8_t regs[3];
	/** Mask of status registers to change. */
	uint8_t masks[3];
};

/**
 * @brief Input parameters for @ref FLASH_FT_XIP_EX_OP_LOCK operation.
 */
struct ft_xip_ex_ops_lock_in {
	/** Set to true to enable the lock, false to disable. */
	bool enable;
};

/**
 * @brief Output parameters for @ref FLASH_FT_XIP_EX_OP_LOCK_STATE operation.
 */
struct ft_xip_ex_ops_lock_state_out {
	/** Current lock state. */
	bool state;
};

#endif /* __ZEPHYR_INCLUDE_DRIVERS_FT90_FLASH_API_EX_H__ */
