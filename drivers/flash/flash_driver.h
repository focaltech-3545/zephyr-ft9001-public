/*****************************************************************************
Copyright: 2018-2019, Focal Tech. Co., Ltd.
File name: flash_drv.h
Description: FT920 flash init, read, write, etc.
Author: Wan Mingming
Version: SV0.1
Date: 2018/03/20
History: Modify history list. Each modification record should
include date, modifier, and modification content.
*****************************************************************************/

#ifndef __FLASH_DRIVER_H__
#define __FLASH_DRIVER_H__

#include <stdint.h>
// #include "ssi_hal.h"

#define SSI_PAGE_SIZE (0x100)
#define SSI_SECTOR_SIZE (0x1000)

// FLASH start Address
#define FLASH_ADDR_START 0x10000000 // FLASH start Address
#define FLASH_ADDR_END 0x10200000
#define FLASH_PAGE_SIZE SSI_PAGE_SIZE
#define FLASH_SECTOR_SIZE SSI_SECTOR_SIZE
#define FLASH_BLOCK_SIZE (0x8000)
#define FLASH_WAITETIME 5 // FLASH wait timeout

#define APP_FLASH_ID 1
#define KEY_FLASH_ID 2
#define CFG_FLASH_ID 3
#define TPL_FLASH_ID 4

#define IAP_DATA_LEN (1024 * 128)
#define KEY_DATA_LEN (1024 * 4)
#define CFG_DATA_LEN (1024 * 4)
#define APP_DATA_LEN (1024 * 256)
#define TPL_DATA_LEN

// flash map
#define IAP_ADDR_START FLASH_ADDR_START                                    // 64K
#define KEY_ADDR_START (FLASH_ADDR_START + IAP_DATA_LEN + SSI_SECTOR_SIZE) // 4K
#define CFG_ADDR_START (KEY_ADDR_START + KEY_DATA_LEN + SSI_SECTOR_SIZE)   // 4K
#define APP_ADDR_START (CFG_ADDR_START + CFG_DATA_LEN + SSI_SECTOR_SIZE)   // 256k
#define TPL_ADDR_START (APP_ADDR_START + APP_DATA_LEN + SSI_SECTOR_SIZE)   // remain

#define ALGO_ENROLLED_ID_START (TPL_ADDR_START)
#define ALGO_FINGER_ID_MAC_START (TPL_ADDR_START + 4 * 1024 + SSI_SECTOR_SIZE)
#define ALGO_TEMPLATE_HEAD_START (ALGO_FINGER_ID_MAC_START + 4 * 1024 + SSI_SECTOR_SIZE)
#define ALGO_TEMPLATE_DATA_START (ALGO_TEMPLATE_HEAD_START + 24 * 1024 + SSI_SECTOR_SIZE)

/* Define the address from where user application will be loaded. */
#define APPLICATION_ADDRESS (uint32_t)APP_ADDR_START
/* End of the Flash address */
#define USER_FLASH_END_ADDRESS (uint32_t)FLASH_ADDR_END
/* Define the user application size */
#define USER_FLASH_SIZE (USER_FLASH_END_ADDRESS - APPLICATION_ADDRESS + 1)

#define STARTUP_ADDR 0x10000000
#define SECT_ERASE_CMD 0x20
#define BLOCK_ERASE_CMD 0x52
#define PAGE_ERASE_CMD 0x81

#define QUAD_PROG_CMD 0x02
#define QUAD_PROGRAM 0x32

typedef struct // FLASH management
{
    uint32_t key_start_addr; // Key storage start address
    uint32_t cfg_start_addr; // Configuration file start address
    uint32_t app_start_addr; // APP start address
    uint32_t tpl_start_addr; // Template storage start address
} flash_map_t;

struct str_flash
{
    uint8_t SsiId; // 1,2,3
    uint8_t SysDiv;
    uint8_t IsQpiMode;
    uint16_t StandBaudr;
    uint16_t QuadBaudr;
    uint32_t RxSampleDelay;

    uint8_t Cmd;    // later
    uint32_t Value; // later

    uint8_t IsMaskInterrupt;
    //------------------------------------
    uint8_t ProgramMode;
    uint32_t Addr; // later
    uint16_t Len;  // later
    uint32_t Buf;  // later
    uint32_t Delay;
    uint32_t Timeout;
};

void DRV_DCACHE_Invalidate(uint32_t addr, uint32_t size);

/**
 * @brief xip FLASH initial
 *
 * @param[in]  NULL
 *
 * @return
 * - NULL
 */
void ft_xip_flash_init();

/**
 * @brief FLASH initial
 *
 * @param[in]  NULL
 *
 * @return
 * - NULL
 */
void flash_init(void);

/**
 * @brief Erase specified FLASH content
 *
 * @param[in]  addr, address of the data to erase.
 * @param[in]  len, length of the data to erase.
 *
 * @return
 * - 0, success;
 * - <0, error.
 */
static int flash_erase_data(uint32_t addr, uint32_t len);

/**
 * @brief Write data to specified on-chip offset address in FLASH.
 *
 * @param[in]  addr, data write address.
 * @param[in]  data, pointer to the data to write.
 * @param[in]  len, length of data to write (must be multiple of 4).
 *
 * @return
 * - 0, success;
 * - <0, error.
 */
static int flash_write_data(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief Read data from specified on-chip offset address in FLASH.
 *
 * @param[in]  offset_addr, read address.
 * @param[out] data, pointer to store read data.
 * @param[in]  len, length to read (must be multiple of 4).
 *
 * @return
 * - 0, success;
 * - <0, error.
 *
 */
static int flash_read_data(uint32_t offset_addr, uint8_t *data, uint32_t len);

/**
 * @brief Erase and set specified FLASH content to specific data
 *
 * @param[in]  id, FLASH ID to operate on.
 * @param[in]  addr, address of data to erase/set.
 * @param[in]  value, specific data value to set.
 * @param[in]  len, length of data to erase/set.
 *
 * @return
 * - 0, success;
 * - <0, error.
 */
 int flash_memset(uint8_t id, uint32_t addr, uint8_t value, uint32_t len);

#endif /* __FLASH_DRV_H__ */