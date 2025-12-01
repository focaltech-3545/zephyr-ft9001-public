
#ifndef __CACHE_REG_H__
#define __CACHE_REG_H__
#include "type.h"
/**
* @brief  CACHE
*/

typedef struct
{
    __IO uint32_t CACHE_CCR;                                                        /**< 0x00  Control      */
    __IO uint32_t CACHE_CLCR;                                                       /**< 0x04  Line Control */
    __IO uint32_t CACHE_CSAR;                                                       /**< 0x08  Addr status  */
    __IO uint32_t CACHE_CCVR;                                                       /**< 0x0C  R/W value    */
    __IO uint32_t RESERVED1[4];                                                     /**< 0x10~0x1C */
    __IO uint32_t CACHE_CACR;                                                       /**< 0x20 */
    __IO uint32_t CACHE_CSACR;                                                      /**< 0x24 */
    __IO uint32_t RESERVED2[6];                                                     /**< 0x28~0x3C */

    __IO uint32_t CACHE_CSPI1S0HA;                                                  /**< 0x40 SPI1 */
    __IO uint32_t CACHE_CSPI1S1HA;                                                  /**< 0x44 */
    __IO uint32_t CACHE_CSPI1S2HA;                                                  /**< 0x48 */
    __IO uint32_t CACHE_CSPI1S3HA;                                                  /**< 0x4C */
    __IO uint32_t CACHE_CSPI1S0LA;                                                  /**< 0x50 */
    __IO uint32_t CACHE_CSPI1S1LA;                                                  /**< 0x54 */
    __IO uint32_t CACHE_CSPI1S2LA;                                                  /**< 0x58 */
    __IO uint32_t CACHE_CSPI1S3LA;                                                  /**< 0x5C */

    __IO uint32_t CACHE_CSPI2S0HA;                                                  /**< 0x60 SPI2 */
    __IO uint32_t CACHE_CSPI2S1HA;                                                  /**< 0x64 */
    __IO uint32_t CACHE_CSPI2S2HA;                                                  /**< 0x68 */
    __IO uint32_t CACHE_CSPI2S3HA;                                                  /**< 0x6C */
    __IO uint32_t CACHE_CSPI2S0LA;                                                  /**< 0x70 */
    __IO uint32_t CACHE_CSPI2S1LA;                                                  /**< 0x74 */
    __IO uint32_t CACHE_CSPI2S2LA;                                                  /**< 0x78 */
    __IO uint32_t CACHE_CSPI2S3LA;                                                  /**< 0x7C */

    __IO uint32_t CACHE_CSPI3S0HA;                                                  /**< 0x80 SPI3 */
    __IO uint32_t CACHE_CSPI3S1HA;                                                  /**< 0x84 */
    __IO uint32_t CACHE_CSPI3S2HA;                                                  /**< 0x88 */
    __IO uint32_t CACHE_CSPI3S3HA;                                                  /**< 0x8C */
    __IO uint32_t CACHE_CSPI3S0LA;                                                  /**< 0x90 */
    __IO uint32_t CACHE_CSPI3S1LA;                                                  /**< 0x94 */
    __IO uint32_t CACHE_CSPI3S2LA;                                                  /**< 0x98 */
    __IO uint32_t CACHE_CSPI3S3LA;                                                  /**< 0x9C */
    __IO uint32_t CACHE_CROMRS0HA;                                                  /**< 0xA0 ROM */
    __IO uint32_t CACHE_CROMRS1HA;                                                  /**< 0xA4 */
    __IO uint32_t CACHE_CROMRS2HA;                                                  /**< 0xA8 */
    __IO uint32_t CACHE_CROMRS3HA;                                                  /**< 0xAC */
    __IO uint32_t CACHE_CROMRS0LA;                                                  /**< 0xB0 */
    __IO uint32_t CACHE_CROMRS1LA;                                                  /**< 0xB4 */
    __IO uint32_t CACHE_CROMRS2LA;                                                  /**< 0xB8 */
    __IO uint32_t CACHE_CROMRS3LA;                                                  /**< 0xBC */


    __IO uint32_t RESERVED3[48];                                                    /**< 0xC0~0x17C */

    __IO uint32_t CACHE_CPEA;                                                       /**< 0x180 Page clear address */
    __IO uint32_t CACHE_CPES;                                                       /**< 0x184 Page clear size    */
    __IO uint32_t CACHE_CCG;                                                        /**< 0x188 clock gating */
} CACHE_TypeDef;


#endif  //__CACHE_REG_H__