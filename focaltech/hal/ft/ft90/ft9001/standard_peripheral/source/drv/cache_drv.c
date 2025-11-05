// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : cache_drv.c
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "cache_drv.h"

/**
 * @brief ICACHE Init
 * @param[in] boot: The attributes of boot memory regions
 *              cacheThrough, cacheBack or cacheOff
 *              rom: The attributes of rom memory regions
 *              cacheThrough, cacheBack or cacheOff
 *              spim1: The attributes of spim1 memory regions
 *              cacheThrough, cacheBack or cacheOff
 *              spim2: The attributes of spim1 memory regions
 *              cacheThrough, cacheBack or cacheOff
 *              spim3: The attributes of spim1 memory regions
 *              cacheThrough, cacheBack or cacheOff
 * @note 
 * @retval NONE
*/
void DRV_ICACHE_Init(CACHE_ComTypeDef boot,
                     CACHE_ComTypeDef rom,
                     CACHE_ComTypeDef spim1,
                     CACHE_ComTypeDef spim2,
                     CACHE_ComTypeDef spim3,
                     uint32_t base)
{
    CACHE_TypeDef *ICACHE = (CACHE_TypeDef *)base;
    //#define ICACHE                              ((CACHE_TypeDef *)(CACHE_BASE_ADDR))
    //#define DCACHE                              ((CACHE_TypeDef *)(CACHE2_BASE_ADDR))
    /*boot cache configuration*/
    if (CACHE_Off == boot)
    {
        ICACHE->CACHE_CSACR &= BOOT_CACHEOFF;
    }
    else if (CACHE_Through == boot)
    {
        ICACHE->CACHE_CSACR &= BOOT_CACHEOFF;
        ICACHE->CACHE_CSACR |= ((uint32_t)WRITE_THROUGH << BOOT_CACHE_SHIFT);
    }
    else if (CACHE_Back == boot)
    {
        ICACHE->CACHE_CSACR &= BOOT_CACHEOFF;
        ICACHE->CACHE_CSACR |= ((uint32_t)WRITE_BACK << BOOT_CACHE_SHIFT);
    }

    /*rom cache configuration*/
    if (CACHE_Off == rom)
    {
        ICACHE->CACHE_CACR &= ROM_CACHEOFF;
    }
    else if (CACHE_Through == rom)
    {
        ICACHE->CACHE_CACR &= ROM_CACHEOFF;
        ICACHE->CACHE_CACR |= ((uint32_t)0x02 << ROM_CACHE_SHIFT);
    }
    else if (CACHE_Back == rom)
    {
        ICACHE->CACHE_CACR &= ROM_CACHEOFF;
        ICACHE->CACHE_CACR |= (0x03 << ROM_CACHE_SHIFT);
    }

    /*spim1 cache configuration*/
    if (CACHE_Off == spim1)
    {
        ICACHE->CACHE_CSACR &= SPIM1_CACHEOFF;
    }
    else if (CACHE_Through == spim1)
    {
        ICACHE->CACHE_CSACR &= SPIM1_CACHEOFF;
        ICACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_THROUGH << SPIM1_CACHE_SHIFT);
    }
    else if (CACHE_Back == spim1)
    {
        ICACHE->CACHE_CSACR &= SPIM1_CACHEOFF;
        ICACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_BACK << SPIM1_CACHE_SHIFT);
    }

    /*spim2 cache configuration*/
    if (CACHE_Off == spim2)
    {
        ICACHE->CACHE_CSACR &= SPIM2_CACHEOFF;
    }
    else if (CACHE_Through == spim2)
    {
        ICACHE->CACHE_CSACR &= SPIM2_CACHEOFF;
        ICACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_THROUGH << SPIM2_CACHE_SHIFT);
    }
    else if (CACHE_Back == spim2)
    {
        ICACHE->CACHE_CSACR &= SPIM2_CACHEOFF;
        ICACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_BACK << SPIM2_CACHE_SHIFT);
    }

    /*spim3 cache configuration*/
    if (CACHE_Off == spim3)
    {
        ICACHE->CACHE_CSACR &= SPIM3_CACHEOFF;
    }
    else if (CACHE_Through == spim3)
    {
        ICACHE->CACHE_CSACR &= SPIM3_CACHEOFF;
        ICACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_THROUGH << SPIM3_CACHE_SHIFT);
    }
    else if (CACHE_Back == spim3)
    {
        ICACHE->CACHE_CSACR &= SPIM3_CACHEOFF;
        ICACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_BACK << SPIM3_CACHE_SHIFT);
    }

    ICACHE->CACHE_CCR |= (GO | INVW1 | INVW0);
    while (((ICACHE->CACHE_CCR) & (GO)) == GO)
        ;
    ICACHE->CACHE_CCR |= ENCACHE;
}

/**
 * @brief DCACHE Init
 * @param[in] boot: The attributes of boot memory regions
 *              cacheThrough, cacheBack or cacheOff
 *              rom: The attributes of rom memory regions
 *              cacheThrough, cacheBack or cacheOff
 *              spim1: The attributes of spim1 memory regions
 *              cacheThrough, cacheBack or cacheOff
 *              spim2: The attributes of spim1 memory regions
 *              cacheThrough, cacheBack or cacheOff
 *              spim3: The attributes of spim1 memory regions
 *              cacheThrough, cacheBack or cacheOff
 * @note 
 * @retval NONE
*/
void DRV_DCACHE_Init(CACHE_ComTypeDef boot,
                     CACHE_ComTypeDef rom,
                     CACHE_ComTypeDef spim1,
                     CACHE_ComTypeDef spim2,
                     CACHE_ComTypeDef spim3,
                     uint32_t base)
{
    CACHE_TypeDef *DCACHE = (CACHE_TypeDef *)base;
    /*boot cache configuration*/
    if (CACHE_Off == boot)
    {
        DCACHE->CACHE_CSACR &= BOOT_CACHEOFF;
    }
    else if (CACHE_Through == boot)
    {
        DCACHE->CACHE_CSACR &= BOOT_CACHEOFF;
        DCACHE->CACHE_CSACR |= ((uint32_t)WRITE_THROUGH << BOOT_CACHE_SHIFT);
    }
    else if (CACHE_Back == boot)
    {
        DCACHE->CACHE_CSACR &= BOOT_CACHEOFF;
        DCACHE->CACHE_CSACR |= ((uint32_t)WRITE_BACK << BOOT_CACHE_SHIFT);
    }

    /*rom cache configuration*/
    if (CACHE_Off == rom)
    {
        DCACHE->CACHE_CACR &= ROM_CACHEOFF;
    }
    else if (CACHE_Through == rom)
    {
        DCACHE->CACHE_CACR &= ROM_CACHEOFF;
        DCACHE->CACHE_CACR |= (0x02 << ROM_CACHE_SHIFT);
    }
    else if (CACHE_Back == rom)
    {
        DCACHE->CACHE_CACR &= ROM_CACHEOFF;
        DCACHE->CACHE_CACR |= (0x03 << ROM_CACHE_SHIFT);
    }

    /*spim1 cache configuration*/
    if (CACHE_Off == spim1)
    {
        DCACHE->CACHE_CSACR &= SPIM1_CACHEOFF;
    }
    else if (CACHE_Through == spim1)
    {
        DCACHE->CACHE_CSACR &= SPIM1_CACHEOFF;
        DCACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_THROUGH << SPIM1_CACHE_SHIFT);
    }
    else if (CACHE_Back == spim1)
    {
        DCACHE->CACHE_CSACR &= SPIM1_CACHEOFF;
        DCACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_BACK << SPIM1_CACHE_SHIFT);
    }

    /*spim2 cache configuration*/
    if (CACHE_Off == spim2)
    {
        DCACHE->CACHE_CSACR &= SPIM2_CACHEOFF;
    }
    else if (CACHE_Through == spim2)
    {
        DCACHE->CACHE_CSACR &= SPIM2_CACHEOFF;
        DCACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_THROUGH << SPIM2_CACHE_SHIFT);
    }
    else if (CACHE_Back == spim2)
    {
        DCACHE->CACHE_CSACR &= SPIM2_CACHEOFF;
        DCACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_BACK << SPIM2_CACHE_SHIFT);
    }

    /*spim3 cache configuration*/
    if (CACHE_Off == spim3)
    {
        DCACHE->CACHE_CSACR &= SPIM3_CACHEOFF;
    }
    else if (CACHE_Through == spim3)
    {
        DCACHE->CACHE_CSACR &= SPIM3_CACHEOFF;
        DCACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_THROUGH << SPIM3_CACHE_SHIFT);
    }
    else if (CACHE_Back == spim3)
    {
        DCACHE->CACHE_CSACR &= SPIM3_CACHEOFF;
        DCACHE->CACHE_CSACR |= ((uint32_t)SPIM_WRITE_BACK << SPIM3_CACHE_SHIFT);
    }

    DCACHE->CACHE_CCR |= (GO | INVW1 | INVW0);
    while (((DCACHE->CACHE_CCR) & (GO)) == GO)
        ;
    DCACHE->CACHE_CCR |= ENCACHE;
}

/**
 * @brief ICACHE Invalidate
 * @param[in] addr: The base addr of memory need to invalidate,
 *                  no need 16-bytes aligned.
 * @param[in] size: The length of memory need to invalidate,
 *                  no need 16-bytes aligned.
 * @note 
 * @retval NONE
*/
void DRV_ICACHE_Invalidate(uint32_t addr, uint32_t size, uint32_t base)
{
    CACHE_TypeDef *ICACHE = (CACHE_TypeDef *)base;
    if (ICACHE->CACHE_CCR & ENCACHE) 
    {
        size = (((addr & 0x0f) + size) + 0x0f) & ~0x0f;
        addr = addr & ~0x0f;

        ICACHE->CACHE_CPEA = addr;
        ICACHE->CACHE_CPES = size | PAGE_CACHE_CLEAN_GO;
        while (ICACHE->CACHE_CPES & PAGE_CACHE_CLEAN_GO)
            ;
    }
}

/**
 * @brief DCACHE Invalidate
 * @param[in] addr: The base addr of memory need to invalidate,
 *                  no need 16-bytes aligned.
 * @param[in] size: The length of memory need to invalidate,
 *                  no need 16-bytes aligned.
 * @note 
 * @retval NONE
*/
void DRV_DCACHE_Invalidate(uint32_t addr, uint32_t size, uint32_t base)
{
    CACHE_TypeDef *DCACHE = (CACHE_TypeDef *)base;
    if (DCACHE->CACHE_CCR & ENCACHE) 
    {
        size = (((addr & 0x0f) + size) + 0x0f) & ~0x0f;
        addr = addr & ~0x0f;
        DCACHE->CACHE_CPEA = addr;
        DCACHE->CACHE_CPES = size | PAGE_CACHE_CLEAN_GO;
        while (DCACHE->CACHE_CPES & PAGE_CACHE_CLEAN_GO)
            ;
    }
}
