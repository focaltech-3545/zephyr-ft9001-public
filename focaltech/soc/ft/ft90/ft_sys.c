/*
 * Copyright (c) 2021 Focaltech Systems CO.,Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/linker/linker-defs.h>

#include "ft_sys.h"

const struct str_flash g_ssi_cfg[]={
    {
	   .SsiId=1,
	   .Cmd = 2,       // 0x00000020;      // erase

        .SysDiv = 2,
        .IsQpiMode = 0,
        .StandBaudr = 0x0002,
        .QuadBaudr = 0x0002,
        .RxSampleDelay = 0x00000001,

        .IsMaskInterrupt = 1,
        .ProgramMode = 0, // dma_mode or not
        .Len = 0,        // program len
        .Buf = 0,        // program buf addr
        .Delay = 10,
        .Timeout = 0xffffffff,
    },
    {
       .SsiId=2,
       .Cmd = 2,       // 0x00000020;      // erase

        .SysDiv = 2,
        .IsQpiMode = 0,
        .StandBaudr = 0x0002,
        .QuadBaudr = 0x0002,
        .RxSampleDelay = 0x00000001,

        .IsMaskInterrupt = 1,
        .ProgramMode = 0, // dma_mode or not
        .Len = 0,        // program len
        .Buf = 0,       // program buf addr
        .Delay = 10,
        .Timeout = 0xffffffff,
    },
    {
       .SsiId=3,
       .Cmd = 2,       // 0x00000020;      // erase

        .SysDiv = 2,
        .IsQpiMode = 0,
        .StandBaudr = 0x0002,
        .QuadBaudr = 0x0002,
        .RxSampleDelay = 0x00000001,

        .IsMaskInterrupt = 1,
        .ProgramMode = 0, // dma_mode or not
        .Len = 0,        // program len
        .Buf = 0,        // program buf addr
        .Delay = 10,
        .Timeout = 0xffffffff,
    },
    
};

static uint32_t g_ips_clk;
static uint32_t g_sys_clk;

static uint32_t g_trim_clk;

uint8_t (*xip_flash_erase)(struct str_flash *p_ssi_para);
uint8_t (*xip_flash_program)(struct str_flash *p_ssi_para);
static uint8_t (*xip_sys_clk_switch)(struct str_flash *p_ssi_para);
uint8_t (*ssi_open_xip)(struct str_flash *p_ssi_para);
uint8_t (*ssi_close_xip)(struct str_flash *p_ssi_para);

/* Calls a ROM function. This function must be called to initialize before any flash operation. */
void ft_xip_flash_init()
{
   
    xip_flash_erase = (uint8_t (*)(struct str_flash *))(((uint32_t *)(0x0400747a | 1)));
    xip_flash_program = (uint8_t (*)(struct str_flash *))(((uint32_t *)(0x0400752e | 1)));
    xip_sys_clk_switch = (uint8_t (*)(struct str_flash *))(((uint32_t *)(0x04007658 | 1)));
	ssi_open_xip = (uint8_t (*)(struct str_flash *))(((uint32_t *)(0x040069d4 | 1)));
	ssi_close_xip = (uint8_t (*)(struct str_flash *))(((uint32_t *)(0x0400665c | 1)));

}

static void DRV_CPM_VccCoreTestModeKeySet(void)
{
    uint32_t tmp = (_cpm_get_core_test_mode_value & 0x3FFFFFFF);

    _cpm_write_core_test_mode_value(tmp);
    _cpm_write_core_test_mode_value(tmp | 0x40000000);
    _cpm_write_core_test_mode_value(tmp | 0x80000000);
    _cpm_write_core_test_mode_value(tmp | 0xC0000000);
}
static void DRV_CPM_SystemClkVrefTrim(uint32_t trim_value)
{
    DRV_CPM_VccCoreTestModeKeySet();

    _cpm_set_overwrite_vccgtrimr_trim_en;

    if (trim_value == CPM_VREF_TRIM_090)
    {
        _cpm_set_core_voltage_0V9_on;
    }
    else
    {
        _cpm_set_core_voltage_0V9_off;
        _cpm_set_ldo_trim_value(trim_value);
    }
}

static void DRV_XIP_SYSCLKSwitch(uint8_t sys_clk_div, uint16_t ssi_clk_div, uint32_t ssi_rx_sample_delay)
{
    uint8_t ssi_id = 1; /* SSI1 */
    struct str_flash ssi_cfg;
    memcpy(&ssi_cfg,&g_ssi_cfg[ssi_id - 1],sizeof( struct str_flash));
 
    
    ssi_cfg.SsiId = ssi_id;
    ssi_cfg.SysDiv = sys_clk_div;
    ssi_cfg.QuadBaudr = ssi_clk_div;
    ssi_cfg.RxSampleDelay = ssi_rx_sample_delay;

    xip_sys_clk_switch(&ssi_cfg);
}

static void DRV_CPM_SystemClkOSC320MSelect(void)
{
    _cpm_set_osc320m_clk_en;
    while (!_cpm_get_osc320m_stable_flag)
        ;

    _cpm_set_soc_clk_osc320m_en;
    while (!_cpm_get_osc320m_select_flag)
        ;
}

static void DRV_CPM_SetIpsClkDiv(uint8_t div)
{
    _cpm_set_ips_clk_div_en;
    _cpm_set_peripheral_ips_clk_div(div);
    _cpm_update_peripheral_clk_div;
}

static uint32_t DRV_CPM_GetSYSClHz(void)
{
    uint32_t clk_freq;

    if (_cpm_chk_sys_clk_src(CPM_SYSCLK_OSC8M))
    {
        clk_freq = 8 * 1000 * 1000;
    }
    else if (_cpm_chk_sys_clk_src(CPM_SYSCLK_OSC320M))
    {
        if (g_trim_clk == OSC_320M_HZ)
        {
            clk_freq = DRV_SYS_OSC_CLK_320M;
        }
        else if (g_trim_clk == OSC_360M_HZ)
        {
            clk_freq = DRV_SYS_OSC_CLK_360M;
        }
        else if (g_trim_clk == OSC_400M_HZ)
        {
            clk_freq = DRV_SYS_OSC_CLK_400M;
        }
        else
        {
            clk_freq = DRV_SYS_OSC_CLK_320M;
        }
    }
    else if (_cpm_chk_sys_clk_src(CPM_SYSCLK_USBPHY240M))
    {
        clk_freq = DRV_SYS_OSC_CLK_240M;
    }
    else if (_cpm_chk_sys_clk_src(CPM_SYSCLK_OSCEXT) == CPM_SYSCLK_OSCEXT)
    {
        clk_freq = DRV_SYS_OSC_CLK_12M;
    }
    else
    {
        clk_freq = 0;
    }

    return ((uint32_t)(clk_freq / ((_cpm_get_sys_clk_value & 0xff) + 1)));
}

static uint32_t DRV_CPM_GetIPSClHz(void)
{
    uint32_t ips_clk;

    if (_cpm_chk_ips_clk_div_en)
    {
        ips_clk = g_sys_clk / ((_cpm_get_peripheral1_clk_div_value & 0x0F) + 1);
    }
    else
    {
        ips_clk = g_sys_clk;
    }

    return ips_clk;
}

static void ft_Sys_SysClkConfig(SYS_ClkInitTypeDef *pClkInit)
{

    // Trim clock source
    LIB_CPM_OscSwitch(pClkInit->SysClkTrim);

    // if ((OSC_320M_HZ < pClkInit->SysClkTrim) && (CLK_DIV_2 == pClkInit->SysClkDiv))
    {
        DRV_CPM_SystemClkVrefTrim(CPM_VREF_TRIM_121);
    }

    // Configure system clock

    if (CPM_SYSCLK_OSC320M == pClkInit->SysClkSource)
    {
        DRV_CPM_SystemClkOSC320MSelect();
    }

    //  DRV_CPM_SetSystemClkDiv(pClkInit->SysClkDiv);
    DRV_XIP_SYSCLKSwitch((pClkInit->SysClkDiv + 1), 2, 1);

    // Configure IPS clock (divider coefficient cannot be 0)
    if (CLK_DIV_1 == pClkInit->IpsClkDiv)
    {
        DRV_CPM_SetIpsClkDiv(pClkInit->IpsClkDiv + 1);
    }
    else
    {
        DRV_CPM_SetIpsClkDiv(pClkInit->IpsClkDiv);
    }

    // Get system clock
    g_sys_clk = DRV_CPM_GetSYSClHz();

    // Get IPS clock
    g_ips_clk = DRV_CPM_GetIPSClHz();
#if 0
    // Get AHB3 clock
    g_ahb3_clk = DRV_CPM_GetAHB3ClHz();

    // When system clock ≤60MHz, trim voltage to 0.9V to reduce power
    if (g_sys_clk <= 60000000)
    {
        DRV_CPM_SystemClkVrefTrim(CPM_VREF_TRIM_090);
    }
#endif
}

static void DRV_DCACHE_Init(CACHE_ComTypeDef boot, CACHE_ComTypeDef rom, CACHE_ComTypeDef spim1, CACHE_ComTypeDef spim2,
                            CACHE_ComTypeDef spim3)
{
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

static void DRV_ICACHE_Init(CACHE_ComTypeDef boot, CACHE_ComTypeDef rom, CACHE_ComTypeDef spim1, CACHE_ComTypeDef spim2,
                            CACHE_ComTypeDef spim3)
{
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

void DRV_PSRAM_OPENXIP( uint32_t ssi_rx_sample_delay)
{
    struct str_flash  ssiconfig;
    ssiconfig.SsiId = 6;
    ssiconfig.StandBaudr =2;
    ssiconfig.QuadBaudr = 2;

    ssiconfig.RxSampleDelay =  ssi_rx_sample_delay ;
    ssiconfig.Cmd = 0x35 ;
    ssiconfig.IsMaskInterrupt =0;
    ssiconfig.Delay =0;
    
    ssi_open_xip(&ssiconfig);
}

void DRV_PSRAM_CloseXIP(void)
{
    struct str_flash  ssiconfig;
    ssiconfig.SsiId = 6;
    ssiconfig.StandBaudr =2;
    ssiconfig.QuadBaudr = 2;

    ssiconfig.RxSampleDelay =  0 ;
    ssiconfig.Cmd = 0x35 ;
    ssiconfig.IsMaskInterrupt =0;
    ssiconfig.Delay =0;

    ssi_close_xip(&ssiconfig);
}
void DRV_DCACHE_Push(uint32_t way)
{
    if (DCACHE->CACHE_CCR & ENCACHE) /* 只有当cache使能时，才执行cache push操作 */
    {
        DCACHE->CACHE_CCR |= (way | GO);
        /* 等待cache清除完成 */
        while (DCACHE->CACHE_CCR & GO)
            ;
    }
}
void HAL_SSI_PSRAMOpenXIP(uint32_t ssi_rx_sample_delay)
{
    DRV_PSRAM_OPENXIP(ssi_rx_sample_delay);
}

void HAL_SSI_PSRAMCloseXIP(void)
{
    DRV_PSRAM_CloseXIP();
}

#if 1
static void ft_Sys_CacheInit(void)
{
    DRV_DCACHE_Init(CACHE_Through, CACHE_Through, CACHE_Through, CACHE_Through, CACHE_Back);

    DRV_ICACHE_Init(CACHE_Through, CACHE_Through, CACHE_Through, CACHE_Through, CACHE_Back);
	 HAL_SSI_PSRAMCloseXIP();
    HAL_SSI_PSRAMOpenXIP(1);
}
#endif

static void ft_delay(uint32_t cnt)
{

    while (cnt--)
    {
        // loop=1000;
        // while(loop--){

        //}
    }
}



   typedef enum
    {
        SWAP_DIS = 0,
        SWAP_EN,

    } IOCTRL_UARTSwapGintENTypeDef;

 typedef struct
    {
        uint8_t pins;
        IOCTRL_UARTSwapGintENTypeDef swap;
    } IOCTRL_UARTSwapTypeDef;

    typedef struct
    {
        volatile uint32_t SPICR;     /**< 0x00  */
        volatile uint32_t USICR;     /**< 0x04  */
        volatile uint32_t I2CCR;     /**< 0x08  */
        volatile uint32_t UARTCR;    /**< 0x0C  */
        volatile uint32_t GINTLCR;   /**< 0x10  */
        volatile uint32_t GINTHCR;   /**< 0x14  */
        volatile uint32_t RESERVED;  /**< 0x18  */
        volatile uint32_t SWAPCR;    /**< 0x1C  */
        volatile uint32_t SPIM1CR;   /**< 0x20  */
        volatile uint32_t SPIM2CR;   /**< 0x24  */
        volatile uint32_t SPIM3CR;   /**< 0x28  */
        volatile uint32_t RESERVED2; /**< 0x2C  */
        volatile uint32_t RESERVED3; /**< 0x30  */
        volatile uint32_t RESERVED4; /**< 0x34  */
        volatile uint32_t WKUPPADCR; /**< 0x38  */
        volatile uint32_t RESERVED5; /**< 0x3C  */
        volatile uint32_t RESERVED6; /**< 0x40  */
        volatile uint32_t RESERVED7; /**< 0x44  */
        volatile uint32_t PSRAMCR1;  /**< 0x48  */
        volatile uint32_t PSRAMCR2;  /**< 0x4C  */
        volatile uint32_t PSRAMCR3;  /**< 0x50  */
        volatile uint32_t EPORT2CR;  /**< 0x54  */
        volatile uint32_t EPORT3CR;  /**< 0x58  */
        volatile uint32_t EPORT4CR;  /**< 0x5C  */
        volatile uint32_t EPORT5CR;  /**< 0x60  */
        volatile uint32_t EPORT6CR;  /**< 0x64  */
        volatile uint32_t EPORT7CR;  /**< 0x68  */
        volatile uint32_t SWAPCR2;   /**< 0x6C  */
        volatile uint32_t SWAPCR3;   /**< 0x70  */
        volatile uint32_t SWAPCR4;   /**< 0x74  */
        volatile uint32_t SWAPCR5;   /**< 0x78  */
        volatile uint32_t I2SIOCR;   /**< 0x7C  */
        volatile uint32_t SSISLVCR;  /**< 0x80  */
        volatile uint32_t PWMTCR;    /**< 0x84  */
        volatile uint32_t CANCR;     /**< 0x88  */
        volatile uint32_t SPI1CR;    /**< 0x8C  */
        volatile uint32_t SPI2CR;    /**< 0x90  */
        volatile uint32_t SPI3CR;    /**< 0x94  */
    } IOCTRL_TypeDef;
    
#define IOCTRL_UARTCR_GINT_SWAP (24)                 /**<  */
#define IOCTRL_BASE_ADDR (0x40000000)   /**< IOCTRL寄存器基地址 */
#define IOCTRL ((IOCTRL_TypeDef *)IOCTRL_BASE_ADDR) 
#define _ioctrl_uart_swap_dis(bits) _bit_clr(IOCTRL->UARTCR, bits << IOCTRL_UARTCR_GINT_SWAP)
#define _ioctrl_uart_swap_en(bits) _bit_set(IOCTRL->UARTCR, bits << IOCTRL_UARTCR_GINT_SWAP)
 typedef enum
    {
        TX1_GINT4 = 0x10,
        RX1_GINT0 = 0x01,
        TX2_GINT5 = 0x20,
        RX2_GINT3 = 0x08,
        TX3_GINT1 = 0x02,
        RX3_GINT2 = 0x04,
    } IOCTRL_UARTSwapGintTypeDef;        
void DRV_IOCTRL_UARTSwapGint(uint8_t pins, IOCTRL_UARTSwapGintENTypeDef swap)
{
    if (SWAP_EN == swap)
    {
        _ioctrl_uart_swap_en(pins);
    }
    else
    {
        _ioctrl_uart_swap_dis(pins);
    }
}
        
int HAL_IOCTRL_UARTSwapGint(IOCTRL_UARTSwapTypeDef *pUARTSwap)
{
    DRV_IOCTRL_UARTSwapGint(pUARTSwap->pins, pUARTSwap->swap);

    return 0;
}

static int uart_init(uint8_t port_id, uint32_t baud_rate)
{
    uint32_t tmp_rate = 0;
    if (port_id == 2)
    {
        // TODO
        
        static uint8_t ucIsUart2Inited = 0;
        if (ucIsUart2Inited)
        {
            // return FF_SUCCESS;
        }

        // UART initialization settings
        tmp_rate = (g_ips_clk * 4 / baud_rate) >> 6;
        UART2->BRDF = (((g_ips_clk * 8 / baud_rate) + 1) / 2) & 0x003f;
        UART2->BRDF = (((g_ips_clk * 8 / baud_rate) + 1) / 2) & 0x003f;

        UART2->BDH = (uint8_t)((tmp_rate >> 8) & 0x00ff);
        UART2->BDL = (uint8_t)(tmp_rate & 0x00ff);

        UART2->CR2 = 0x00;
        UART2->CR1 = 0x00;
        UART2->CR2 |= (UART_RE | UART_CR2_TE_MASK);

        UART2->PURD |= 0x81;

        ucIsUart2Inited = 1;
    }else if (port_id == 3){
        IOCTRL_UARTSwapTypeDef uart_gint_Swap;    //默认GINT与UART引脚不交换   
        uart_gint_Swap.pins = TX3_GINT1 | RX3_GINT2;   
        uart_gint_Swap.swap = SWAP_EN;  
        HAL_IOCTRL_UARTSwapGint(&uart_gint_Swap);

        tmp_rate = (g_ips_clk * 4 / baud_rate) >> 6;
        UART3->BRDF = (((g_ips_clk * 8 / baud_rate) + 1) / 2) & 0x003f;
        UART3->BRDF = (((g_ips_clk * 8 / baud_rate) + 1) / 2) & 0x003f;
    
        UART3->BDH = (uint8_t)((tmp_rate >> 8) & 0x00ff);
        UART3->BDL = (uint8_t)(tmp_rate & 0x00ff);
        
        UART3->CR2 = 0x00;
        UART3->CR1 = 0x00;
        UART3->CR2 |= (UART_RE|UART_CR2_TE_MASK);


    }

    return 0;
}

void close_wdt()
{

    typedef struct
    {
        __IO unsigned short WDT_WCR; // 0x00
        __IO unsigned short WDT_WMR; // 0x02

        __IO unsigned short WDT_WCNTR; // 0x04
        __IO unsigned short WDT_WSR;   // 0x06

    } WDT_TypeDef;
#define WDT_BASE_ADDR (0x40005000)
#define WDT ((WDT_TypeDef *)WDT_BASE_ADDR)
#define WDT_EN 0x01
    WDT->WDT_WCR &= ~WDT_EN;
}

typedef struct
{
    __IO uint32_t RCR;  // 0x00
    __IO uint8_t LVDCR; // 0x04
    __IO uint8_t HVDCR; // 0x05
    __IO uint8_t RTR;   // 0x06
    __IO uint8_t RSR;   // 0x07
} RESET_TypeDef;

#define RESET_BASE_ADDR (0x40002000)

#define RST ((RESET_TypeDef *)(RESET_BASE_ADDR))
#define RESET_RCR_SOFTRST (((uint32_t)1U << 31))

// #define _bit_set(value, bit)    ((value) |=  (bit))
#define _reset_softreset _bit_set(RST->RCR, RESET_RCR_SOFTRST)

void HAL_RESET_SoftReset()
{
    _reset_softreset;
}

void ft_DRV_DCACHE_Invalidate(uint32_t addr, uint32_t size)
{
    uint8_t bRestartup = 0;
#define STARTUP_ADDR 0x10000000

    if (addr == STARTUP_ADDR)
    {
        bRestartup = 0x55;
    }

    if (DCACHE->CACHE_CCR & ENCACHE) /* 只有当cache使能时，才执行cache清除操作 */
    {
        /* 参数16字节对齐处理 */
        size = (((addr & 0x0f) + size) + 0x0f) & ~0x0f;
        addr = addr & ~0x0f;
        /* 写入cache清除页地址 */
        DCACHE->CACHE_CPEA = addr;
        /* 写入cache清除页大小 */
        DCACHE->CACHE_CPES = size | PAGE_CACHE_CLEAN_GO;
        /* 等待cache清除完成 */
        while (DCACHE->CACHE_CPES & PAGE_CACHE_CLEAN_GO)
        {
            if (bRestartup == 0x55)
            {
                bRestartup = 0;
                HAL_RESET_SoftReset();
            }
        }
    }
}

typedef struct
{
    __IO unsigned short TCCR;   // 0x0
    __IO unsigned short TCMR;   // 0x2
    __IO unsigned short TCCNTR; // 0x4
    __IO unsigned short TCSR;   // 0x6

} TC_TypeDef;

void Timer_Rst_Disable(TC_TypeDef *tc)
{

    tc->TCCR &= ~TC_RN;
}
void ft_close_tc_reset()
{

    Timer_Rst_Disable((TC_TypeDef *)TC_BASE_ADDR);
}

void ft_Sys_Init(void)
{
    SYS_ClkInitTypeDef clk_init;

#if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | /* set CP10 Full Access */
                   (3UL << 11 * 2)); /* set CP11 Full Access */
#endif

#ifdef UNALIGNED_SUPPORT_DISABLE
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

    /* Recommended to enable debug clock to prevent simulation failure due to clock modification errors */
    ft_delay(50);

    ft_xip_flash_init();

    /* System clock source options: internal 320M, internal 8M, internal 240M, external 12M. Internal 320M recommended
     */
    clk_init.SysClkSource = CPM_SYSCLK_OSC320M;
    /* System clock source can be trimmed to 320M, 360M, 400M */
    clk_init.SysClkTrim = OSC_320M_HZ; // OSC_400M_HZ;//OSC_320M_HZ;
    /* Min 2-division, max 256-division */
    clk_init.SysClkDiv = CLK_DIV_2;
    /* Min 2-division, max 16-division */
    clk_init.IpsClkDiv = CLK_DIV_2;
    /*   */
    g_trim_clk = clk_init.SysClkTrim;

    /* Initialize clock */
    ft_Sys_SysClkConfig(&clk_init);

    /* Initialize CACHE */
    ft_Sys_CacheInit();

    uart_init(2, 115200);

    close_wdt();

    ft_close_tc_reset();
}
void ft_sys_wake_up(void)
{

    SYS_ClkInitTypeDef clk_init;

    /* Recommended to enable debug clock to prevent simulation failure due to clock modification errors */

    /* System clock source options: internal 320M, internal 8M, internal 240M, external 12M. Internal 320M recommended
     */
    clk_init.SysClkSource = CPM_SYSCLK_OSC320M;
    /* System clock source can be trimmed to 320M, 360M, 400M */
    clk_init.SysClkTrim = OSC_320M_HZ; // OSC_400M_HZ;//OSC_320M_HZ;
    /* Min 2-division, max 256-division */
    clk_init.SysClkDiv = CLK_DIV_2;
    /* Min 2-division, max 16-division */
    clk_init.IpsClkDiv = CLK_DIV_2;
    /*   */
    g_trim_clk = clk_init.SysClkTrim;

    /* Initialize clock */
    ft_Sys_SysClkConfig(&clk_init);
}
#define _reset_get_status _reg_read(RST->RSR)
uint8_t Get_RESET_GetStatus(void)
{
    uint8_t temp;
    temp = _reset_get_status;
    return temp;
}




