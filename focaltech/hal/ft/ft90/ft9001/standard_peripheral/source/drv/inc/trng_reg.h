#ifndef __TRNG_REG_H__
#define __TRNG_REG_H__

#include "type.h"
#include "ft_sys.h"

typedef struct
    {
        __IO uint32_t CTRL;          /**< 00*/
        __IO uint32_t DR;            /**< 04*/
        __IO uint32_t TMCTRL;        /**< 08*/
        __IO uint32_t STSCR;         /**< 0C*/
        __IO uint32_t OSCR_Mx[4];    /**< 10 14 18 1C*/
        __IO uint32_t RESERVED0[10]; /**< 20 24 28 2C 30 24 38 3C 40 44*/
        __IO uint32_t SM3DRx[8];     /**< 48 4C 50 54 58 5C 60 64 */
        __IO uint32_t OSCTRIMR1;     /**< 68*/
        __IO uint32_t OSCTRIMR2;     /**< 6C*/
        __IO uint32_t OSCCTR;        /**< 70*/
        __IO uint32_t OSCDIVR;       /**< 74*/
    } TRNG_TypeDef;

#define TRNG ((TRNG_TypeDef *)TRNG_BASE_ADDR)

/*** TRNG **********************************************/
#define TRNG_CLK_SEL (((uint32_t)1 << 31)) /**< TRNG时钟选择     */
#define TRNG_ANA_VALUE_MASK               ((uint32_t)0x8FFFFFFFF))
#define TRNG_ANA_VALUE_SHIFT_MASK ((uint32_t)(28))
#define TRNG_RST_ANA_MASK ((uint32_t)(0xFF0FFFFF))
#define TRNG_RST_ANA_SHIFT_MASK ((uint32_t)(20))
#define TRNG_EN_ANA_MASK ((uint32_t)(0xFFF0FFFF))
#define TRNG_EN_ANA_SHIFT_MASK (((uint32_t)16))
#define TRNG_SM3_EN (((uint32_t)1 << 15))          /**<          */
#define TRNG_SM3_SM3_RD_DONE (((uint32_t)1 << 14)) /**<          */
#define TRNG_IT_MASK (((uint32_t)1 << 11))              /**< TRNG中断标志位   */
#define TRNG_CLR_IT (((uint32_t)1 << 10))          /**< TNRG清除中断检位 */
#define TRNG_EN_IT (((uint32_t)1 << 9))            /**< TRNG中断使能     */
#define TRNG_EN (((uint32_t)1 << 8))               /**< TRNG使能         */

#define TRNG_CLK_DIV_MASK ((uint32_t)0xFFFFFF00)
#define TRNG_CLK_DIV_SHIFT_MASK ((uint32_t)(0))


#define _trng_alalog_model_reset_en(trng)               _reg_modify(trng->CTRL,TRNG_RST_ANA_MASK,(0X0F<<(TRNG_RST_ANA_SHIFT_MASK)))
#define _trng_alalog_model_en(trng)                     _reg_modify(trng->CTRL,TRNG_EN_ANA_MASK,(0X0F<<(TRNG_EN_ANA_SHIFT_MASK)))
#define _trng_alalog_model_dis(trng)                    _reg_modify(trng->CTRL,TRNG_EN_ANA_MASK, 0)
#define _trng_start(trng)                               _bit_set(trng->CTRL, TRNG_EN)                            
#define _trng_stop(trng)                                _bit_clr(trng->CTRL, TRNG_EN) 
#define _trng_set_clk_prescaler(trng,value)             _reg_modify(trng->CTRL,TRNG_CLK_DIV_MASK,((value)<<TRNG_CLK_DIV_SHIFT_MASK))
#define _trng_get_it_flag(trng)                         _reg_chk(trng->CTRL,TRNG_IT_MASK) 		                      
#define _trng_clr_it_flag(trng)                         _bit_set(trng->CTRL,TRNG_CLR_IT)  	


#endif
