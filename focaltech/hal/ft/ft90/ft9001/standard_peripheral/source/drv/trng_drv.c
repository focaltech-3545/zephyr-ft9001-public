/**********************************************************************************
             Copyright(c) 2020 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************/

#include <stdint.h>  
#include "trng_drv.h"

TRNG_HandleTypeDef TRNG_Handler;

static void DRV_TRNG_Cmd(TRNG_TypeDef *ptrng, FunctionalState status)
{
    if(status == ENABLE)
    {
        _trng_start(ptrng);
    }

    else
    {
        _trng_stop(ptrng);
    }
}

static void DRV_TRNG_SetCLKPrescaler(TRNG_TypeDef *ptrng,uint8_t dividor)
{
	_trng_set_clk_prescaler(ptrng,dividor);
}

static void DRV_TRNG_ResetAlalogMode(TRNG_TypeDef *ptrng)
{
	_trng_alalog_model_reset_en(ptrng);
}

static void DRV_TRNG_ConfigAlalogMode(TRNG_TypeDef *ptrng, FunctionalState status)
{
    if(status == ENABLE)
	{
    	_trng_alalog_model_en(ptrng);
	}

	else
	{
		_trng_alalog_model_dis(ptrng);
	}
}

static void DRV_TRNG_ClearFlag_IT(TRNG_TypeDef *ptrng)
{
    _trng_clr_it_flag(ptrng);
}

static uint32_t DRV_GetRandomWord(TRNG_TypeDef *ptrng)
{
	uint32_t random;

    while(_trng_get_it_flag(ptrng) == 0);

    random = ptrng->DR;
    
    _trng_clr_it_flag(ptrng);

    return random;
}

static void HAL_TRNG_Init(TRNG_HandleTypeDef *htrng)
{
	DRV_TRNG_Cmd(htrng->instance,DISABLE);

	DRV_TRNG_SetCLKPrescaler(htrng->instance,htrng->dividor);

	DRV_TRNG_ResetAlalogMode(htrng->instance);

	DRV_TRNG_ConfigAlalogMode(htrng->instance,ENABLE);

	DRV_TRNG_ClearFlag_IT(htrng->instance);

	DRV_TRNG_Cmd(htrng->instance,ENABLE);

}

static void HAL_TRNG_DeInit(TRNG_HandleTypeDef *htrng)
{

	DRV_TRNG_ConfigAlalogMode(htrng->instance,DISABLE);

	DRV_TRNG_Cmd(htrng->instance,DISABLE);
}

static uint32_t HAL_TRNG_GetRandom(TRNG_HandleTypeDef *htrng)
{
	return DRV_GetRandomWord(htrng->instance);
}

void random_init(void)
{
	TRNG_Handler.instance = TRNG;
	TRNG_Handler.dividor = 59;
	HAL_TRNG_Init(&TRNG_Handler);
}

void random_deinit(void)
{
	HAL_TRNG_DeInit(&TRNG_Handler);
}

uint32_t random_get_data(void)
{
	uint32_t random = HAL_TRNG_GetRandom(&TRNG_Handler);
	return random;
}

