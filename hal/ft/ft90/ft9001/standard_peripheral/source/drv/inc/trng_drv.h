#ifndef TRNG_DRV_H_
#define TRNG_DRV_H_

#include "memmap.h"
#include "trng_reg.h"

typedef struct{

    TRNG_TypeDef      *instance;       

    uint8_t            dividor;    

}TRNG_HandleTypeDef;


extern void random_init(void);
	
extern void random_deinit(void);

extern uint32_t random_get_data(void);


#endif
