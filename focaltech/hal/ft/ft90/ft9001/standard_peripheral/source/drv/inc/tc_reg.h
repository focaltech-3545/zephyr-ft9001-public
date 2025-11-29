/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#ifndef __TC_REG_H__
#define __TC_REG_H__

#include "type.h"


typedef struct
{
	__IO unsigned short TCCR;//0x0
	__IO unsigned short TCMR;//0x2
	__IO unsigned short TCCNTR;//0x4
	__IO unsigned short TCSR;//0x6

}TC_TypeDef;

#define TC_WAIT				(1<<11)
#define TC_DOZE				(1<<10)
#define TC_STOP		    	(1<<9)
#define TC_DBG				(1<<8)
#define TC_IS				(1<<7)
#define TC_IF				(1<<3)
#define TC_IE				(1<<2)
#define TC_CU				(1<<1)
#define TC_RN				(1<<0)
#define TC2_SEL_MASK                        (0x300)
#define TC3_SEL_MASK                        (0x3000)

#define TC_WDP_SHIFT        (4)
#define TC_SEL_ST_SHIFT					    (6)
#define TC2_SEL_ST_SHIFT					(8)
#define TC3_SEL_ST_SHIFT					(12)

#endif /* __TC_REG_H__ */
