// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : wdt_reg.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef __WDT_REG_H__
#define __WDT_REG_H__

#include "type.h"

typedef struct
{
	__IO unsigned short WDT_WCR;//0x00
	__IO unsigned short WDT_WMR;//0x02

	__IO unsigned short WDT_WCNTR;//0x04
	__IO unsigned short WDT_WSR;//0x06

}WDT_TypeDef;

#define WDT_EN    0x01
#define WDT_DBG   0x02
#define WDT_DOZE  0x04
#define WDT_WAIT  0x08

#endif /* __WDT_REG_H__ */
