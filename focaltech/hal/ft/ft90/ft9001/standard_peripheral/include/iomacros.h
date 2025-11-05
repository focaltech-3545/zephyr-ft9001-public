 /*
*  DESCRIPTION for iomacros.h
*  	Read/Write macros for memory mapped I/O
*/

#ifndef _IOMACROS_H
#define _IOMACROS_H
#include "type.h"
/****************************************************************************
						Constants Definitions
****************************************************************************/
#define IO_READ8(p)            (UINT8)(*(volatile UINT8 *)(p))
#define IO_WRITE8(p,c)         (*((volatile UINT8 *)(p)) = ((UINT8)(c)))

#define IO_READ16(p)           (UINT16)(*(volatile UINT16 *)(p))
#define IO_WRITE16(p,v)        (*((volatile UINT16 *)(p)) = (UINT16)(v))

#define IO_READ32(p)           (UINT32)(*(volatile UINT32 *)(p))
#define IO_WRITE32(p,v)        (*((volatile UINT32 *)(p)) = (UINT32)(v))

//bit operation
#define BSET32(reg, bit)		((reg) |= (bit))
#define BCLR32(reg, bit)		((reg) &= ~(bit))

#endif
