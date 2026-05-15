// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : rtc_reg.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef RTC_REG_H_
#define RTC_REG_H_

#include "type.h"

typedef struct
{
	__IO unsigned int RTC_PRT1R;//0x00
	__IO unsigned int RTC_PRT2R;//0x04
	__IO unsigned int RTC_PRA1R;//0x08
	__IO unsigned int RTC_PRA2R;//0x0c

	__IO unsigned int RTC_PRTCR;//0x10
	__IO unsigned int RTC_PRC1R;//0x14
	__IO unsigned int RTC_PRENR;//0x18
	__IO unsigned int RTC_PRKEYR;//0x1c

}RTC_TypeDef;

#define	AlaD_en		0x00008000
#define	AlaH_en		0x00800000
#define	AlaM_en		0x00008000
#define	AlaS_en		0x00000080

#define	Day_IEN		0x40000000
#define	Hou_IEN		0x20000000
#define	Min_IEN		0x10000000
#define	Sec_IEN		0x08000000
#define	Ala_IEN		0x04000000
#define	CLK_1KHz_IEN	0x02000000
#define	CLK_32KHz_IEN	0x01000000



#define	LOW_LEVEL	0x00000000
#define	RISE_EDGE	0x00000100
#define	FALL_EDGE	0x00000200
#define	BOTH_EDGE	0x00000300

#define Cntupdate_en 0x00000004
#define RCNT_WEN	 0x00000002
#define Dir		     0x00000001

#define WCLK_DIV_MASK   0x000000F8
#define DIV2		 (1<<3)
#define DIV3         (3<<3)
#define DIV8         (8<<3)

#endif /* RTC_REG_H_ */
