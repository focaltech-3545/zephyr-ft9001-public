// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : type.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


#ifndef __TYPE_H__
#define __TYPE_H__

#include "ft9001.h"
#include "stdbool.h"

typedef unsigned char  			BOOLEAN;
typedef unsigned char           UINT8;
typedef signed char             INT8;
typedef unsigned short int      UINT16;
typedef signed short int        INT16;
typedef unsigned int            UINT32;
typedef signed int              INT32;
typedef float          		    FP32;
typedef double         		    FP64;

typedef unsigned long           ULONG;
typedef signed long             LONG;

typedef char 			        TCHAR ;
typedef unsigned char 	        TBYTE ;
typedef unsigned int  	        TWORD ;
typedef unsigned long           TDWORD;
typedef unsigned char           BOOL;



#define BIT0 (1<<0)
#define BIT1 (1<<1)
#define BIT2 (1<<2)
#define BIT3 (1<<3)
#define BIT4 (1<<4)
#define BIT5 (1<<5)
#define BIT6 (1<<6)
#define BIT7 (1<<7)
#define BIT8 (1<<8)
#define BIT9 (1<<9)
#define BIT11 (1<<11)
#define BITS(n) (1<<(n))
















//typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif 

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))


/* Bit_SET and Bit_RESET enumeration -----------------------------------------*/
typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;
#define IS_GPIO_BIT_ACTION(ACTION) (((ACTION) == Bit_RESET) || ((ACTION) == Bit_SET))

typedef unsigned long useconds_t;
typedef long suseconds_t;

#define HW_READ_REG32(addr)     (*((volatile uint32_t *)(addr)))
#define HW_READ_REG16(addr)     (*((volatile uint16_t *)(addr)))
#define HW_READ_REG8(addr)      (*((volatile uint8_t *)(addr)))

#define HW_WRITE_REG32(addr,data)     *((volatile uint32_t *)(addr))=data
#define HW_WRITE_REG16(addr, data)    *((volatile uint16_t *)(addr))=data
#define HW_WRITE_REG8(addr, data)     *((volatile uint8_t *)(addr))= data

//#define _U	0x01	/* upper */
//#define _L	0x02	/* lower */
//#define _D	0x04	/* digit */
//#define _C	0x08	/* cntrl */
//#define _P	0x10	/* punct */
//#define _S	0x20	/* white space (space/lf/tab) */
//#define _X	0x40	/* hex digit */
//#define _SP	0x80	/* hard space (0x20) */

//extern unsigned char _ctype[];
//extern char _ctmp;

//#define isalnum(c) ((_ctype+1)[c]&(_U|_L|_D))
//#define isalpha(c) ((_ctype+1)[c]&(_U|_L))
//#define iscntrl(c) ((_ctype+1)[c]&(_C))
//#define isdigit(c) ((_ctype+1)[c]&(_D))
//#define isgraph(c) ((_ctype+1)[c]&(_P|_U|_L|_D))
//#define islower(c) ((_ctype+1)[c]&(_L))
//#define isprint(c) ((_ctype+1)[c]&(_P|_U|_L|_D|_SP))
//#define ispunct(c) ((_ctype+1)[c]&(_P))
//#define isspace(c) ((_ctype+1)[c]&(_S))
//#define isupper(c) ((_ctype+1)[c]&(_U))
//#define isxdigit(c) ((_ctype+1)[c]&(_D|_X))

//#define isascii(c) (((unsigned) c)<=0x7f)
//#define toascii(c) (((unsigned) c)&0x7f)

//#define tolower(c) (_ctmp=c,isupper(_ctmp)?_ctmp-('A'-'a'):_ctmp)
//#define toupper(c) (_ctmp=c,islower(_ctmp)?_ctmp-('a'-'A'):_ctmp)

/* COMPILER SPECIFIC DEFINES (IAR, ARMCC and GNUC) */
#if defined(__GNUC__)
#ifndef __weak
#define __weak __attribute__((weak))
#endif

#elif defined(__CC_ARM)

#define inline __inline
#pragma anon_unions

#endif

#endif   /* __TYPE_H__ */

