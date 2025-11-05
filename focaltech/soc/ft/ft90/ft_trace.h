#ifndef _FT_TRACE_H_
#define _FT_TRACE_H_

#define FT_TRACE_EN 1

typedef struct ft_trace_st
{
    volatile unsigned int func_addr;
    volatile unsigned int line;
    volatile unsigned int reserved1;
    volatile unsigned int reserved2;
} ft_trace_t;

typedef struct ft_trace_reg_st
{
    volatile unsigned int r0;
    volatile unsigned int r1;
    volatile unsigned int r2;
    volatile unsigned int r3;
    volatile unsigned int lr;
    volatile unsigned int sp;
} ft_trace_reg_t;

#ifdef FT_TRACE_EN
void ft_set_func_info(unsigned int func_addr, unsigned int line);
#define FT_TRACE(addr, line) ft_set_func_info(addr, line)

#else

#define FT_TRACE(addr, line)

#endif

#endif
