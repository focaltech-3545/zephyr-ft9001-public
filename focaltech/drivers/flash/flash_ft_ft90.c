#define DT_DRV_COMPAT ft_ft90_flash_controller

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>


#include <ft_trace.h>
#include "flash_driver.h"


//#define BIT(x) ((1<<x))
#define FT_FLASH_EX_OP_SET_WP BIT(0)
#define FT_FLASH_EX_OP_GET_WP BIT(1)
#define FT_FLASH_EX_OP_CLR_WP BIT(2)

#define FT_FLASH_PROTECT_STATUS_UNSET 0

#define FT_FLASH_FT90_EX_OP_WR_SR  BIT(15)
#define FT_FLASH_FT90_EX_OP_RD_SR  BIT(14)

#define FT_FLASH_BASE_ADDR DT_REG_ADDR(DT_CHOSEN(zephyr_flash))
#define FT_FLASH_MAX_SIZE DT_REG_SIZE(DT_CHOSEN(zephyr_flash))
#define FT_FLASH_SIZE_PER_ID 0x4000000


typedef struct
{
        __IO uint32_t CTRLR0;         //0x00
        __IO uint32_t CTRLR1;         //0x04
        __IO uint32_t SSIENR;         //0x08
        __IO uint32_t MWCR;           //0x0c
        __IO uint32_t SER;            //0x10
        __IO uint32_t BAUDR;          //0x14
        __IO uint32_t TXFTLR;         //0x18
        __IO uint32_t RXFTLR;         //0x1c
        __IO uint32_t TXFLR;          //0x20
        __IO uint32_t RXFLR;          //0x24
        __IO uint32_t SR;             //0x28
        __IO uint32_t IMR;            //0x2c
        __IO uint32_t ISR;            //0x30
        __IO uint32_t RISR;           //0x34
        __IO uint32_t TXOICR;         //0x38
        __IO uint32_t RXOICR;         //0x3c
        __IO uint32_t RXUICR;         //0x40
        __IO uint32_t reserve_44;     //0x44
        __IO uint32_t ICR;            //0x48
        __IO uint32_t DMACR;          //0x4c
        __IO uint32_t DMATDLR;        //0x50
        __IO uint32_t DMARDLR;        //0x54
        __IO uint32_t reserve_58;     //0x58
        __IO uint32_t reserve_5c;     //0x5c
        __IO uint32_t DR;             //0x60
        __IO uint32_t RESERVERED[35]; //0x64~0xec
        __IO uint32_t RXSDR;          //0xf0
        __IO uint32_t SPICTRLR0;      //0xf4
        __IO uint32_t reserve_f8;     //0xf8
        __IO uint32_t XIPMBR;         //0xfc
        __IO uint32_t XIPIIR;         //0x100
        __IO uint32_t XIPWIR;         //0x104
        __IO uint32_t XIPCR;          //0x108
        __IO uint32_t XIPSER;         //0x10C
        __IO uint32_t XRXIOCR;        //0x110
        __IO uint32_t reserve_114;    //0x114
} SSI_TypeDef;



extern struct str_flash ssi_cfg[3];
extern uint8_t (*xip_flash_erase)(struct str_flash *p_ssi_para);
extern uint8_t (*xip_flash_program)(struct str_flash *p_ssi_para);
 void ft_DRV_DCACHE_Invalidate(uint32_t addr, uint32_t size); 
static int flash_ft90_erase(const struct device *dev, off_t addr, size_t size);

//LOG_MODULE_REGISTER(flash_ft90_flash, CONFIG_FLASH_LOG_LEVEL);

static const struct flash_parameters flash_ft90_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};


static uint32_t drv_ssi_get_ssi_id(uint32_t addr)
{
    uint32_t ssi_id = 0;

    if ((addr >= FT_FLASH_BASE_ADDR) && (addr < FT_FLASH_BASE_ADDR+FT_FLASH_SIZE_PER_ID))
    {
        ssi_id = 1; /*SSI1*/
    }
    else if ((addr >= FT_FLASH_BASE_ADDR+FT_FLASH_SIZE_PER_ID) && (addr < FT_FLASH_BASE_ADDR+FT_FLASH_SIZE_PER_ID*2))
    {
        ssi_id = 2; /*SSI2*/
    }
    else if ((addr >= FT_FLASH_BASE_ADDR+FT_FLASH_SIZE_PER_ID*2) && (addr < FT_FLASH_BASE_ADDR+FT_FLASH_SIZE_PER_ID*3))
    {
        ssi_id = 3; /*SSI3*/
    }

    return ssi_id;
}

#if defined(CONFIG_FLASH_EX_OP_ENABLED)

#define CCM_SSICFG (*(volatile uint32_t *)(0x40001000 + 0x3c))

#define FT_NOP()				\
	do {	  __builtin_arm_nop();		\
		      __builtin_arm_nop();	\
		      __builtin_arm_nop();	\
	}while(0)

__attribute__((section(".ramfunc"))) __attribute__((noinline))   uint8_t wr_FlashSRx(uint8_t st)
{
 
#define SSI_ID 1
#define SRx 1
    
        volatile SSI_TypeDef *SSI=(SSI_TypeDef *)0x13000000;

        volatile uint32_t temp,CR0_Back ;
        uint32_t timeout = 0,reg;

        
        reg = arch_irq_lock(); //disable interrupt

        CCM_SSICFG &= ~((1 << (16 + SSI_ID - 1)) | (1 << (20 + SSI_ID - 1))); //disable XIP

        
        temp=SSI->SR;
        SSI->SSIENR=0;

        CR0_Back = SSI->CTRLR0;

        SSI->CTRLR0 &= ~(0x3 << 22);           //CR0 STD mode
        SSI->CTRLR0 |= (0x1 << 14);            //CR0 SSTE
        SSI->CTRLR0 &= ~(0x3 << 10);           //CR0 Tmode
        SSI->CTRLR0 &= ~(0x1F << 0);           //CR0 DFS,8 bits data
        SSI->CTRLR0 |= 0x07;
       
        SSI->SSIENR=1;
        
	FT_NOP();

        SSI->DR=0x05;	
        SSI->DR=0x55;

    
	FT_NOP();
        while(SSI->SR&0x01);   
 
        temp=SSI->DR;
        temp=SSI->DR;
   
        if(st&0x80){
            goto exit;
        }

        SSI->DR=0x06;
    
    
	FT_NOP();
        while(SSI->SR&0x01);
 
        temp=SSI->DR;

	FT_NOP();
        SSI->DR=0x1;
        SSI->DR=st;

	FT_NOP();
        while(SSI->SR&0x1);
        
        temp=SSI->DR;
        temp=SSI->DR;
        
        do{
            timeout++;
            if(timeout > 1000000)
                break;
            SSI->DR=0x05;
            SSI->DR=0x55;

	    FT_NOP();
            while(SSI->SR&0x1);
       
            temp=SSI->DR;
            temp=SSI->DR;

        }while((temp&0x01) == 0x01);
 
        SSI->DR=0x04;

	FT_NOP();
        while(SSI->SR&0x01);
 
        temp=SSI->DR; 

exit:
    SSI->SSIENR=0;
    SSI->CTRLR0=CR0_Back;
    SSI->SSIENR=1;

    CCM_SSICFG |= ((1 << (16 + SSI_ID - 1)) | (1 << (20 + SSI_ID - 1))); //enable XIP

    arch_irq_unlock(reg);
    return temp&0xff;
}

#endif


static uint8_t drv_xip_flash_erase(uint32_t addr)
{
    uint8_t ssi_id = drv_ssi_get_ssi_id(addr);

    ssi_cfg[ssi_id - 1].SsiId = ssi_id;
    ssi_cfg[ssi_id - 1].Cmd = SECT_ERASE_CMD;
    ssi_cfg[ssi_id - 1].Addr = addr & 0x03ffffff;

    xip_flash_erase(&ssi_cfg[ssi_id - 1]);
    return 0;
}

static uint8_t hal_xip_flash_erase(uint32_t addr)
{
    drv_xip_flash_erase(addr);
    ft_DRV_DCACHE_Invalidate(addr, SSI_SECTOR_SIZE);
    return 0;
}

static uint8_t drv_xip_flash_erase_block(uint32_t addr)
{
    uint8_t ssi_id = drv_ssi_get_ssi_id(addr);

    if (ssi_id == 1) // 0x10001000~0x101FFFFF
    {
        if ((addr >= FT_FLASH_BASE_ADDR+FT_FLASH_MAX_SIZE) || (addr <= FT_FLASH_BASE_ADDR))
        {
            return 1;
        }
    }
    ssi_cfg[ssi_id - 1].SsiId = ssi_id;
    ssi_cfg[ssi_id - 1].Cmd = BLOCK_ERASE_CMD;
    ssi_cfg[ssi_id - 1].Addr = addr & 0x03ffffff;

    xip_flash_erase(&ssi_cfg[ssi_id - 1]);
    return 0;
}

static uint8_t hal_xip_flash_erase_block(uint32_t addr)
{
    drv_xip_flash_erase_block(addr);
    ft_DRV_DCACHE_Invalidate(addr, 0x8000); // SSI_PAGE_SIZE
    return 0;
}

static uint8_t drv_xip_flash_erase_page(uint32_t addr)
{
    uint8_t ssi_id = drv_ssi_get_ssi_id(addr);

    if (ssi_id == 1) // 0x10001000~0x101FFFFF
    {
        if ((addr >=  FT_FLASH_BASE_ADDR+FT_FLASH_MAX_SIZE) || (addr <= FT_FLASH_BASE_ADDR))
        {
            return 1;
        }
    }
    ssi_cfg[ssi_id - 1].SsiId = ssi_id;
    ssi_cfg[ssi_id - 1].Cmd = PAGE_ERASE_CMD; // PAGE_ERASE_CMD;//SECT_ERASE_CMD;
    ssi_cfg[ssi_id - 1].Addr = addr & 0x03ffffff;

    xip_flash_erase(&ssi_cfg[ssi_id - 1]);
    return 0;
}

static uint8_t hal_xip_flash_erase_page(uint32_t addr)
{
    drv_xip_flash_erase_page(addr);
    ft_DRV_DCACHE_Invalidate(addr, SSI_PAGE_SIZE);
    return 0;
}

uint8_t ft_flash_erase_page(uint32_t addr)
{
    uint32_t start_addr = 0;
    uint16_t remain = addr % FLASH_PAGE_SIZE;
    start_addr = addr - remain;
    return hal_xip_flash_erase_page(start_addr);
}

uint8_t ft_flash_erase_sector(uint32_t addr)
{
    uint32_t start_addr = 0;
    uint32_t remain = addr % FLASH_SECTOR_SIZE;
    start_addr = addr - remain;
    return hal_xip_flash_erase(start_addr);
}

uint8_t ft_flash_erase_block(uint32_t addr)
{
    uint32_t start_addr = 0;
    uint32_t remain = addr % FLASH_BLOCK_SIZE;
    start_addr = addr - remain;
    return hal_xip_flash_erase_block(start_addr);
}

static uint8_t drv_xip_flash_program(uint32_t addr, uint8_t *p_buff, uint32_t len)
{
    uint8_t ssi_id = drv_ssi_get_ssi_id(addr);
    if (ssi_id == 1) // 0x10001000~0x101FFFFF
    {
        if ((addr >= FT_FLASH_BASE_ADDR+FT_FLASH_MAX_SIZE) || (addr < FT_FLASH_BASE_ADDR))
        {
            return 1;
        }
    }

    ssi_cfg[ssi_id - 1].SsiId = ssi_id;
    ssi_cfg[ssi_id - 1].Cmd = QUAD_PROG_CMD;
    ssi_cfg[ssi_id - 1].Value = 0x00;
    ssi_cfg[ssi_id - 1].ProgramMode = QUAD_PROGRAM;

    ssi_cfg[ssi_id - 1].Addr = addr & 0x03ffffff;
    ssi_cfg[ssi_id - 1].Len = len;
    ssi_cfg[ssi_id - 1].Buf = (uint32_t)p_buff;

    xip_flash_program(&ssi_cfg[ssi_id - 1]);
    return 0;
}

static uint8_t hal_xip_flash_program(uint32_t addr, uint8_t *p_buff, uint32_t len)
{
    drv_xip_flash_program(addr, p_buff, len);
    ft_DRV_DCACHE_Invalidate(addr, SSI_PAGE_SIZE);
    return 0;
}

static int flash_bulk_program(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t i = 0;
    int ret = 0;
    uint8_t *ptr = (uint8_t *)data;
    uint32_t start_addr = addr;
    uint32_t page_count = len / FLASH_PAGE_SIZE;
    uint16_t remain = len % FLASH_PAGE_SIZE;
    // FF_CHECK_PTR(data);

    for (i = 0; i < page_count; i++)
    {
        ret = hal_xip_flash_program(start_addr, ptr, FLASH_PAGE_SIZE);
        if (ret != 0)
        {
            break;
        }
        start_addr += FLASH_PAGE_SIZE;
        ptr += FLASH_PAGE_SIZE;
    }
    // remain data
    if (ret == 0&&remain!=0)
    {
        ret = hal_xip_flash_program(start_addr, ptr, remain);
    }

    return ret;
}

static int flash_read_data(uint32_t addr, uint8_t *data, uint32_t len)
{

    uint32_t i = 0;
    // FF_CHECK_PTR(data);

    for (i = 0; i < len; i++)
    {
        data[i] = *((uint8_t *)(addr + i));
    }

    return 0;
}

/*erase flash, 4K align*/
static int flash_erase_data(uint32_t addr, uint32_t len)
{
    int ret = 0;
    uint16_t remain = 0;
    uint32_t addr_start = addr;
    int32_t length = len;

    if (addr_start % FLASH_SECTOR_SIZE != 0)
    {
        // MSG_ERR("the addr(%08x) must be an interger multiple of the sector(%08x)", addr_start, FLASH_SECTOR_SIZE);
        return -1;
    }

    do
    {
        if ((length >= FLASH_BLOCK_SIZE) && ((addr_start % FLASH_BLOCK_SIZE) == 0))
        {
            ret = hal_xip_flash_erase_block(addr_start);
            addr_start += FLASH_BLOCK_SIZE;
            length -= FLASH_BLOCK_SIZE;
        }
        else if ((length >= FLASH_SECTOR_SIZE) && ((addr_start % FLASH_SECTOR_SIZE) == 0))
        {
            ret = hal_xip_flash_erase(addr_start);
            addr_start += FLASH_SECTOR_SIZE;
            length -= FLASH_SECTOR_SIZE;
        }
        else
        {
            remain = addr_start % FLASH_SECTOR_SIZE;
            addr_start = addr_start - remain;
            ret = hal_xip_flash_erase(addr_start);
            addr_start += FLASH_SECTOR_SIZE;
            length = length - FLASH_SECTOR_SIZE + remain;
        }

        if (ret != 0)
        {
            // MSG_ERR("flash erase error !");
            break;
        }
    } while (length > 0);

    return ret;
}

/*
**When writing data to flash, the function includes boundary checks on the flash addresses. 
**In addition, the flash_erase_data function should be called to erase data before writing.
*/

static int flash_write_data(uint32_t addr, uint8_t *data, uint32_t len)
{
    int ret = 0;

    uint32_t i = 0;
    uint32_t start_addr = addr;

    // FF_CHECK_PTR(data);

    if ((start_addr < FLASH_ADDR_START) || (start_addr > FLASH_ADDR_END))
    {
        printk("this addr is not authoried to write(%08x)", start_addr);
        return -2;
    }

    do
    {

        if ((start_addr < FT_FLASH_BASE_ADDR) || (start_addr >=FT_FLASH_MAX_SIZE+FT_FLASH_BASE_ADDR) )
        {
            printk("this app addr or len is wrong(%08x, %x)", start_addr, len);
            ret = -1;
            break;
        }
        else
        {

            ret = flash_bulk_program(addr, data, len);


            if (ret == 0)
            {
                /* Compare whether the written data is correct. */
                for (i = 0; i < len; i++)
                {
                    if (*((uint8_t *)(addr + i)) != data[i])
                    {
                        printk(" APP_FLASH_ID: %x != %x", *((uint8_t *)(addr + i)), data[i]);
                        ret = -3;
                        break;
                    }
                }
            }
            else
            {
                printk("flash_bulk_program(id = %d) =  %d", CFG_FLASH_ID, ret);
                break;
            }
        }

    } while (0);

    return ret;
}

static int flash_ft90_read(const struct device *dev, off_t addr, void *dest, size_t size)
{
	uint32_t physical_addr=addr|FT_FLASH_BASE_ADDR;

	flash_read_data(physical_addr, dest, size);
	return 0;
}

static int flash_ft90_write(const struct device *dev, off_t addr, const void *src,
				      size_t size)
{
	uint32_t physical_addr=addr|FT_FLASH_BASE_ADDR;
	off_t offset=addr%SSI_PAGE_SIZE;
	int ret=0;


	if(offset){
	    uint8_t page_read_back[SSI_PAGE_SIZE];
	    memset(page_read_back,0xff,SSI_PAGE_SIZE);

	    off_t addr_priv=(addr/SSI_PAGE_SIZE)*SSI_PAGE_SIZE;
	    flash_ft90_read(dev,addr_priv,page_read_back,offset);
	    offset=SSI_PAGE_SIZE-offset;
	    if(size>offset){
		memcpy(page_read_back+SSI_PAGE_SIZE-offset,src,offset);
	    }else{
		memcpy(page_read_back+SSI_PAGE_SIZE-offset,src,size);
	    }

	    flash_ft90_erase(dev,addr_priv,SSI_PAGE_SIZE);
	    physical_addr=addr_priv|FT_FLASH_BASE_ADDR;

	    ret = flash_write_data(physical_addr, page_read_back, SSI_PAGE_SIZE);
	    if(ret){

		printk("flash write failed1\n");
		return ret;
	    }


	    if(size>offset){
		 physical_addr=physical_addr+SSI_PAGE_SIZE;
		 ret = flash_write_data(physical_addr, (uint8_t*)src+offset, size-offset);

		 if(ret){
			printk("flash write failed2\n");
		 }
	    }
	    return ret;
	}

	
	 ret = flash_write_data(physical_addr, (uint8_t*)src, size);
	return ret;
}

static int flash_ft90_erase(const struct device *dev, off_t addr, size_t size)
{
	
	uint32_t physical_addr=addr|FT_FLASH_BASE_ADDR;

	flash_erase_data(physical_addr, size);
	return 0;
}



static int flash_ft90_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	ft_xip_flash_init();

	return 0;
}


#if defined(CONFIG_FLASH_EX_OP_ENABLED)

static int flash_ft90_ex_op(const struct device *dev, uint16_t code,
                               const uintptr_t in, void *out)
{
	
	ARG_UNUSED(dev);
	uint8_t *value;
		
	switch(code){
	    
	    case FT_FLASH_EX_OP_SET_WP:
		if(out){
		    value=(uint8_t*)out;
		    printk("unsupport wp right now,ingnore,enable=%d\n",*value);
		}
		break;

	    case FT_FLASH_EX_OP_GET_WP:
		printk("unsupport wp right now,ingnore\n");
		    if(in){
			value=(uint8_t*)in;
			*value=0;
		    }
		    break;
	    
        case FT_FLASH_FT90_EX_OP_WR_SR:
		value=(uint8_t*)out;

	    //		printk("WR sr=%d",*value);
		if(*value>0x7f){

		    printk("wrong value\n");
		    return 0;
		}

		wr_FlashSRx(*value);

		break;

        case FT_FLASH_FT90_EX_OP_RD_SR:
		value=(uint8_t*)in;
		*value=wr_FlashSRx(0x80);
		//printk("RD sr=%d",*value);
		break;

		default :break;
	}

	return 0;
}

#endif



static const struct flash_parameters *flash_ft90_get_parameters(const struct device *dev)
{
	//const struct flash_ft90_config *config = dev->config;
	ARG_UNUSED(dev);

	return &flash_ft90_parameters;//config->parameters;
}

static const struct flash_driver_api flash_ft90_api = {
	.read = flash_ft90_read,
	.write = flash_ft90_write,
	.erase = flash_ft90_erase,
	.get_parameters = flash_ft90_get_parameters,
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
	.ex_op=flash_ft90_ex_op,
#endif
};

#define FLASH_FT_INIT(n)                                        \
                                                                            \
	DEVICE_DT_INST_DEFINE(n, &flash_ft90_init, NULL, NULL,              \
			      NULL, POST_KERNEL,                            \
			      CONFIG_FLASH_FT_FT90_INIT_PRIORITY, &flash_ft90_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_FT_INIT)
