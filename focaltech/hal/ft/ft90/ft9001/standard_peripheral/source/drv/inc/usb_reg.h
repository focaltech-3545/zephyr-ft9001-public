
#ifndef __USB_REG__
#define __USB_REG__

#include "type.h"
/**
* @brief USBC DMA Registers
*/
typedef struct
{
    __IO uint32_t DMA_CNTL;
    __IO uint32_t DMA_ADDR;
    __IO uint32_t DMA_COUNT;
    __IO uint32_t RESERVED;
} DMAReg_TypeDef;

/**
* @brief USBC Registers
*/
typedef struct
{
    __IO uint8_t FADDRR;                                                            /**<Function address register*/
    __IO uint8_t UCSR;                                                              /**<USB control and status register*/

    union
	{
		__IO uint16_t INTRTX;
		struct
		{
			__IO uint8_t INTRTX_L;                                                 /**<Low byte of Interrupt register for Endpoint0 and Tx Endpoint*/
			__IO uint8_t INTRTX_H;                                                 /**<High byte of Interrupt register for Endpoint0 and Tx Endpoint*/
		};
	};
    
    union
	{
		__IO uint16_t INTRRX;
		struct
		{
			__IO uint8_t INTRRX_L;                                                  /**<Low byte of Interrupt register for Rx Endpoint*/
			__IO uint8_t INTRRX_H;                                                  /**<High byte of Interrupt register for Rx Endpoint*/
		};
	};

    union
	{
		__IO uint16_t INTRTXE;
		struct
		{
			__IO uint8_t INTRTXE_L;                                                 /**<Low byte of Interrupt enable register for IntrTx*/
			__IO uint8_t INTRTXE_H;                                                 /**<High byte of Interrupt enable register for IntrTx*/
		};
	};
    
    union
	{
		__IO uint16_t INTRRXE;
		struct
		{
			__IO uint8_t INTRRXE_L;                                                 /**<Low byte of Interrupt enable register for IntrRx*/
			__IO uint8_t INTRRXE_H;                                                 /**<High byte of Interrupt enable register for IntrRx*/
		};
	};

    __IO uint8_t INTRUSB;                                                           /**<Interrupt register for common USB interrupts*/
    __IO uint8_t INTRUSBE;                                                          /**<Interrupt enable register for IntrUSB*/

    union
	{
		__IO uint16_t FNUMR;
		struct
		{
			__IO uint8_t FNUMR_L;                                                   /**<Low byte of Frame number*/
			__IO uint8_t FNUMR_H;                                                   /**<High byte of Frame number*/
		};
	};

    __IO uint8_t EINDEX;                                                            /**<Index register for selecting the endpoint status and control register*/
    __IO uint8_t TSTMODE;                                                           /**<Enables the USB test modes*/

    union
	{
		__IO uint16_t TXMAXP;
		struct
		{
			__IO uint8_t TXMAXP_L;                                                  /**<Low byte of Maximum packet size for peripheral Tx endpoint*/
			__IO uint8_t TXMAXP_H;                                                  /**<High byte of Maximum packet size for peripheral Tx endpoint*/
		};
	};
    
    union
    {
        __IO uint8_t E0CSR_L;                                                       /**<Low byte of Control Status register for Endpoint0*/
        __IO uint8_t TXCSR_L;                                                       /**<Low byte of Control Status register for peripheral Tx endpoint*/
    };
    union
    {
        __IO uint8_t E0CSR_H;                                                       /**<High byte of Control Status register for Endpoint0*/
        __IO uint8_t TXCSR_H;                                                       /**<High byte of Control Status register for peripheral Tx endpoint*/
    };

    union
	{
		__IO uint16_t RXMAXP;
		struct
		{
			__IO uint8_t RXMAXP_L;                                                  /**<Low byte of Maximum packet size for peripheral Rx endpoint*/
			__IO uint8_t RXMAXP_H;                                                  /**<High byte of Maximum packet size for peripheral Rx endpoint*/
		};
	};

    __IO uint8_t RXCSR_L;                                                           /**<Low byte of Control Status register for peripheral Rx endpoint*/
    __IO uint8_t RXCSR_H;                                                           /**<High byte of Control Status register for peripheral Rx endpoint*/
    union
    {
		__IO uint16_t E0COUNTR;
		struct
		{
		    __IO uint8_t E0COUNTR_L;                                                /**<Low byte of Number of received bytes in Endpoint0 FIFO*/
		    __IO uint8_t E0COUNTR_H;                                                /**<High byte of Number of received bytes in Endpoint0 FIFO*/
	    };  
        __IO uint16_t RXCOUNTR;
        struct
        {
            __IO uint8_t RXCOUNTR_L;                                                /**<Low byte of Number of bytes in peripheral Rx endpoint FIFO*/
            __IO uint8_t RXCOUNTR_H;                                                /**<High byte of Number of bytes in peripheral Rx endpoint FIFO*/
        };
    };
    __IO uint8_t TXTYPE;
    union
    {
        __IO uint8_t NAKLIMIT0;
        __IO uint8_t TXINTERVAL;
    };
    __IO uint8_t RXTYPE;                                                            /*0x1C  *< */
    __IO uint8_t RXINTERVAL;                                                        /*0x1D  *< */
    __IO uint16_t rsv_0x1e_0x1f;
    __IO uint32_t FIFO_EP0;
    __IO uint32_t rsv_0x24_0x5f[15];
    __IO uint8_t OTGCTRL;
    __IO uint8_t RESERVED;
    __IO uint8_t TXFIFOSZ;                                                          /**<Tx Endpoint FIFO size,double buffer only set in one register(TX_fifosz/RX_fifosz)*/
    __IO uint8_t RXFIFOSZ;                                                          /**<Rx Endpoint FIFO size,MAX FIFO size is 1024byte*/
    __IO uint8_t TX_fifoadd_L;                                                      /**<Tx Endpoint FIFO address(Low 8bit)*/
    __IO uint8_t TX_fifoadd_H;                                                      /**<Tx Endpoint FIFO address(High 8bit)*/
    __IO uint8_t RX_fifoadd_L;                                                      /**<Rx Endpoint FIFO address(Low 8bit)*/
    __IO uint8_t RX_fifoadd_H;                                                      /**<Rx Endpoint FIFO address(High 8bit)*/
    __IO uint32_t rsv_0x68_0x1ff[102];                                              /*6+32+64*/
    __IO uint32_t DMA_INTR;                                                         /*0x200*/
    DMAReg_TypeDef USB_DMAReg[8];                                                   /*0x204-0x283, USBC DMA Channel Registers */
} FT_USBD_Type;

/**
* @brief USBC PHY Registers
*/
typedef struct
{
    __IO uint32_t PHY_ADDR;
    __IO uint32_t PHY_WDATA;
    __IO uint32_t PHY_WREN;
    __IO uint32_t PHY_RDEN;
    __IO uint32_t PHY_RDATA;
} USBCPHYREG_TypeDef;

//#define USB_BASE_ADDR                         (0x4004C000)
//#define USBC_BASE_ADDR                        (0x4004C000)
//#define USBC_INDEXED_ADDR                   (USBC_BASE_ADDR + 0x10)
//#define USBC_FIFOCFG_ADDR                   (USBC_BASE_ADDR + 0x60)
//#define USBC_FIFOREG_ADDR                   (USBC_BASE_ADDR + 0x20)
//#define USBC_DMACCFG_ADDR                   (USBC_BASE_ADDR + 0x200)
//#define USBC_PHYREG_ADDR                    (USBC_BASE_ADDR + 0x800)
//#define gUSBC_CommonReg                     ((USBCCOMMON_TypeDef *)(USBC_BASE_ADDR))
//#define gUSBC_IndexReg                      ((USBCINDEXED_TypeDef *)USBC_INDEXED_ADDR)
//#define gUSBC_ControlReg                    ((USBCFIFOCFG_TypeDef *)USBC_FIFOCFG_ADDR)
//#define gUSBC_FIFOReg                       ((USBCFIFOREG_TypeDef *)USBC_FIFOREG_ADDR)
//#define gUSBC_DMAReg                        ((USBCDMACFG_TypeDef *)USBC_DMACCFG_ADDR)
//#define gUSBC_PHYReg                        ((USBCPHYREG_TypeDef *)USBC_PHYREG_ADDR)

#define USB_POWER_ENAB_SUSP                 (1 << 0)
#define USB_POWER_SUSP_MODE                 (1 << 1)
#define USB_POWER_RESUME                    (1 << 2)
#define USB_POWER_RESET                     (1 << 3)
#define USB_POWER_HS_MODE                   (1 << 4)
#define USB_POWER_HS_ENAB                   (1 << 5)
#define USB_POWER_SOFT_CONN                 (1 << 6)
#define USB_POWER_ISO_UPDATE                (1 << 7)

//usb common interrupt number
#define USB_INTERRUPT_SUSPEND               (1 << 0)
#define USB_INTERRUPT_RESUME                (1 << 1)
#define USB_INTERRUPT_RESET                 (1 << 2)
#define USB_INTERRUPT_SOF                   (1 << 3)
#define USB_INTERRUPT_CONNECT               (1 << 4)
#define USB_INTERRUPT_DISCON                (1 << 5)
#define USB_INTERRUPT_SESSREQ               (1 << 6)
#define USB_INTERRUPT_VBUSERR               (1 << 7)

#define USB_TESTMODE_SE0NAK                 (1 << 0)
#define USB_TESTMODE_TESTJ                  (1 << 1)
#define USB_TESTMODE_TESTK                  (1 << 2)
#define USB_TESTMODE_TESTPACKET             (1 << 3)

//usb tx interrupt number
#define USB_INTERRUPT_EP0                   (1 << 0)
#define USB_TX_INTERRUPT_EP1                (1 << 1)
#define USB_TX_INTERRUPT_EP2                (1 << 2)
#define USB_TX_INTERRUPT_EP3                (1 << 3)
#define USB_TX_INTERRUPT_EP4                (1 << 4)
#define USB_TX_INTERRUPT_EP5                (1 << 5)
#define USB_TX_INTERRUPT_EP6                (1 << 6)
#define USB_TX_INTERRUPT_EP7                (1 << 7)

//Usb Rx Interrupt Number
#define USB_RX_INTERRUPT_EP0                (1 << 0)
#define USB_RX_INTERRUPT_EP1                (1 << 1)
#define USB_RX_INTERRUPT_EP2                (1 << 2)
#define USB_RX_INTERRUPT_EP3                (1 << 3)
#define USB_RX_INTERRUPT_EP4                (1 << 4)
#define USB_RX_INTERRUPT_EP5                (1 << 5)
#define USB_RX_INTERRUPT_EP6                (1 << 6)
#define USB_RX_INTERRUPT_EP7                (1 << 7)
//Device CSR0 Bit Define
#define DEV_CSR0_RXPKTRDY                   (1 << 0)
#define DEV_CSR0_TXPKTRDY                   (1 << 1)
#define DEV_CSR0_SENTSTALL                  (1 << 2)
#define DEV_CSR0_DATAEND                    (1 << 3)
#define DEV_CSR0_SETUPEND                   (1 << 4)
#define DEV_CSR0_SENDSTALL                  (1 << 5)
#define DEV_CSR0_SERVICE_RXPKTRDY           (1 << 6)
#define DEV_CSR0_SERVICE_SETUPEND           (1 << 7)

//TX Register Bit Low as Device
#define DEV_TXCSR_TXPKTRDY                  (1 << 0)
#define DEV_TXCSR_FIFO_NOT_EMPTY            (1 << 1)
#define DEV_TXCSR_UNDER_RUN                 (1 << 2)
#define DEV_TXCSR_FLUSH_FIFO                (1 << 3)
#define DEV_TXCSR_SEND_STALL                (1 << 4)
#define DEV_TXCSR_SENT_SATLL                (1 << 5)
#define DEV_TXCSR_CLR_DATA_TOG              (1 << 6)
#define DEV_TXCSR_INCOMP_TX                 (1 << 7)

//TX Register Bit High as Device
#define DEV_TXCSR_DMAMODE                   (1 << 2)
#define DEV_TXCSR_FRC_DATA_TOG              (1 << 3)
#define DEV_TXCSR_DMA_ENAB                  (1 << 4)
#define DEV_TXCSR_TXMODE                    (1 << 5)
#define DEV_TXCSR_ISO                       (1 << 6)
#define DEV_TXCSR_AUTO_SET                  (1 << 7)

//RX Register Bit Low as Device
#define DEV_RXCSR_RXPKTRDY                  (1 << 0)
#define DEV_RXCSR_FIFOFULL                  (1 << 1)
#define DEV_RXCSR_FLUSH_FIFO                (1 << 4)
#define DEV_RXCSR_SEND_STALL                (1 << 5)
#define DEV_RXCSR_SENT_STALL                (1 << 6)
#define DEV_RXCSR_CLR_DATA_TOG              (1 << 7)

//RX Register Bit High as Device
#define DEV_RXCSR_INCOMP_RX                 (1 << 0)
#define DEV_RXCSR_DMAMODE                   (1 << 3)
#define DEV_RXCSR_DISNYET                   (1 << 4)
#define DEV_RXCSR_DMA_ENAB                  (1 << 5)
#define DEV_RXCSR_ISO                       (1 << 6)
#define DEV_RXCSR_AUTOCLEAR                 (1 << 7)

#define DEV_INTR_CHANNEL(n)                 (1 << (n - 1))

//dma cntl
#define DEV_CNTL_DMAEN                      (1 << 0)
#define DEV_CNTL_DIRECTION_READ             (1 << 1)
#define DEV_CNTL_DMAMODE                    (1 << 2)
#define DEV_CNTL_INTERE                     (1 << 3)
#define DEV_CNTL_EP(x)                      ((x & 0x07) << 4)
#define DEV_CNTL_BUSERROR                   (1 << 8)
#define DEV_CNTL_BURSTMODE(x)               ((x & 0x03) << 9)


#define USB_FADDR_OFFSET 0x00
#define USB_POWER_OFFSET 0x01
#define USB_TXIS_OFFSET  0x02
#define USB_RXIS_OFFSET  0x04
#define USB_TXIE_OFFSET  0x06
#define USB_RXIE_OFFSET  0x08
#define USB_IS_OFFSET    0x0A
#define USB_IE_OFFSET    0x0B
#define USB_FRAM_OFFSET  0x0C

#define USB_EPIDX_OFFSET 0x0E

#define USB_IND_TXMAP_OFFSET   0x10
#define USB_IND_TXCSRL_OFFSET  0x12
#define USB_IND_TXCSRH_OFFSET  0x13
#define USB_IND_RXMAP_OFFSET   0x14
#define USB_IND_RXCSRL_OFFSET  0x16
#define USB_IND_RXCSRH_OFFSET  0x17
#define USB_IND_RXCOUNT_OFFSET 0x18

#define USB_FIFO_OFFSET   0x20
#define USB_DEVCTL_OFFSET 0x60

#define USB_TXFIFOSZ_OFFSET  0x62
#define USB_RXFIFOSZ_OFFSET  0x63
#define USB_TXFIFOADD_OFFSET 0x64
#define USB_RXFIFOADD_OFFSET 0x66


#endif /* end __USB__ */