/**
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#include "usb_drv.h"
#include "cpm_drv.h"
#include "cpm_reg.h"
#include "stdio.h"
#include "uart_reg.h"
#include "usb_reg.h"

int USBC_EpxOpen(FT_USBD_Type *USBCx, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type, int speed_idx)
{
    uint8_t ep_idx;
    uint8_t saved_idx;
    uint8_t ep_dir_in;
    uint16_t pkt_size;
    uint16_t fifo_addr;

    ep_idx = ep_addr & 0x7F;
    ep_dir_in = (ep_addr >> 7) & 0x01;
    pkt_size = ep_mps;

    fifo_addr = ep_idx * 0x200;
    // printf("HAL_USBCEPxOpen pkt_size:%x, fifo_addr:%x, ep:%x\n", pkt_size, fifo_addr, ep_addr);
    // size = MIN(udc_mps_ep_size(ep), cfg->ep_mps);

    saved_idx = USBCx->EINDEX;
    USBCx->EINDEX = ep_idx;
    // set FIFO size,USB1.1
    if (speed_idx == 2)
    {
        // usb2.0
        if (ep_dir_in)
        {
            USBCx->RXFIFOSZ = 0x06;
        }
        else
        {
            USBCx->TXFIFOSZ = 0x06;
        }
    }
    else
    {
        // usb1.1
        if (ep_dir_in)
        {
            USBCx->RXFIFOSZ = 0x03;
        }
        else
        {
            USBCx->TXFIFOSZ = 0x03;
        }
    }

    if (!ep_idx) // ep0
    {
        // set fifo offset address
        if (ep_dir_in)
        {
            USBCx->TX_fifoadd_L = (uint8_t)((fifo_addr >> 3) & 0x00FF);
            USBCx->TX_fifoadd_H = (uint8_t)((fifo_addr >> 11) & 0x00FF);

            // USBCx->TXTYPE = ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK)<<4) | (ep_idx);
            // USBCx->TXTYPE = (ep_type<<4) | (ep_idx);
            USBCx->E0CSR_L = DEV_CSR0_SERVICE_RXPKTRDY | DEV_CSR0_DATAEND;
            USBCx->E0CSR_H = 0x0;

            USBCx->TXMAXP_L = (uint8_t)(pkt_size & 0xFF);
            USBCx->TXMAXP_H = (uint8_t)((pkt_size >> 8) & 0xFF);

            // Flush Tx Ep FIFO
            USBCx->E0CSR_H = 0x1;

            // USBCx->TXTYPE = ep_idx | ((ep_type&0x0F)<<4);
        }
        else
        {
            // set fifo offset address(0~0x2000,only 8k SRAM can set to be FIFO)
            USBCx->RX_fifoadd_L = (uint8_t)((fifo_addr >> 3) & 0x00FF);
            USBCx->RX_fifoadd_H = (uint8_t)((fifo_addr >> 11) & 0x00FF);

            // USBCx->RXTYPE = ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK)<<4) | (ep_idx);
            // USBCx->RXTYPE = (ep_type <<4) | (ep_idx);

            USBCx->RXCSR_L = DEV_RXCSR_CLR_DATA_TOG;
            USBCx->RXCSR_H = 0x0;

            USBCx->RXMAXP_L = (uint8_t)(pkt_size & 0xFF);
            USBCx->RXMAXP_H = (uint8_t)((pkt_size >> 8) & 0x07);
            // Flush Rx Ep FIFO
            // USBCx->RXCSR_L = DEV_RXCSR_FLUSH_FIFO;
            // USBCx->E0CSR_H = DEV_CSR0_FLUSH_FIFO;
            USBCx->E0CSR_H = 0x1;
            // USBCx->RXTYPE = ep_idx | ((ep_type&0x03)<<4);
        }
        USBCx->INTRTX_L |= (1 << ep_idx);

        USBCx->EINDEX = saved_idx;
        return 0;
    }

    // set fifo offset address
    if (ep_dir_in)
    {
        USBCx->TX_fifoadd_L = (uint8_t)((fifo_addr >> 3) & 0x00FF);
        USBCx->TX_fifoadd_H = (uint8_t)((fifo_addr >> 11) & 0x00FF);

        USBCx->TXCSR_L = DEV_TXCSR_CLR_DATA_TOG;
        USBCx->TXCSR_H = 0;

        USBCx->TXMAXP_L = (uint8_t)(pkt_size & 0xFF);
        USBCx->TXMAXP_H = (uint8_t)((pkt_size >> 8) & 0x07);

        // Flush Tx Ep FIFO
        USBCx->TXCSR_L = DEV_TXCSR_FLUSH_FIFO;
#if 1
        if ((ep_type & 0x03) == 0x01) // USB_EP_TYPE_ISO
        {
            USBCx->TXCSR_H = DEV_TXCSR_ISO;
        }
        else // USB_EP_TYPE_BULK/USB_EP_TYPE_INTERRUPT/USB_EP_TYPE_CONTROL
        {
            USBCx->TXCSR_H = 0x00;
        }
#endif
        // USBCx->TXTYPE = ep_idx | ((ep_type&0x03)<<4);
        USBCx->INTRTX_L |= (1 << ep_idx);
    }
    else
    {
        // set fifo offset address(0~0x2000,only 8k SRAM can set to be FIFO)
        USBCx->RX_fifoadd_L = (uint8_t)((fifo_addr >> 3) & 0x00FF);
        USBCx->RX_fifoadd_H = (uint8_t)((fifo_addr >> 11) & 0x00FF);

        USBCx->RXCSR_L = DEV_RXCSR_CLR_DATA_TOG;
        USBCx->RXCSR_H = 0x0;

        USBCx->RXMAXP_L = (uint8_t)(pkt_size & 0xFF);
        USBCx->RXMAXP_H = (uint8_t)((pkt_size >> 8) & 0x07);
        // Flush Rx Ep FIFO
        USBCx->RXCSR_L = DEV_RXCSR_FLUSH_FIFO;
#if 1
        if ((ep_type & 0x03) == 0x01) // USB_EP_TYPE_ISO
        {
            USBCx->RXCSR_H = DEV_RXCSR_ISO;
        }
        else // USB_EP_TYPE_BULK/USB_EP_TYPE_INTERRUPT/USB_EP_TYPE_CONTROL
        {
            USBCx->RXCSR_H = 0x00;
        }
#endif
        // USBCx->RXTYPE = ep_idx | ((ep_type&0x03)<<4);
        USBCx->INTRRX_L |= (1 << ep_idx);
    }
    USBCx->EINDEX = saved_idx;
    return 0;
}

int USBC_EpxReset(FT_USBD_Type *USBCx, uint8_t ep_addr)
{
    uint8_t ep_idx;
    uint8_t saved_idx;
    uint8_t ep_dir_in;
    uint16_t fifo_addr;

    ep_idx = ep_addr & 0x7F;
    ep_dir_in = (ep_addr >> 7) & 0x01;
    saved_idx = USBCx->EINDEX;
    USBCx->EINDEX = ep_idx;

    fifo_addr = ep_idx * 0x200;

    if (!ep_idx) // ep0
    {
        USBCx->INTRTX_L &= ~(1 << ep_idx);
        // set fifo offset address
        if (ep_dir_in)
        {
            USBCx->E0CSR_L = DEV_CSR0_SERVICE_RXPKTRDY | DEV_CSR0_DATAEND;
            USBCx->E0CSR_H = 0x0;
            USBCx->E0CSR_H = 0x1;
        }
        else
        {
            // set fifo offset address(0~0x2000,only 8k SRAM can set to be FIFO)
            USBCx->RX_fifoadd_L = (uint8_t)((fifo_addr >> 3) & 0x00FF);
            USBCx->RX_fifoadd_H = (uint8_t)((fifo_addr >> 11) & 0x00FF);
            USBCx->RXCSR_L = DEV_RXCSR_CLR_DATA_TOG;
            USBCx->E0CSR_H = 0x1;
        }
        USBCx->INTRTX_L |= (1 << ep_idx);

        USBCx->EINDEX = saved_idx;
        return 0;
    }

    // set fifo offset address
    if (ep_dir_in)
    {
        USBCx->INTRTX_L &= ~(1 << ep_idx);

        USBCx->TXCSR_L = DEV_TXCSR_CLR_DATA_TOG;
        USBCx->TXCSR_H = 0;

        USBCx->TXMAXP_L = 0x00;
        USBCx->TXMAXP_H = 0x00;

        // Flush Tx Ep FIFO
        USBCx->TXCSR_L = DEV_TXCSR_FLUSH_FIFO;
        // USBCx->TXTYPE = ep_idx | ((ep_type&0x03)<<4);
        USBCx->INTRTX_L |= (1 << ep_idx);
    }
    else
    {
        USBCx->INTRRX_L &= ~(1 << ep_idx);
        // set fifo offset address(0~0x2000,only 8k SRAM can set to be FIFO)
        USBCx->RX_fifoadd_L = (uint8_t)((fifo_addr >> 3) & 0x00FF);
        USBCx->RX_fifoadd_H = (uint8_t)((fifo_addr >> 11) & 0x00FF);

        USBCx->RXCSR_L = DEV_RXCSR_CLR_DATA_TOG;
        USBCx->RXCSR_H = 0x0;

        USBCx->RXMAXP_L = 0x00;
        USBCx->RXMAXP_H = 0x00;
        // Flush Rx Ep FIFO
        USBCx->RXCSR_L = DEV_RXCSR_FLUSH_FIFO;

        // USBCx->RXTYPE = ep_idx | ((ep_type&0x03)<<4);
        USBCx->INTRRX_L |= (1 << ep_idx);
    }
    USBCx->EINDEX = saved_idx;
    return 0;
}

int USBC_EpxClose(FT_USBD_Type *USBCx, uint8_t ep_addr)
{
    uint8_t ep_idx;
    uint8_t ep_dir_in;
    uint8_t saved_idx;

    ep_idx = ep_addr & 0x7F;
    ep_dir_in = (ep_addr >> 7) & 0x01;
    saved_idx = USBCx->EINDEX;
    USBCx->EINDEX = ep_idx;

    if (!ep_idx) // ep0
    {
        // set fifo offset address
        if (ep_dir_in)
        {
            USBCx->E0CSR_H = 0x01;
            USBCx->INTRTX_L &= ~(1 << ep_idx);
        }
        else
        {
            // set fifo offset address(0~0x2000,only 8k SRAM can set to be FIFO)
            USBCx->RXCSR_L = DEV_RXCSR_CLR_DATA_TOG;
            USBCx->RXCSR_H = 0x0;

            // Flush Rx Ep FIFO
            USBCx->E0CSR_H = 0x01; // DEV_CSR0_FLUSH_FIFO;
            USBCx->INTRRX_L &= ~(1 << ep_idx);
        }
        USBCx->EINDEX = saved_idx;
        return 0;
    }

    // set fifo offset address
    if (ep_dir_in)
    {
        USBCx->TXCSR_L = DEV_TXCSR_CLR_DATA_TOG;
        USBCx->TXCSR_H = 0x0;
        // Flush Tx Ep FIFO
        USBCx->TXCSR_L = DEV_TXCSR_FLUSH_FIFO;
        USBCx->INTRTX_L &= ~(1 << ep_idx);
    }
    else
    {
        // set fifo offset address(0~0x2000,only 8k SRAM can set to be FIFO)
        USBCx->RXCSR_L = DEV_RXCSR_CLR_DATA_TOG;
        USBCx->RXCSR_H = 0x0;
        // Flush Rx Ep FIFO
        USBCx->RXCSR_L = DEV_RXCSR_FLUSH_FIFO;
        USBCx->INTRRX_L &= ~(1 << ep_idx);
    }
    USBCx->EINDEX = saved_idx;
    return 0;
}

void USBC_BusReset(FT_USBD_Type *USBCx, uint8_t speed_idx)
{
    uint8_t tempL, tempH;
    uint8_t ep_idx = 1;
    uint16_t fifo_addr;
    uint8_t csr_l;
    uint8_t usb_mode;

    // write FAddr 0
    USBCx->FADDRR = 0;
    // access DATA_OUT_EP register map
    // device into idle state
    // check whether device works on HS.
    usb_mode = USBCx->UCSR; // need clear suspend flag?
    usb_mode = 0;

    USBCx->EINDEX = 0;
    // set FIFO size
    if (speed_idx == 0x02)
    {
        USBCx->TXFIFOSZ = 0x06;
        USBCx->RXFIFOSZ = 0x06;
        tempL = USB_MAX_PACKET_SIZE_LOW;
        tempH = USB_MAX_PACKET_SIZE_HIGH;
    }
    else
    {
        USBCx->TXFIFOSZ = 0x03;
        USBCx->RXFIFOSZ = 0x03;
        tempL = USB_MAX_PACKET_SIZE_LOW_V11;
        tempH = USB_MAX_PACKET_SIZE_HIGH_V11;
    }

    csr_l = USBCx->E0CSR_L;
    // printf("USBC_BusReset :%x\n", csr_l);
    USBCx->E0CSR_L = DEV_CSR0_SERVICE_RXPKTRDY | DEV_CSR0_DATAEND;
    USBCx->E0CSR_H = 0x0;

    USBCx->TXMAXP_L = tempL;
    USBCx->TXMAXP_H = tempH;

    // Flush Tx Ep FIFO
    USBCx->E0CSR_H = 0x1;

    for (ep_idx = 1; ep_idx < 8; ep_idx++)
    {
        USBCx->EINDEX = ep_idx;
        // set FIFO size
        if (speed_idx == 0x02)
        {
            USBCx->TXFIFOSZ = 0x06;
            USBCx->RXFIFOSZ = 0x06;
            tempL = USB_MAX_PACKET_SIZE_LOW;
            tempH = USB_MAX_PACKET_SIZE_HIGH;
        }
        else
        {
            USBCx->TXFIFOSZ = 0x03;
            USBCx->RXFIFOSZ = 0x03;
            tempL = USB_MAX_PACKET_SIZE_LOW_V11;
            tempH = USB_MAX_PACKET_SIZE_HIGH_V11;
        }

        // set fifo offset address
        fifo_addr = ep_idx * 0x200;
        USBCx->TX_fifoadd_L = ((fifo_addr >> 3) & 0x00FF);
        USBCx->TX_fifoadd_H = ((fifo_addr >> 11) & 0x00FF);

        USBCx->RX_fifoadd_L = ((fifo_addr >> 3) & 0x00FF);
        USBCx->RX_fifoadd_H = ((fifo_addr >> 11) & 0x00FF);

        USBCx->TXCSR_L = DEV_TXCSR_CLR_DATA_TOG;
        USBCx->TXCSR_H = 0;

        USBCx->TXMAXP_L = tempL;
        USBCx->TXMAXP_H = tempH;

        // USBCx->RXCSR_L = 0x80;
        USBCx->RXCSR_L = DEV_RXCSR_CLR_DATA_TOG;
        USBCx->RXCSR_H = 0x0;

        USBCx->RXMAXP_L = tempL;
        USBCx->RXMAXP_H = tempH;
        //=================================

        // Flush Tx Ep FIFO
        USBCx->TXCSR_L = DEV_TXCSR_FLUSH_FIFO;

        // Flush Rx Ep FIFO
        USBCx->RXCSR_L = DEV_RXCSR_FLUSH_FIFO;
    }
    USBCx->EINDEX = 0;
}

void USBC_Connect(FT_USBD_Type *USBCx)
{
    USBCx->UCSR |= USB_POWER_SOFT_CONN;
    USBCx->INTRUSB &= ~(USB_INTERRUPT_CONNECT);
}

void USBC_Disconnect(FT_USBD_Type *USBCx)
{
    USBCx->UCSR &= ~USB_POWER_SOFT_CONN;
    USBCx->INTRUSB &= ~(USB_INTERRUPT_DISCON);
}

void USBC_Enbale_Irq_Ep0(FT_USBD_Type *USBCx)
{

    USBCx->INTRTXE_L |= USB_INTERRUPT_EP0;
}

void USBC_Disbale_Irq_Ep0(FT_USBD_Type *USBCx)
{
    USBCx->INTRTXE_L &= ~(USB_INTERRUPT_EP0);
}

void USBC_Init(FT_USBD_Type *USBCx, uint8_t speed_idx)
{
    uint8_t ints_usb;

    // clear all interrupts
    ints_usb = USBCx->INTRUSB;
    // CPM_UsbPhyInit();

    // printf("USBC_Init isr :%x, uscr:%x ise:%x\n", ints_usb, USBCx->UCSR, USBCx->INTRUSBE);
    // CCM->PHYPA |= 0x0e00;	//no need to supply VBUS.
    *(volatile uint16_t *)0x40001006 = (*(volatile uint16_t *)0x40001006) | 0x0E00;
    // VBUS Mode
    //*(volatile uint16_t*)0x40001006 = (*(volatile uint16_t*)0x40001006) & (~0x0E00);

    /* Setup USB register */
    // enable usb common interrupt
    // 0		1		2		3		4		5		6		7 (bit)
    // Susp	Resume	Reset	SOF		Conn	Discon	SessReq	VBusErr
    USBCx->INTRUSBE = USB_INTERRUPT_SUSPEND | USB_INTERRUPT_RESUME | USB_INTERRUPT_RESET | USB_INTERRUPT_SOF |
                      USB_INTERRUPT_CONNECT | USB_INTERRUPT_DISCON;

    // Enable Soft connection
    if (speed_idx == 0x02)
        USBCx->UCSR = USB_POWER_SOFT_CONN | USB_POWER_HS_ENAB | USB_POWER_ENAB_SUSP;
    else
        USBCx->UCSR = USB_POWER_SOFT_CONN | USB_POWER_ENAB_SUSP;
}

void USBC_DeInit(FT_USBD_Type *USBCx, uint8_t speed_idx)
{
    uint8_t ints_usb;
    /* Setup USB register */
    // enable usb common interrupt
    // 0		1		2		3		4		5		6		7 (bit)
    // Susp	Resume	Reset	SOF		Conn	Discon	SessReq	VBusErr
    USBCx->INTRUSBE = USB_INTERRUPT_RESET | USB_INTERRUPT_RESUME;

    // Disable soft connections,
    if (speed_idx == 0x02)
        USBCx->UCSR = USB_POWER_HS_ENAB;
    else
        USBCx->UCSR = 0;

    USBCx->FADDRR = 0;

    // clear all interrupts
    ints_usb = USBCx->INTRUSB;
}

/*************************************************
Function: USBC_EP0SendStall
Description: 设置EPORT0位空闲态。
Input :无
Output: 无
Return: 无
Others: 无
*************************************************/
void USBC_EP0SendStall(FT_USBD_Type *USBCx)
{
    uint8_t csr_l;
    uint8_t saved_idx;

    saved_idx = USBCx->EINDEX;
    USBCx->EINDEX = 0;
    csr_l = USBCx->E0CSR_L;
    // csr_l |= DEV_CSR0_SENDSTALL;
    csr_l |= DEV_CSR0_SENDSTALL | DEV_CSR0_SERVICE_RXPKTRDY;

    USBCx->E0CSR_L = csr_l;
    USBCx->EINDEX = saved_idx;
}

void USBC_EP0ClearSendStall(FT_USBD_Type *USBCx)
{
    uint8_t csr_l;
    uint8_t saved_idx;

    saved_idx = USBCx->EINDEX;
    USBCx->EINDEX = 0;
    csr_l = USBCx->E0CSR_L;
    csr_l &= ~(DEV_CSR0_SENDSTALL);

    USBCx->E0CSR_L = csr_l;
    USBCx->EINDEX = saved_idx;
}

/*************************************************
Function: USBC_EPxSendStall
Description: 设置EPORTx位空闲态。
Input :无
Output: 无
Return: 无
Others: 无
*************************************************/
void USBC_EPxSendStall(FT_USBD_Type *USBCx, uint8_t ep_addr)
{
    uint8_t csr_l;
    uint8_t ep_idx;
    uint8_t saved_idx;
    uint8_t ep_dir_in;

    ep_idx = ep_addr & 0x7F;
    saved_idx = USBCx->EINDEX;
    USBCx->EINDEX = ep_idx;

    ep_dir_in = (ep_addr >> 7) & 0x01;
    if (ep_dir_in == 0x1)
    {
        csr_l = USBCx->TXCSR_L;
        csr_l |= DEV_TXCSR_SEND_STALL;
        USBCx->TXCSR_L = csr_l;
    }
    else
    {
        csr_l = USBCx->RXCSR_L;
        csr_l |= DEV_RXCSR_SEND_STALL;
        USBCx->RXCSR_L = csr_l;
    }
    USBCx->EINDEX = saved_idx;
}

void USBC_EPxClearSendStall(FT_USBD_Type *USBCx, uint8_t ep_addr)
{
    uint8_t csr_l, csr_h;
    uint8_t ep_idx;
    uint8_t saved_idx;
    uint8_t ep_dir_in;

    ep_idx = ep_addr & 0x7F;
    saved_idx = USBCx->EINDEX;

    USBCx->EINDEX = ep_idx;
    ep_dir_in = (ep_addr >> 7) & 0x01;

    // Clear sentstall and restart data toggle.
    if (ep_dir_in == 0x1)
    {
        csr_l = USBCx->TXCSR_L;
        csr_h = USBCx->TXCSR_H;
        if (csr_l & DEV_TXCSR_SENT_SATLL)
        {
            csr_l &= ~(DEV_TXCSR_SEND_STALL);

            if (!(csr_h & DEV_TXCSR_ISO))
            {
                csr_l |= DEV_TXCSR_CLR_DATA_TOG;
            }
        }
        USBCx->TXCSR_L = csr_l;
    }
    else
    {
        csr_l = USBCx->RXCSR_L;
        csr_h = USBCx->RXCSR_H;
        // printf("USBC_EPxClearSendStall Rxcsr %x\n",csr_l);
        if (csr_l & DEV_RXCSR_SENT_STALL)
        {
            csr_l &= ~(DEV_RXCSR_SEND_STALL);
            if (!(csr_h & DEV_RXCSR_ISO))
            {
                csr_l |= DEV_RXCSR_CLR_DATA_TOG;
            }
        }
        USBCx->RXCSR_L = csr_l;
    }
    USBCx->EINDEX = saved_idx;
}
