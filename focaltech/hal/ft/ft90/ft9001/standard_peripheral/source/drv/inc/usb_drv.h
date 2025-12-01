

#ifndef __USB_DRV__
#define __USB_DRV__

#include "usb_reg.h"

#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U

//bulk transfer packet size 
#define		USB_MAX_PACKET_SIZE							512
#define		USB_MAX_PACKET_SIZE_LOW						0x00
#define		USB_MAX_PACKET_SIZE_HIGH					0x02
#define		USB_MAX_PACKET_SIZE_V11						64
#define		USB_MAX_PACKET_SIZE_LOW_V11					0x40
#define		USB_MAX_PACKET_SIZE_HIGH_V11				0x00

#define USB_MAX_PACKET_SIZE_EP0             64

int USBC_EpxOpen(FT_USBD_Type* USBCx, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type, int speed_idx);
int USBC_EpxReset(FT_USBD_Type* USBCx, uint8_t ep_addr);
int USBC_EpxClose(FT_USBD_Type* USBCx, uint8_t ep_addr);
void CPM_UsbPhyInit(uint8_t src_type);
void DRV_CPM_UsbPhyDeinit(void);
void USBC_Enbale_Irq_Ep0(FT_USBD_Type* USBCx);
void USBC_Disbale_Irq_Ep0(FT_USBD_Type* USBCx);
void USBC_Init(FT_USBD_Type* USBCx, uint8_t speed_idx);
void USBC_DeInit(FT_USBD_Type* USBCx, uint8_t speed_idx);
void USBC_Connect(FT_USBD_Type* USBCx);
void USBC_Disconnect(FT_USBD_Type* USBCx);
void USBC_BusReset(FT_USBD_Type* USBCx, uint8_t speed_idx);
void USBC_EP0SendStall(FT_USBD_Type* USBCx);
void USBC_EPxSendStall(FT_USBD_Type* USBCx, uint8_t ep_addr);
void USBC_EP0ClearSendStall(FT_USBD_Type* USBCx);
void USBC_EPxClearSendStall(FT_USBD_Type* USBCx, uint8_t ep_addr);



#endif //__USB_DRV__