// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : def.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef DEF_FOCALTECH_H_
#define DEF_FOCALTECH_H_

/*************Status definition***********************/
#define STATUS_OK                           (int)0x0000
#define STATUS_NULL                         (int)0x0001
#define STATUS_BUSY                         (int)0x0002
#define STATUS_TIMEOUT                      (int)0x0003
#define STATUS_UNKNOW_ERR                   (int)0x0004
#define STATUS_ADDR_ERR                     (int)0x0005
#define STATUS_ID_ERR                       (int)0x0006
    
//UART 
#define STATUS_UART_OVERRUN                 (int)0x1001
#define STATUS_UART_NOISE                   (int)0x1002
#define STATUS_UART_FRAMING_ERR             (int)0x1004
#define STATUS_UART_PARITY_ERR              (int)0x1008
#define STATUS_UART_FIFO_PARITY_ERR         (int)0x1010
#define STATUS_UART_FIFO_FRAMING_ERR        (int)0x1020
#define STATUS_UART_FIFO_NOISE              (int)0x1040
#define STATUS_UART_FIFO_OVERRUN            (int)0x1080
#define STATUS_UART_TIMEOUT                 (int)0x1100
    
//SPI 
#define STATUS_SPI_TXFIFO_TIMEOUT           (int)0x2001
#define STATUS_SPI_TXFIFO_OVERFLOW_ERR      (int)0x2002
#define STATUS_SPI_TXFIFO_UNDERFLOW_ERR     (int)0x2004
#define STATUS_SPI_RXFIFO_TIMEOUT           (int)0x2010
#define STATUS_SPI_RXFIFO_OVERFLOW_ERR      (int)0x2020
#define STATUS_SPI_RXFIFO_UNDERFLOW_ERR     (int)0x2040
#define STATUS_SPI_FRAME_LOSR_ERR           (int)0x2080
    
//SSI 
#define STATUS_SSI_RXFIFO_OVERFLOW_ERR      (int)0x3001
#define STATUS_SSI_RXFIFO_UNDERFLOW_ERR     (int)0x3002
#define STATUS_SSI_TXFIFO_OVERFLOW_ERR      (int)0x3004
#define STATUS_XIP_RXFIFO_OVERFLOW_ERR      (int)0x3008
#define STATUS_SSI_RAW_RXFIFO_OVERFLOW_ERR  (int)0x3010
#define STATUS_SSI_RAW_RXFIFO_UNDERFLOW_ERR (int)0x3020
#define STATUS_SSI_RAW_TXFIFO_OVERFLOW_ERR  (int)0x3040
#define STATUS_XIP_RAW_RXFIFO_OVERFLOW_ERR  (int)0x3080

//I2C
#define STATUS_I2C_MODE_ERR                 (int)0x4001
#define STATUS_I2C_I2CS_AACK_ERR            (int)0x4002
    
//EFLASH
#define STATUS_EFLASH_ERROR                 (int)0x5001

//CPM
#define STATUS_ERR                          (int)0x1FFF

/*************End Status definition***********************/

#endif /* DEF_FOCALTECH_H_ */
