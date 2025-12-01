// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : spi_drv.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef SPI_DRV_H_
#define SPI_DRV_H_

#include "def_ft.h"
#include "spi_reg.h"
#include "memmap.h"
//#include "main.h"


void SPI_Enable(SPI_TypeDef *SPIx);
void SPI_Disable(SPI_TypeDef *SPIx);

void SPI_CSLow(SPI_TypeDef *SPIx);
void SPI_CSHigh(SPI_TypeDef *SPIx);

void SPI_IrqEnable(SPI_TypeDef *SPIx);
void SPI_IrqDisable(SPI_TypeDef *SPIx);

void SPI_IrqEnableRx(SPI_TypeDef *SPIx);
void SPI_IrqDisableRx(SPI_TypeDef *SPIx);
void SPI_IrqEnableTx(SPI_TypeDef *SPIx);
void SPI_IrqDisableTx(SPI_TypeDef *SPIx);

void SPI_IrqEnableRxErrors(SPI_TypeDef *SPIx);
void SPI_IrqDisableRxErrors(SPI_TypeDef *SPIx);
void SPI_IrqEnableTxErrors(SPI_TypeDef *SPIx);
void SPI_IrqDisableTxErrors(SPI_TypeDef *SPIx);


void SPI_RxReadAll(SPI_TypeDef *SPIx, uint8_t mode);
void SPI_TxTranserDone(SPI_TypeDef *SPIx);
int SPI_GetStatusErr(SPI_TypeDef *SPIx);

#endif /* SPI_DRV_H_ */
