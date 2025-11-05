// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : spi_drv.c
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//#include "debug.h"
#include "spi_drv.h"

void SPI_Enable(SPI_TypeDef *SPIx)
{
	SPIx->SPICR1 |= SPICR1_SPE_MASK;
}

void SPI_Disable(SPI_TypeDef *SPIx)
{
	SPIx->SPICR1 &= ~SPICR1_SPE_MASK;
}

void SPI_IrqEnable(SPI_TypeDef *SPIx)
{
	SPIx->SPICR1 |= SPICR1_SPIE_MASK;
}

void SPI_IrqDisable(SPI_TypeDef *SPIx)
{
	SPIx->SPICR1 &= ~SPICR1_SPIE_MASK;
}

void SPI_IrqEnableTxErrors(SPI_TypeDef *SPIx)
{
	SPIx->SPIICR |= SPIICR_FLOSTIE_MASK | SPIICR_MODFIE_MASK;
	SPIx->SPITXFCR |= SPITXFCR_TXFOVIE_MASK | SPITXFCR_TXFUDIE_MASK;
}

void SPI_IrqDisableTxErrors(SPI_TypeDef *SPIx)
{
	SPIx->SPIICR &= ~(SPIICR_FLOSTIE_MASK | SPIICR_MODFIE_MASK);
	SPIx->SPITXFCR &= ~(SPITXFCR_TXFOVIE_MASK | SPITXFCR_TXFUDIE_MASK);
}

void SPI_IrqEnableRxErrors(SPI_TypeDef *SPIx)
{
	SPIx->SPIICR |= SPIICR_FLOSTIE_MASK | SPIICR_MODFIE_MASK;
	SPIx->SPIRXFCR |= SPIRXFCR_RXFOVIE_MASK | SPIRXFCR_RXFUDIE_MASK;
}

void SPI_IrqDisableRxErrors(SPI_TypeDef *SPIx)
{
	SPIx->SPIICR &= ~(SPIICR_FLOSTIE_MASK | SPIICR_MODFIE_MASK);
	SPIx->SPIRXFCR &= ~(SPIRXFCR_RXFOVIE_MASK | SPIRXFCR_RXFUDIE_MASK);
}

void SPI_IrqEnableRx(SPI_TypeDef *SPIx)
{
	SPIx->SPIRXFCR |= SPIRXFCR_RXFSTHIE_MASK;
}

void SPI_IrqEnableTx(SPI_TypeDef *SPIx)
{
	SPIx->SPITXFCR |= SPITXFCR_TXFSTHIE_MASK;
}

void SPI_IrqDisableRx(SPI_TypeDef *SPIx)
{
	SPIx->SPIRXFCR &= ~SPIRXFCR_RXFSTHIE_MASK;
}

void SPI_IrqDisableTx(SPI_TypeDef *SPIx)
{
	SPIx->SPITXFCR &= ~SPITXFCR_TXFSTHIE_MASK;
}

int SPI_GetStatusErr(SPI_TypeDef *SPIx)
{
	int ret = 0;
	uint16_t status;
	status = SPIx->SPISRHW;
	if (status & 0x6610)
	{
		if((SPIx->SPIRXFCR & SPIRXFCR_RXFOVIE_MASK) && (status & SPISR_RXFOVF_MASK))
		{
			ret |= SPISR_RXFOVF_MASK;
		}
		if((SPIx->SPIRXFCR & SPIRXFCR_RXFUDIE_MASK) && (status & SPISR_RXFUDF_MASK))
		{
			ret |= SPISR_RXFUDF_MASK;
		}
		if((SPIx->SPITXFCR & SPITXFCR_TXFOVIE_MASK) && (status & SPISR_TXFOVF_MASK))
		{
			ret |= SPISR_TXFOVF_MASK;
		}
		if((SPIx->SPITXFCR & SPITXFCR_TXFUDIE_MASK) && (status & SPISR_TXFUDF_MASK))
		{
			ret |= SPISR_TXFUDF_MASK;
		}
		if(status & SPISR_MODF_MASK)
		{
			ret |= SPISR_MODF_MASK;
		}
	}
	SPIx->SPISRHW = (status & 0x6610) | 0x40;
	return ret;
}

void SPI_CSLow(SPI_TypeDef *SPIx)
{
	SPIx->SPIPORT = SPIx->SPIPORT & (~SPIPORT_SS_MASK) ;
	return;
}

void SPI_CSHigh(SPI_TypeDef *SPIx)
{
	SPIx->SPIPORT = SPIx->SPIPORT | SPIPORT_SS_MASK;
	return;
}

void SPI_RxReadAll(SPI_TypeDef *SPIx, uint8_t mode)
{
	uint16_t rx_frame;
	while(!(SPIx->SPISRHW & SPISR_RXFEMP_MASK))
	{
		if(mode <= 8)
		{
			rx_frame = SPIx->SPIDR;
		}
		else
		{
			rx_frame = SPIx->SPIDRHW;
		}
	}
}

void SPI_TxTranserDone(SPI_TypeDef *SPIx)
{
	while(!(SPIx->SPISRHW & SPISR_TXFEMP_MASK));
}


