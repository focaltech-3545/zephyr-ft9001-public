#ifndef __SPI_DRV_H__
#define __SPI_DRV_H__

#include <stdio.h>

void ft_spi_init(void);

int ft_spi_write(uint8_t *buffer, uint32_t len);

int ft_spi_write_then_read(uint8_t *tx_buffer, uint32_t tx_len, uint8_t *rx_buffer, uint32_t rx_len);

#endif
