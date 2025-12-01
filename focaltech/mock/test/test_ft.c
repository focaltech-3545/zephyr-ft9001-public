/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "flash_driver.h"

#ifdef CONFIG_MOCK 
int ft_spi_test(void)
{
	
	struct device *const spi_dev =(struct device *const) DEVICE_DT_GET(DT_NODELABEL(spi2));

	struct spi_config spi_cfg = {0};
	int i,j;

	uint8_t send[8]={0x04,0xFB,0x9A,0x8B,0x00,0x01,0x00,0x00};
	uint8_t recv[8]={0};

	
	struct spi_buf bufs[] = {
		{
			.buf = send,
			.len = 6
		}
	};
	struct spi_buf_set tx = {
		.buffers = bufs,
		.count=1
	};


	struct spi_buf bufr[] = {
		{
			.buf = recv,
			.len = 2
		}
	};
	struct spi_buf_set rx = {
		.buffers = bufr,
		.count=1
	};

	spi_cfg.operation = SPI_TRANSFER_MSB| SPI_WORD_SET(8)|SPI_OP_MODE_MASTER |BIT(11);
	spi_cfg.frequency = 4*1000*1000U;

	if (!device_is_ready(spi_dev)) {
		printk("SPI device %s is not ready\n", spi_dev->name);
		return 0;
	}else{
		printk("SPI get success");

	}

	for(i=0;i<3;i++){
		spi_transceive(spi_dev, &spi_cfg, &tx,&rx);
	
		printk("\n-------565------\n");
		for (j=0;j<8;j++){
			printk("\nrx=%x,",recv[j]);
		}
	}

	//spi_write(spi_dev, &spi_cfg, &tx);




	return 1;
}




#endif
