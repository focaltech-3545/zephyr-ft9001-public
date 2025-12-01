/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>

#include "flash_driver.h"


void gpio_init(void);

#ifdef CONFIG_MOCK

#if 1
int ft_flash_test(void)
{
 
   #if 0
    int j, ret;

    uint8_t send[8] = {0x04, 0xFB, 0x9A, 0x8B, 0x96, 0x69, 0x55, 0xAA};
    uint8_t recv[8] = {0};

    static const struct device *const flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    if(!device_is_ready(flash_dev)){
        printk("flash dev is fail\n");
        return -1;
    }

    printk("flash dev is ok\n");

    printk("\n------start flash test1-------\n");
    /*erase first sector (4K)*/
    flash_erase(flash_dev,APPLICATION_ADDRESS,4096);

    printk("\n------start flash write-------\n"); 
    /*write sample data to flash*/   
    ret=flash_write(flash_dev,APPLICATION_ADDRESS,send,8);
    if (ret)
    {
        printk("\n------flash write failed-------\n");
        return -1;
    }

    printk("\n------start flash read-------\n");
    /*read sample data from flash*/
    flash_read(flash_dev,APPLICATION_ADDRESS,recv,8);

    for (j = 0; j < 8; j++)
    {
        printk("\nread=%x,", recv[j]);
    }

#endif
    return 0;
}
#endif

#if 0

#include <stdio.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "gpio_drv.h"
#include "spi_drv.h"

#define SENSOR_RST_PIN  2
#define SENSOR_INT_PIN  0

static const struct device *gpio_dev5;

static struct gpio_callback gpio_cb_sensor_int;

static volatile uint8_t sensor_int_flag = 0;

/**
 * @brief GPIO40 interrupt handler - Reads GPIO status
 */
void sensor_int_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    //printf("sensor int\n");
    sensor_int_flag = 1;
}

void gpio_init(void)
{
    gpio_dev5 = DEVICE_DT_GET(DT_NODELABEL(eport5));
    if (device_is_ready(gpio_dev5)) {
        printf("eport5 device initialized!\n");
    }   
    
    gpio_pin_configure(gpio_dev5, SENSOR_RST_PIN, GPIO_OUTPUT); // GINT42 OUTPUT
    gpio_pin_configure(gpio_dev5, SENSOR_INT_PIN, GPIO_INPUT | GPIO_PULL_UP); // GINT40 input   
    
    /* Configure GPIO interrupts */
    gpio_pin_interrupt_configure(gpio_dev5, SENSOR_INT_PIN, GPIO_INT_MODE_EDGE | GPIO_INT_TRIG_HIGH); // GINT40 
    gpio_init_callback(&gpio_cb_sensor_int, sensor_int_cb, 1<<SENSOR_INT_PIN); //The third parameter is the mask value of the pin.
    gpio_add_callback(gpio_dev5, &gpio_cb_sensor_int);
    
    gpio_pin_set_raw(gpio_dev5, SENSOR_RST_PIN, 1); 
}
#endif


#endif
