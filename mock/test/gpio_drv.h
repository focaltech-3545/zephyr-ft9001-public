#ifndef __GPIO_DRV_H__
#define __GPIO_DRV_H__

void gpio_init(void);

int ft93xx_hw_reset(void);

int get_sensor_int(void);

void clear_sensor_int(void);


#endif
