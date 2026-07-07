/* Internal prototypes + handle externs for this board's bsp files. */
#ifndef BOARD_H
#define BOARD_H

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;

void bsp_clock_init(void);
void bsp_gpio_init(void);
void bsp_adc_init(void);
void bsp_spi1_init(void);
void bsp_tim1_init(void);
void bsp_tim4_init(void);
void bsp_usart3_init(void);
void bsp_fdcan1_init(void);
void bsp_i2c1_init(void);

#endif
