/* Board entry points for Nucleo-H755ZI-Q (CM7). bsp_init() brings the chip fully
 * up but leaves everything idle; bsp_start() arms the Hall capture and the injected
 * ADCs so the 40 kHz current loop goes live. Mirrors the old main.c wiring order.
 *
 * Note: only CM7 is used. CM4 is an unflashed stub, so there is no HSEM boot-sync
 * (same as the bench firmware, where that sequence was compiled out). */
#include "bsp.h"
#include "board.h"

void bsp_init(void)
{
  SCB_EnableICache();   /* D-cache stays off, no MPU — parity with bench firmware */
  HAL_Init();
  bsp_clock_init();

  bsp_gpio_init();
  bsp_adc_init();
  bsp_spi1_init();
  bsp_tim1_init();
  bsp_tim4_init();
  bsp_usart3_init();
  bsp_fdcan1_init();
  bsp_i2c1_init();
}

void bsp_start(void)
{
  HAL_TIMEx_HallSensor_Start_IT(&htim4);   /* Hall edge capture @ priority 5      */
  HAL_ADCEx_InjectedStart(&hadc2);         /* slave ADC2 (phase B), no interrupt  */
  HAL_ADCEx_InjectedStart_IT(&hadc1);      /* master ADC1 drives the 40 kHz ISR   */
}

void Error_Handler(void)
{
  __disable_irq();
  for (;;) {}
}
