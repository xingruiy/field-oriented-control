/* Interrupt service routines. The processor fault handlers spin forever; the
 * peripheral handlers just forward to the HAL, which dispatches the weak
 * callbacks overridden in src/ (e.g. HAL_ADCEx_InjectedConvCpltCallback). */
#include "main.h"
#include "stm32h7xx_it.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;

void NMI_Handler(void)        { for (;;) {} }
void HardFault_Handler(void)  { for (;;) {} }
void MemManage_Handler(void)  { for (;;) {} }
void BusFault_Handler(void)   { for (;;) {} }
void UsageFault_Handler(void) { for (;;) {} }
void SVC_Handler(void)        {}
void DebugMon_Handler(void)   {}
void PendSV_Handler(void)     {}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* ADC1 and ADC2 share one vector; master first so the 40 kHz loop reads both JDRs */
void ADC_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
}

/* PC6 nFAULT falling edge → HAL_GPIO_EXTI_Callback (fault.c) */
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(nFault_Pin);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}
