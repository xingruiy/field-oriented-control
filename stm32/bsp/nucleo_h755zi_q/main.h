/* Board shim for the portable src/ code: HAL types, Error_Handler, pin symbols. */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

void Error_Handler(void);

#define nFault_Pin GPIO_PIN_6
#define nFault_GPIO_Port GPIOC
#define nFault_EXTI_IRQn EXTI9_5_IRQn
#define nSCS_Pin GPIO_PIN_8
#define nSCS_GPIO_Port GPIOC

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
