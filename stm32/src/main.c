/* Portable FOC entry point — board-independent. All hardware bring-up hides
 * behind bsp_init()/bsp_start(); the module wiring order matches the bench
 * firmware exactly. To retarget, add a bsp/<board>/ implementing bsp.h. */
#include "bsp.h"

#include "drv8316/drv8316.h"
#include "hall/hall.h"
#include "control/foc.h"
#include "fault/fault.h"
#include "cli/cli.h"
#include "can/can.h"
#include "encoder/encoder.h"
#include "arm/arm_pos.h"

int main(void)
{
  bsp_init();          /* caches, clocks, every peripheral — nothing running yet */

  drv8316_init();      /* configure gate driver over SPI                         */
  hall_init();         /* sample initial Hall state, seed PLL                    */
  foc_init();          /* start TIM1 PWM, calibrate ADC offsets                  */

  bsp_start();         /* arm Hall capture + injected ADCs → 40 kHz loop live    */

  fault_init();        /* sample nFAULT, arm supervision                         */
  cli_init();          /* start CLI on USART3                                    */
  can_init();          /* FDCAN1 filter + start (control/telemetry)             */
  encoder_init();      /* TMAG5273 probe+config (I2C1); ok to fail if absent    */
  arm_pos_init();      /* TMAG arm position PID supervisor                       */

  for (;;)
  {
    fault_poll();      /* service nFAULT EXTI + periodic soft-fault poll         */
    cli_process();     /* dispatch any completed CLI command line               */
    can_poll();        /* dispatch CAN commands + broadcast telemetry           */
    encoder_poll();    /* paced TMAG5273 angle read (blocking I2C)              */
    arm_pos_poll();    /* TMAG arm position PID, outputs speed reference         */
  }
}
