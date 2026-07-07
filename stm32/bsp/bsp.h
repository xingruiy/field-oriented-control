/* Board-agnostic BSP contract. Each board under bsp/<name>/ implements these. */
#ifndef BSP_H
#define BSP_H

void bsp_init(void);   /* caches, clocks, all peripheral config — nothing running yet */
void bsp_start(void);  /* start hall capture + injected ADCs; 40 kHz loop goes live   */

#endif
