# FOC

Field-oriented control firmware for a hall-sensored BLDC motor, built with
**CMake + Ninja + arm-none-eabi-gcc** against ST's **HAL + LL** drivers — no
CubeMX, no CubeIDE, all board bring-up hand-written.

The application (`src/`) is board-independent and talks to hardware only
through `bsp/bsp.h`. Each board is a self-contained directory under `bsp/`,
selected at configure time with `-DBOARD=<name>`. The first (and reference)
board is `nucleo_h755zi_q` — ST Nucleo-H755ZI-Q, Cortex-M7 core only.

## Layout

```
src/main.c              portable entry: bsp_init -> module inits -> bsp_start -> superloop
src/                    portable app (control, hall, drv8316, encoder, can, cli, fault, arm)
bsp/bsp.h               board-agnostic contract: bsp_init(), bsp_start()
bsp/<board>/            everything hardware-specific: clocks, pins, peripherals, IRQs, linker
  board.cmake           CPU flags, defines, sources, HAL source list, linker script
  clock.c periph.c      hand-written bring-up (LL for pins/buses, HAL for handle peripherals)
  board.c               bsp_init()/bsp_start()/Error_Handler
  stm32h7xx_it.c        IRQ handlers -> HAL -> src/ weak-callback overrides
  main.h                shim for src/ (pin symbols, Error_Handler)
cmake/arm-gcc-toolchain.cmake   generic arm-none-eabi setup (no CPU flags here)
drivers/stm32h7/        vendored CMSIS + STM32H7xx HAL/LL (device pack 1.10.7)
```

## Build

```sh
cmake --preset debug          # -Og -g3
cmake --build --preset debug
# → build/debug/foc_mcu.{elf,hex,bin} + foc_mcu.map, with a memory-usage report
```

`release` (`-O2`) is the other preset. Override the board with
`cmake --preset debug -DBOARD=<name>`.

The toolchain auto-detects the GCC bundled with STM32CubeIDE under `/opt/st`.
Point it elsewhere with `-DARM_TOOLCHAIN_DIR=/path/to/gcc/bin` or the
`ARM_TOOLCHAIN_DIR` environment variable.

## Flash

Hardware steps are left to you — the build never flashes automatically.

```sh
cmake --build --preset debug --target flash   # STM32_Programmer_CLI over SWD, then reset
```

Override the programmer path with `-DSTM32_PROGRAMMER_CLI=/path/to/STM32_Programmer_CLI`.

## Adding a board

1. `mkdir bsp/<name>/` and add a `board.cmake` defining `BOARD_CPU_FLAGS`,
   `BOARD_DEFINES`, `BOARD_SOURCES`, `BOARD_LDSCRIPT`, `BOARD_LINK_FLAGS`,
   `BOARD_HAL_INCLUDES`, `BOARD_HAL_SOURCES` (see `nucleo_h755zi_q` as the template).
2. Implement `bsp.h` (`bsp_init`, `bsp_start`) plus the IRQ handlers, a `main.h`
   shim (the pin symbols / `Error_Handler` the src/ code expects), startup file
   and linker script for the part.
3. If the MCU family is new, drop its CMSIS + HAL tree under `drivers/<family>/`
   and point `BOARD_HAL_*` at it.
4. `cmake --preset debug -DBOARD=<name>`.

Nothing in `src/` or the top-level `CMakeLists.txt` changes.

## Port caveat (honest scope)

The `bsp/` boundary abstracts *bring-up and startup*, not the full hardware
surface. The copied application code still calls ST HAL functions and touches
registers directly (`htim1`, `hadc1`, `TIM1->CCR1`, `ADC1->JDR1`, …). Retargeting
to a non-STM32H7 part therefore means providing the same HAL/handle surface, not
just a new `bsp/`. Within the STM32H7 family the split is clean; across families
it is best-effort by design (the app is carried over unmodified).

## Nucleo-H755ZI-Q notes

- Only the Cortex-M7 core is built and flashed. The Cortex-M4 is left as an
  unflashed stub; there is no HSEM dual-core boot-sync (matching the bench setup).
- I-cache on, D-cache off, no MPU — parity with the validated bench firmware.
- 400 MHz SYSCLK / 200 MHz timer kernels; TIM1 40 kHz center-aligned PWM,
  TIM4 hall interface, ADC1/2 injected-simultaneous sampling phase A/B.
- `-u _printf_float` is mandatory: newlib-nano drops `%f` support otherwise and
  the CLI/telemetry float prints go silently blank.
