# Board: ST Nucleo-H755ZI-Q, Cortex-M7 core only (CM4 left unflashed).
# Everything the top-level CMakeLists needs to know about this board.

set(BOARD_CPU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)

set(BOARD_DEFINES
    CORE_CM7
    STM32H755xx
    USE_HAL_DRIVER
    USE_FULL_LL_DRIVER
    USE_PWR_DIRECT_SMPS_SUPPLY)

set(BOARD_LDSCRIPT   ${CMAKE_CURRENT_LIST_DIR}/STM32H755ZITX_FLASH.ld)
set(BOARD_LINK_FLAGS --specs=nano.specs -u _printf_float)  # CLI prints %f

set(BOARD_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/board.c
    ${CMAKE_CURRENT_LIST_DIR}/clock.c
    ${CMAKE_CURRENT_LIST_DIR}/periph.c
    ${CMAKE_CURRENT_LIST_DIR}/stm32h7xx_it.c
    ${CMAKE_CURRENT_LIST_DIR}/syscalls.c
    ${CMAKE_CURRENT_LIST_DIR}/sysmem.c
    ${CMAKE_CURRENT_LIST_DIR}/system_stm32h7xx.c
    ${CMAKE_CURRENT_LIST_DIR}/startup_stm32h755zitx.s)

set(BOARD_HAL_DIR ${CMAKE_SOURCE_DIR}/drivers/stm32h7)
set(BOARD_HAL_INCLUDES
    ${BOARD_HAL_DIR}/STM32H7xx_HAL_Driver/Inc
    ${BOARD_HAL_DIR}/CMSIS/Device/ST/STM32H7xx/Include
    ${BOARD_HAL_DIR}/CMSIS/Include)

# Only the HAL modules enabled in stm32h7xx_hal_conf.h
set(_hal_src ${BOARD_HAL_DIR}/STM32H7xx_HAL_Driver/Src)
set(BOARD_HAL_SOURCES
    ${_hal_src}/stm32h7xx_hal.c
    ${_hal_src}/stm32h7xx_hal_cortex.c
    ${_hal_src}/stm32h7xx_hal_rcc.c
    ${_hal_src}/stm32h7xx_hal_rcc_ex.c
    ${_hal_src}/stm32h7xx_hal_pwr.c
    ${_hal_src}/stm32h7xx_hal_pwr_ex.c
    ${_hal_src}/stm32h7xx_hal_gpio.c
    ${_hal_src}/stm32h7xx_hal_exti.c
    ${_hal_src}/stm32h7xx_hal_flash.c
    ${_hal_src}/stm32h7xx_hal_flash_ex.c
    ${_hal_src}/stm32h7xx_hal_dma.c
    ${_hal_src}/stm32h7xx_hal_dma_ex.c
    ${_hal_src}/stm32h7xx_hal_mdma.c
    ${_hal_src}/stm32h7xx_hal_hsem.c
    ${_hal_src}/stm32h7xx_hal_adc.c
    ${_hal_src}/stm32h7xx_hal_adc_ex.c
    ${_hal_src}/stm32h7xx_hal_tim.c
    ${_hal_src}/stm32h7xx_hal_tim_ex.c
    ${_hal_src}/stm32h7xx_hal_uart.c
    ${_hal_src}/stm32h7xx_hal_uart_ex.c
    ${_hal_src}/stm32h7xx_hal_spi.c
    ${_hal_src}/stm32h7xx_hal_spi_ex.c
    ${_hal_src}/stm32h7xx_hal_i2c.c
    ${_hal_src}/stm32h7xx_hal_i2c_ex.c
    ${_hal_src}/stm32h7xx_hal_fdcan.c)
