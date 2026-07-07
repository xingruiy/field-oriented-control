cmake_minimum_required(VERSION 3.22)
project(foc-mcu C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Board selection: each board is a self-contained dir under bsp/ with a board.cmake
set(BOARD nucleo_h755zi_q CACHE STRING "Target board (directory name under bsp/)")
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/bsp/${BOARD}/board.cmake)
  message(FATAL_ERROR "Unknown BOARD '${BOARD}': bsp/${BOARD}/board.cmake not found")
endif()
include(${CMAKE_SOURCE_DIR}/bsp/${BOARD}/board.cmake)

# CPU/FPU flags apply to every object (drivers, bsp, app, startup) so ABIs match
add_compile_options(${BOARD_CPU_FLAGS})
add_link_options(${BOARD_CPU_FLAGS})

# Vendored ST HAL/LL drivers — separate lib so its warnings stay silenced
add_library(hal_drivers STATIC ${BOARD_HAL_SOURCES})
target_include_directories(hal_drivers PUBLIC
    ${BOARD_HAL_INCLUDES}
    ${CMAKE_SOURCE_DIR}/bsp/${BOARD})        # stm32h7xx_hal_conf.h
target_compile_definitions(hal_drivers PUBLIC ${BOARD_DEFINES})
target_compile_options(hal_drivers PRIVATE -w)

# Firmware: portable app (src/) + board support
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/*.c)
add_executable(foc_mcu
    ${APP_SOURCES}
    ${BOARD_SOURCES})
set_target_properties(foc_mcu PROPERTIES SUFFIX .elf)
target_include_directories(foc_mcu PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/bsp
    ${CMAKE_SOURCE_DIR}/bsp/${BOARD})
target_compile_options(foc_mcu PRIVATE
    -Wall -Wextra
    -ffunction-sections -fdata-sections)
target_link_libraries(foc_mcu PRIVATE hal_drivers)
target_link_options(foc_mcu PRIVATE
    -T${BOARD_LDSCRIPT}
    ${BOARD_LINK_FLAGS}
    -Wl,--gc-sections
    -Wl,-Map=$<TARGET_FILE_DIR:foc_mcu>/foc_mcu.map
    -Wl,--print-memory-usage)

add_custom_command(TARGET foc_mcu POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:foc_mcu> $<TARGET_FILE_DIR:foc_mcu>/foc_mcu.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:foc_mcu> $<TARGET_FILE_DIR:foc_mcu>/foc_mcu.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:foc_mcu>)

# ninja flash — writes over SWD, then resets. Never run automatically.
file(GLOB _prog "/opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin/STM32_Programmer_CLI")
if(_prog)
  list(GET _prog -1 _prog_default)
else()
  set(_prog_default STM32_Programmer_CLI)
endif()
set(STM32_PROGRAMMER_CLI ${_prog_default} CACHE FILEPATH "Path to STM32_Programmer_CLI")
add_custom_target(flash
    COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD mode=UR -w $<TARGET_FILE_DIR:foc_mcu>/foc_mcu.hex -v -rst
    DEPENDS foc_mcu
    USES_TERMINAL)
