# Generic arm-none-eabi-gcc toolchain. CPU flags come from the board's board.cmake,
# not from here, so one toolchain file serves every board.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain location: -DARM_TOOLCHAIN_DIR=... > env ARM_TOOLCHAIN_DIR > CubeIDE bundle > PATH
set(ARM_TOOLCHAIN_DIR "" CACHE PATH "Directory containing arm-none-eabi-gcc (empty = auto)")
if(NOT ARM_TOOLCHAIN_DIR AND DEFINED ENV{ARM_TOOLCHAIN_DIR})
  set(ARM_TOOLCHAIN_DIR "$ENV{ARM_TOOLCHAIN_DIR}")
endif()
if(NOT ARM_TOOLCHAIN_DIR)
  file(GLOB _cubeide_gcc "/opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin")
  if(_cubeide_gcc)
    list(SORT _cubeide_gcc)
    list(GET _cubeide_gcc -1 ARM_TOOLCHAIN_DIR)
  endif()
endif()

if(ARM_TOOLCHAIN_DIR)
  set(_cross "${ARM_TOOLCHAIN_DIR}/arm-none-eabi-")
else()
  set(_cross "arm-none-eabi-")
endif()

set(CMAKE_C_COMPILER   "${_cross}gcc")
set(CMAKE_ASM_COMPILER "${_cross}gcc")
set(CMAKE_OBJCOPY      "${_cross}objcopy" CACHE FILEPATH "objcopy")
set(CMAKE_SIZE         "${_cross}size"    CACHE FILEPATH "size")

# No OS to link against: test the compiler with a static lib, not an executable
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
