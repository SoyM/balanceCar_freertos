# Target-specific flags
set(MCU_ARCH ARMCM3)
set(MCU_FAMILY STM32F103xB)
set(MCU_NAME STM32F103C8)

set(CPU "-mcpu=cortex-m3")
set(FPU "")
set(FLOAT_ABI "")

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/MDK-ARM/STM32F103C8TX_FLASH.ld)
SET (STARTUP_CODE ${CMAKE_CURRENT_SOURCE_DIR}/Core/Startup/startup_stm32f103c8tx.s)

option(USE_LL_LIB "Enable LL library" OFF)
option(USE_HAL_LIB "Enable HAL library" ON)

option(USE_SYSTEM_VIEW "Enable Segger SystemView library" OFF)