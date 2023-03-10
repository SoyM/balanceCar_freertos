# Target-specific flags
set(MCU_ARCH ARMCM3)
set(MCU_FAMILY STM32F103xB)
set(MCU_NAME STM32F103C8)

set(CPU "-mcpu=cortex-m3")
set(FPU "")
set(FLOAT_ABI "")

set(STM32_HAL_DIR_NAME STM32F1xx_HAL_Driver)
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/MDK-ARM/STM32F103C8TX_FLASH.ld)
set(STARTUP_CODE ${CMAKE_CURRENT_SOURCE_DIR}/portable/Templates/gcc/startup_stm32f103xb.s)
set(FREERTOS_PROT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM3)

option(USE_LL_LIB "Enable LL library" OFF)
option(USE_HAL_LIB "Enable HAL library" ON)
option(USE_SYSTEM_VIEW "Enable Segger SystemView library" OFF)

set(OPENOCD_MCU_TYPE "stm32f1x")
set(OPENOCD_INTERFACE "stlink-v2")
