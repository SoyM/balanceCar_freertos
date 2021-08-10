CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT (balancecar_freertos)

# Startup code is written by ST in assembly, so without this statement there are errors.
ENABLE_LANGUAGE (ASM-ATT)

# Resonator used in this project. Stm32F4-DISCO uses 8MHz crystal. I left this definition here
# in the CMakeLists.txt rather than the toolchain file, because it's project dependent, not
# "platform" dependent, where by platform I mean STM32F1.
# ADD_DEFINITIONS (-DHSE_VALUE=8000000)

SET (FREERTOS_ROOT "../Middlewares/Third_Party/FreeRTOS/Source")
SET (STM32F1xx_HAL_ROOT "../Drivers/STM32F1xx_HAL_Driver/Src")


# All the sources goes here. Adding headers isn't obligatory, but since QtCreator treats CMakeLists.txt as
# its "project configuration" it simply makes header files appear in the source tree pane.
ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf
 "../Src/main.c"
 "../Src/stm32f1xx_hal_msp.c"
 "../Src/stm32f1xx_it.c"
 "../Src/system_stm32f1xx.c"

 "../Src/MPU6050.c"
 "../Src/inv_mpu_dmp_motion_driver.c"
 "../Src/inv_mpu.c"
 "../Src/filter.c"
 "../Src/layerCompati_i2c.c"
 "../Src/libmotot.c"
 "../Src/gpio.c"
 "../Src/freertos.c"
 "../Src/adc.c"
 "../Src/i2c.c"
 "../Src/tim.c"
 "../Src/usart.c"
 "../Src/stm32f1xx_hal_timebase_tim.c"
)

# Workaround : splitting C and C++ code helps QtCreator parse header files correctly. Without it, QtCreator
# sometimes treats C++ as C and vice versa. EDIT : this comment was written when in the ADD_EXECUTABLE C++
# files were present.
add_library ("stm" STATIC
 "${STARTUP_CODE}"

 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_cortex.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_gpio.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_rcc.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_tim.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_i2c.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_adc.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_rcc_ex.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_tim_ex.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_tim_ex.c"
 "${STM32F1xx_HAL_ROOT}/stm32f1xx_hal_uart.c"

 "${FREERTOS_ROOT}/CMSIS_RTOS/cmsis_os.c"
 "${FREERTOS_ROOT}/portable/MemMang/heap_4.c"
 "${FREERTOS_ROOT}/portable/RVDS/ARM_CM3/port.c"
 "${FREERTOS_ROOT}/croutine.c"
 "${FREERTOS_ROOT}/event_groups.c"
 "${FREERTOS_ROOT}/list.c"
 "${FREERTOS_ROOT}/queue.c"
 "${FREERTOS_ROOT}/stream_buffer.c"
 "${FREERTOS_ROOT}/tasks.c"
 "${FREERTOS_ROOT}/timers.c"
)



# This links both pieces together.
TARGET_LINK_LIBRARIES (${CMAKE_PROJECT_NAME}.elf -Wl,--whole-archive stm -Wl,--no-whole-archive)

FIND_PROGRAM (OPENOCD openocd)
ADD_CUSTOM_TARGET("upload" DEPENDS ${CMAKE_PROJECT_NAME}.elf COMMAND ${OPENOCD} -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c 'program ${CMAKE_PROJECT_NAME}.elf verify reset exit')
