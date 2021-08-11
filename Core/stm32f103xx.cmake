# This variable is used later to set a C/C++ macro which tells CubeF4 which µC to use. ST
# drivers, and other code is bloated with all sorts of macros, #defines and #ifdefs.
SET (DEVICE "STM32F103xB")

# This is a variable which is later used here and in the CMakeLists.txt. It simply tells
# where to find the SDK (CubeF4). Please change it accordingly if you have other 
# version of CubeF4 installed.
# SET (CUBE_ROOT "$ENV{HOME}/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.4")

# Startup code and linker script - more on it later.
SET (STARTUP_CODE "../Core/Startup/startup_stm32f103c8tx.s")
SET (LINKER_SCRIPT "../../STM32F103C8TX_FLASH.ld")

# Magic settings. Without it CMake tries to run test programs on the host platform, which
# fails of course.
SET (CMAKE_SYSTEM_NAME Generic)
SET (CMAKE_SYSTEM_PROCESSOR arm)

# -mcpu tells which CPU to target obviously. -fdata-sections -ffunction-sections Tells GCC to.
# get rid of unused code in the output binary. -Wall produces verbose warnings.
SET(CMAKE_C_FLAGS "-mcpu=cortex-m3 -std=gnu99 -O2 -fdata-sections -ffunction-sections -Wall" CACHE INTERNAL "c compiler flags")

# Flags for g++ are used only when compliing C++ sources (*.cc, *.cpp etc). -std=c++17 Turns
# on all the C++17 goodies, -fno-rtti -fno-exceptions turns off rtti and exceptions.
SET(CMAKE_CXX_FLAGS "-mcpu=cortex-m3 -std=c++17 -fno-rtti -fno-exceptions -Wall -fdata-sections -ffunction-sections -MD -Wall" CACHE INTERNAL "cxx compiler flags")

# Those flags gets passed into the linker which is run by the GCC at he end of the process..
# -T tells the linker which LD script to use, -specs=nosys.specs sets the specs which most 
# notably tells the compiler to use libnosys.a which contains all the syscalls like _sbrk,
# _exit and much more. They are more like an interface between our program and operating system /
# bare metal system we are running it on. You can use rdimon.specs instead or write syscalls 
# yourself which for bare-metal isn't difficult. --gc-sections strips out unused code from 
#binaries I think.
SET (CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_SCRIPT} -specs=nosys.specs -Wl,--gc-sections" CACHE INTERNAL "exe link flags")

# Some directories in the GCC tree.
INCLUDE_DIRECTORIES(${SUPPORT_FILES})
LINK_DIRECTORIES(${SUPPORT_FILES})

# Macro I wrote about in the first line.
ADD_DEFINITIONS(-D${DEVICE})
ADD_DEFINITIONS (-DUSE_HAL_DRIVER)

# Random include paths for CubeF4 peripheral drivers and CMSIS.

# INCLUDE_DIRECTORIES("/usr/lib/arm-none-eabi/include/")
INCLUDE_DIRECTORIES("Inc/")
INCLUDE_DIRECTORIES("../Drivers/STM32F1xx_HAL_Driver/Inc/")
INCLUDE_DIRECTORIES("../Drivers/STM32F1xx_HAL_Driver/Inc/legacy/")
INCLUDE_DIRECTORIES("../Drivers/CMSIS/Device/ST/STM32F1xx/Include/")
INCLUDE_DIRECTORIES("../Drivers/CMSIS/Include/")
INCLUDE_DIRECTORIES("../Middlewares/Third_Party/FreeRTOS/Source/include/")
INCLUDE_DIRECTORIES("../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/")
INCLUDE_DIRECTORIES("../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM3/")

