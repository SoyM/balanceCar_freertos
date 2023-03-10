cmake_minimum_required(VERSION 3.6)

# adjust project folder name if necessary
set(STM32_PROJ_FOLDER stm32proj)

message(STATUS "Building project: ${PROJECT_NAME}")
set(STM32_CMAKE_DIR "${CMAKE_SOURCE_DIR}")

# # Setup cross-compile toolchain
set(CROSS_COMPILE_PREFIX arm-none-eabi-)
include(${STM32_CMAKE_DIR}/cmake/gcc-arm-none-eabi.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/target-def.cmake)

# # auto-set variables from user input
include(${STM32_CMAKE_DIR}/cmake/stm32-gcc-flags.cmake)

project(stm32proj C CXX ASM)

# set the build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Build type: Debug")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Build type: Release")
endif()

# print summary of configurations
message(STATUS "MCU_FAMILY: ${MCU_FAMILY}")
message(STATUS "MCU_NAME: ${MCU_NAME}")
message(STATUS "Specified C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Linker script: ${LINKER_SCRIPT}")
message(STATUS "Startup code: ${STARTUP_CODE}")
message(STATUS "Use LL library: ${USE_LL_LIB}")
message(STATUS "Use HAL library: ${USE_HAL_LIB}")
message(STATUS "Use Segger SystemView library: ${USE_SYSTEM_VIEW}")

# # version information
set(VERSION_MAJOR 0 CACHE STRING "Project major version number.")
set(VERSION_MINOR 1 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc)

if(USE_SYSTEM_VIEW)
  add_definitions("-DENABLE_SYSTEMVIEW")
endif()

# # add project components
set(ELF_TARGET ${PROJECT_NAME}.elf)
set(PROJ_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Drivers)

# project-specific modules
# add_subdirectory(driver)

# common libraries
# add_subdirectory(${PROJ_LIB_PATH}/CMSIS ${CMAKE_BINARY_DIR}/libraries/CMSIS)
add_library(cmsis INTERFACE)
target_compile_definitions(cmsis INTERFACE -D${MCU_FAMILY})
target_include_directories(cmsis INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Core/Include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include>)

# add_subdirectory(${PROJ_LIB_PATH}/STM32F1xx_HAL_Driver ${CMAKE_BINARY_DIR}/libraries/STM32F1xx_HAL_Driver)
file(GLOB STM32_HAL_SRC "Drivers/${STM32_HAL_DIR_NAME}/Src/*.c")
file(GLOB FREERTOS_LIB "${FREERTOS_ROOT}/*.c")

if(USE_HAL_LIB)
  set(STM32_LIB_SRC ${STM32_HAL_SRC} ${STM32F1_LL_SRC})
else()
  set(STM32_LIB_SRC ${STM32F1_LL_SRC})
endif()

add_library(stm32xxxx_hal ${STM32_LIB_SRC})

if(USE_LL_LIB)
  target_compile_definitions(stm32xxxx_hal PUBLIC -DUSE_FULL_LL_DRIVER)
endif()

if(USE_HAL_LIB)
  target_compile_definitions(stm32xxxx_hal PUBLIC -DUSE_HAL_DRIVER)
endif()

target_link_libraries(stm32xxxx_hal PUBLIC cmsis)
target_include_directories(stm32xxxx_hal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${STM32_HAL_DIR_NAME}/Inc>
  PRIVATE Src)

# FreeRTOS
file(GLOB FREERTOS_LIB "Middlewares/Third_Party/FreeRTOS/Source/*.c")
set(FREERTOS_PORT
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
  ${CMAKE_CURRENT_SOURCE_DIR}/portable/MemMang/heap_4.c
  ${FREERTOS_PROT_PATH}/port.c
)
add_library(freertos_lib ${FREERTOS_LIB} ${FREERTOS_PORT})
target_link_libraries(freertos_lib PUBLIC cmsis)
target_include_directories(freertos_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include>)
target_include_directories(freertos_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS>)
target_include_directories(freertos_lib PUBLIC
  $<BUILD_INTERFACE:${FREERTOS_PROT_PATH}>)

# STM32CubeMX generated source files
file(GLOB CUBEMX_SRC "Core/Src/*.c")
add_executable(${ELF_TARGET} ${CUBEMX_SRC} ${STARTUP_CODE})

target_compile_definitions(${ELF_TARGET} PUBLIC -D${MCU_FAMILY})
target_link_libraries(${ELF_TARGET} PUBLIC cmsis stm32xxxx_hal freertos_lib -lc -lm -lnosys)
target_link_libraries(${ELF_TARGET} PRIVATE -T${LINKER_SCRIPT} ${EXTRA_LINK_FLAGS})
target_include_directories(${ELF_TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc>)

# # add post-build targets
include(${STM32_CMAKE_DIR}/cmake/stm32-gcc-postbuild.cmake)

# include(${CMAKE_CURRENT_SOURCE_DIR}/${STM32_PROJ_FOLDER}/stm32-gcc-jlink.cmake)
FIND_PROGRAM(OPENOCD openocd)
ADD_CUSTOM_TARGET("upload" DEPENDS ${ELF_TARGET} COMMAND ${OPENOCD} -f /usr/share/openocd/scripts/interface/${OPENOCD_INTERFACE}.cfg -f /usr/share/openocd/scripts/target/${OPENOCD_MCU_TYPE}.cfg -c 'program ${ELF_TARGET} verify reset exit')
