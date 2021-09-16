# Compile stm32 in ubuntu

## Requirements

* ninja-build
* openocd

## Usage

build

```sh
# 
# ninja
# 
mkdir -p build && cd build && cmake -GNinja .. && ninja clean &&  ninja
# upload
ninjia upload 

# or
# 
# make
# 
mkdir -p build && cd build && cmake .. && make clean &&  make -j
# upload
make upload
```

## 更换MCU

修改 target-def.cmake

## To hex & bin

```sh
arm-none-eabi-objcopy -O ihex balancecar_freertos.elf balancecar_freertos.hex
arm-none-eabi-objcopy -O binary balancecar_freertos.elf balancecar_freertos.bin
```
