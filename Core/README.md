# Compile stm32 in ubuntu

## Requirements

* ninja
* openocd

## Usage

build

```sh
cmake -DCMAKE_CXX_COMPILER=arm-none-eabi-g++ -DCMAKE_C_COMPILER=arm-none-eabi-gcc -DCMAKE_TOOLCHAIN_FILE=../stm32f103xx.cmake -GNinja ..

ninja clean &&  ninja

ninjia upload 
```

## To hex & bin

```sh
arm-none-eabi-objcopy -O ihex balancecar_freertos.elf balancecar_freertos.hex
arm-none-eabi-objcopy -O binary balancecar_freertos.elf balancecar_freertos.bin
```
