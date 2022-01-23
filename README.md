A LoRa node LMIC example with low power support on STM32.

The example is tested with a RFM95W LoRa module and https://www.thethingsnetwork.org.

# Instructions
 1. git clone --recurse-submodules https://github.com/fk0815/lora-node-example.git
 2. cd lora-node-example
 3. make -C libopencm3 # (Only needed once)
 4. modify TX_TIMEOUT, APPEUI, DEVEUI and DEVKEY in lora-node.c to match to the node network settings
 4. make -C lora-node-stm32f103 # build for STM32F103C8
 5. make -C lora-node-stm32l431 # build for STM32L431CB

If you have an older git, or got ahead of yourself and skipped the ```--recurse-submodules```
you can fix things by running ```git submodule update --init``` (This is only needed once)

# Directories
* libopencm3 contains https://github.com/libopencm3/libopencm3 ARM Coretx-M suppor library.
* common contains debug helper and the hal code with low power support.
* lora-node-stm32f103 contains the example files for STM32F103C8 (as used on Bluepill boards).
* lora-node-stm32l431 contains the example files for STM32L431CB.
* arduino-lmic contains the https://github.com/mcci-catena/arduino-lmic code.

# Hardware Setup
RFM95W pins | MCU pins | Remark
--- | --- | ---
RESET | PA3 |
NSS | PA4 |
SCK | PA5 |
MISO | PA6 |
MOSI | PA7 |
DIO0 | PA0  | on STM32F103B8
DIO1 | PA1 | on STM32F103B8
DIO0 | PB0 | on STM32L431CB
DIO1 | PB1 | on STM32L431CB

The MCU needs to have a 32768Hz crystal for the LSE clock. The timing depends on that. It
might be needed to relax the timing by configure a clock error by using LMIC_setClockError()
compensate an inaccurate clock.

# Debug Support
The project is prepared to support debug prints via semihosting on a debugger. To enable that
set the DEBUG=semihost environment variable while compiling the code for the dedicated MCU.

# Arduino-lmic Patches
The arduino-lmic requires some patches to allow to compile it free of errors and warnings on
a C compiler. Also the HAL API is changed to use ostime_t for all timing related functions.
With that a 64bit integer is used for ostime_t to avoid timer overflows.
