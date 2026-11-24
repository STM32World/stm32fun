# Secure Bootloader - The Bootloader

Part of the STM32World **Secure Firmware Update** series on the STM32F407 (Streamline MCU blade).

This is the bootloader.  It occupies the first 32 kB of flash (sectors 0-1) and does nothing but copy an already verified firmware image from the external W25Q128 SPI flash into the application area at 0x0800 8000.  It does not check the signature itself; the application does that before it sets the boot flag.

* Wiki: [Secure Firmware Upgrade Bootloader and Application](https://stm32world.com/wiki/Secure_Firmware_Upgrade_Bootloader_and_Application)
* Video: [STM32 Secure Bootloader: The Bootloader | Bootloader #2](https://www.youtube.com/watch?v=nagbMzDy2FA)

Other projects in the series:

* [sl_mcu_stm32f407_bootloader_signature](../sl_mcu_stm32f407_bootloader_signature) - DFU, flash and signature check
* [sl_mcu_stm32f407_bootloader_signed_app](../sl_mcu_stm32f407_bootloader_signed_app) - the relocated application

**Warning:** this code is **not** meant to be used verbatim in final products.  It illustrates the principles of a secure firmware update and is meant as inspiration for your own implementation.

## How it works

At reset, before `HAL_Init()`, the bootloader checks the boot flag in the top 8 bytes of RAM (`_bflag` in the linker script):

* Flag is not `0xDEADBEEF`: jump straight to the application at 0x0800 8000.
* Flag is `0xDEADBEEF`: clear it, read the 128 byte header from the SPI flash, erase the application sectors, copy the firmware, erase the magic marker and reset.

## How to use

Flash this project with STM32CubeIDE first, then flash [the application](../sl_mcu_stm32f407_bootloader_signed_app).  After that, updates go over USB DFU.
