# Secure Bootloader - Relocated Application

Part of the STM32World **Secure Firmware Update** series on the STM32F407 (Streamline MCU blade).

This is the application.  It is relocated to 0x0800 8000 to make room for the bootloader, and it presents a USB DFU interface (TinyUSB) all the time.  An uploaded image is stored on the external W25Q128 SPI flash and its ECDSA signature is verified with MbedTLS.  If it is valid, the application sets the boot flag and resets into [the bootloader](../sl_mcu_stm32f407_bootloader_signed_bootloader), which installs it.

* Wiki: [Secure Firmware Upgrade Bootloader and Application](https://stm32world.com/wiki/Secure_Firmware_Upgrade_Bootloader_and_Application)
* Video: [STM32 Secure Bootloader: Relocated Application | Bootloader #3](https://www.youtube.com/watch?v=pagFJKGz61E)

Other projects in the series:

* [sl_mcu_stm32f407_bootloader_signature](../sl_mcu_stm32f407_bootloader_signature) - DFU, flash and signature check
* [sl_mcu_stm32f407_bootloader_signed_bootloader](../sl_mcu_stm32f407_bootloader_signed_bootloader) - the bootloader

**Warning:** this code is **not** meant to be used verbatim in final products.  It illustrates the principles of a secure firmware update and is meant as inspiration for your own implementation.  The private key in this folder is public and only meant for the example.

## How to use

### Generate key

`openssl ecparam -name prime256v1 -genkey -noout -out private_key.pem`

### Generate public key

`openssl ec -in private_key.pem -pubout -out public_key.pem`

### Sign firmware

`./package_firmware.sh Debug/sl_mcu_stm32f407_bootloader_signed_app.bin`

### Upload

`dfu-util -D app_signed.bin`
