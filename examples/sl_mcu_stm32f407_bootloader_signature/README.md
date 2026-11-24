# Secure Bootloader - DFU, Flash and Signature Check

Part of the STM32World **Secure Firmware Update** series on the STM32F407 (Streamline MCU blade).

This project tests all the moving parts on their own: a signed firmware image is uploaded over USB DFU (TinyUSB), stored on an external W25Q128 SPI flash and its ECDSA signature is verified with MbedTLS.  It is not relocated and does not use the bootloader.

* Wiki: [Secure Firmware Upgrade Bootloader and Application](https://stm32world.com/wiki/Secure_Firmware_Upgrade_Bootloader_and_Application)
* Video: [STM32 Secure Bootloader: DFU & Signature Verification | Bootloader #1](https://www.youtube.com/watch?v=z6RehAgbSnE)

Other projects in the series:

* [sl_mcu_stm32f407_bootloader_signed_bootloader](../sl_mcu_stm32f407_bootloader_signed_bootloader) - the bootloader
* [sl_mcu_stm32f407_bootloader_signed_app](../sl_mcu_stm32f407_bootloader_signed_app) - the relocated application

**Warning:** this code is **not** meant to be used verbatim in final products.  It illustrates the principles of a secure firmware update and is meant as inspiration for your own implementation.  The private key in this folder is public and only meant for the example.

## How to use

### Generate key

`openssl ecparam -name prime256v1 -genkey -noout -out private_key.pem`

### Generate public key

`openssl ec -in private_key.pem -pubout -out public_key.pem`

### Sign firmware

`./package_firmware.sh Debug/sl_mcu_stm32f407_bootloader_signature.bin`

### Upload

`dfu-util -D app_signed.bin`
