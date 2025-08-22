################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../trng/trng.c 

OBJS += \
./trng/trng.o 

C_DEPS += \
./trng/trng.d 


# Each subdirectory must supply rules for building sources it contributes
trng/%.o trng/%.su trng/%.cyclo: ../trng/%.c trng/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/lth/src/stm32fun/stm32world_trng/trng" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-trng

clean-trng:
	-$(RM) ./trng/trng.cyclo ./trng/trng.d ./trng/trng.o ./trng/trng.su

.PHONY: clean-trng

