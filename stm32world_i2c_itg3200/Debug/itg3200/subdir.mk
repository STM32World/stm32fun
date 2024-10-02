################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../itg3200/itg3200.c 

OBJS += \
./itg3200/itg3200.o 

C_DEPS += \
./itg3200/itg3200.d 


# Each subdirectory must supply rules for building sources it contributes
itg3200/%.o itg3200/%.su itg3200/%.cyclo: ../itg3200/%.c itg3200/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/lth/src/stm32fun/stm32world_i2c_itg3200/itg3200" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-itg3200

clean-itg3200:
	-$(RM) ./itg3200/itg3200.cyclo ./itg3200/itg3200.d ./itg3200/itg3200.o ./itg3200/itg3200.su

.PHONY: clean-itg3200

