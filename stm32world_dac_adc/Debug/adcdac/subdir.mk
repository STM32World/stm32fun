################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../adcdac/ledtask.c \
../adcdac/printmutex.c \
../adcdac/statustask.c \
../adcdac/ticktask.c 

OBJS += \
./adcdac/ledtask.o \
./adcdac/printmutex.o \
./adcdac/statustask.o \
./adcdac/ticktask.o 

C_DEPS += \
./adcdac/ledtask.d \
./adcdac/printmutex.d \
./adcdac/statustask.d \
./adcdac/ticktask.d 


# Each subdirectory must supply rules for building sources it contributes
adcdac/%.o adcdac/%.su adcdac/%.cyclo: ../adcdac/%.c adcdac/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/lth/src/stm32fun/stm32world_dac_adc/adcdac" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-adcdac

clean-adcdac:
	-$(RM) ./adcdac/ledtask.cyclo ./adcdac/ledtask.d ./adcdac/ledtask.o ./adcdac/ledtask.su ./adcdac/printmutex.cyclo ./adcdac/printmutex.d ./adcdac/printmutex.o ./adcdac/printmutex.su ./adcdac/statustask.cyclo ./adcdac/statustask.d ./adcdac/statustask.o ./adcdac/statustask.su ./adcdac/ticktask.cyclo ./adcdac/ticktask.d ./adcdac/ticktask.o ./adcdac/ticktask.su

.PHONY: clean-adcdac

