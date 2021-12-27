################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/005ButtonInterrupt.c 

OBJS += \
./Src/005ButtonInterrupt.o 

C_DEPS += \
./Src/005ButtonInterrupt.d 


# Each subdirectory must supply rules for building sources it contributes
Src/005ButtonInterrupt.o: ../Src/005ButtonInterrupt.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F429ZITx -DSTM32F4 -c -I../Inc -I"/root/STM32CubeIDE/uart_spi_i2c/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/005ButtonInterrupt.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

