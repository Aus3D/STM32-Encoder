################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

OBJS += \
./startup/startup_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo %cd%
	arm-none-eabi-as -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -I"C:/Users/Chris/workspace/STM32-Encoder/inc" -I"C:/Users/Chris/workspace/STM32-Encoder/CMSIS/core" -I"C:/Users/Chris/workspace/STM32-Encoder/CMSIS/device" -I"C:/Users/Chris/workspace/STM32-Encoder/HAL_Driver/Inc/Legacy" -I"C:/Users/Chris/workspace/STM32-Encoder/HAL_Driver/Inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


