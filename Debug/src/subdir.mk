################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/stm32f0xx_it.c \
../src/syscalls.c \
../src/system_stm32f0xx.c \
../src/tm_stm32_gpio.c 

CPP_SRCS += \
../src/encoder.cpp \
../src/main.cpp \
../src/softi2c.cpp \
../src/spi.cpp \
../src/timing.cpp \
../src/ws2812.cpp 

OBJS += \
./src/encoder.o \
./src/main.o \
./src/softi2c.o \
./src/spi.o \
./src/stm32f0xx_it.o \
./src/syscalls.o \
./src/system_stm32f0xx.o \
./src/timing.o \
./src/tm_stm32_gpio.o \
./src/ws2812.o 

C_DEPS += \
./src/stm32f0xx_it.d \
./src/syscalls.d \
./src/system_stm32f0xx.d \
./src/tm_stm32_gpio.d 

CPP_DEPS += \
./src/encoder.d \
./src/main.d \
./src/softi2c.d \
./src/spi.d \
./src/timing.d \
./src/ws2812.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo %cd%
	arm-none-eabi-g++ -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F030F4Px -DSTM32F0 -DSTM32 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030x6 -I"C:/Users/Chris/workspace/STM32-Encoder/inc" -I"C:/Users/Chris/workspace/STM32-Encoder/CMSIS/core" -I"C:/Users/Chris/workspace/STM32-Encoder/CMSIS/device" -I"C:/Users/Chris/workspace/STM32-Encoder/HAL_Driver/Inc/Legacy" -I"C:/Users/Chris/workspace/STM32-Encoder/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32F030F4Px -DSTM32F0 -DSTM32 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030x6 -I"C:/Users/Chris/workspace/STM32-Encoder/inc" -I"C:/Users/Chris/workspace/STM32-Encoder/CMSIS/core" -I"C:/Users/Chris/workspace/STM32-Encoder/CMSIS/device" -I"C:/Users/Chris/workspace/STM32-Encoder/HAL_Driver/Inc/Legacy" -I"C:/Users/Chris/workspace/STM32-Encoder/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


