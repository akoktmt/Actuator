################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PID\ Control/PID.c 

OBJS += \
./PID\ Control/PID.o 

C_DEPS += \
./PID\ Control/PID.d 


# Each subdirectory must supply rules for building sources it contributes
PID\ Control/PID.o: ../PID\ Control/PID.c PID\ Control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../EncoderVelocity -I../Motor -I../Motor_Driver -I"../PID Control" -I../BNO -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../CAN_Handle -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"PID Control/PID.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-PID-20-Control

clean-PID-20-Control:
	-$(RM) ./PID\ Control/PID.cyclo ./PID\ Control/PID.d ./PID\ Control/PID.o ./PID\ Control/PID.su

.PHONY: clean-PID-20-Control

