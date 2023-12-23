################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Motor_Driver/Motor_Driver.c 

OBJS += \
./Motor_Driver/Motor_Driver.o 

C_DEPS += \
./Motor_Driver/Motor_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
Motor_Driver/%.o Motor_Driver/%.su Motor_Driver/%.cyclo: ../Motor_Driver/%.c Motor_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../EncoderVelocity -I../Motor -I../Motor_Driver -I"../PID Control" -I../BNO -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../CAN_Handle -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Motor_Driver

clean-Motor_Driver:
	-$(RM) ./Motor_Driver/Motor_Driver.cyclo ./Motor_Driver/Motor_Driver.d ./Motor_Driver/Motor_Driver.o ./Motor_Driver/Motor_Driver.su

.PHONY: clean-Motor_Driver

