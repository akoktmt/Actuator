################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BNO055/Calib.c \
../BNO055/bno055.c 

OBJS += \
./BNO055/Calib.o \
./BNO055/bno055.o 

C_DEPS += \
./BNO055/Calib.d \
./BNO055/bno055.d 


# Each subdirectory must supply rules for building sources it contributes
BNO055/%.o BNO055/%.su BNO055/%.cyclo: ../BNO055/%.c BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../EncoderVelocity -I../Motor -I../Motor_Driver -I"../PID Control" -I../BNO -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BNO055

clean-BNO055:
	-$(RM) ./BNO055/Calib.cyclo ./BNO055/Calib.d ./BNO055/Calib.o ./BNO055/Calib.su ./BNO055/bno055.cyclo ./BNO055/bno055.d ./BNO055/bno055.o ./BNO055/bno055.su

.PHONY: clean-BNO055

