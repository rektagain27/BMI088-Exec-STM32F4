################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32-BMI088-main/Src/Accel.c \
../Drivers/STM32-BMI088-main/Src/Gyro.c \
../Drivers/STM32-BMI088-main/Src/IMU.c \
../Drivers/STM32-BMI088-main/Src/Vectors.c 

OBJS += \
./Drivers/STM32-BMI088-main/Src/Accel.o \
./Drivers/STM32-BMI088-main/Src/Gyro.o \
./Drivers/STM32-BMI088-main/Src/IMU.o \
./Drivers/STM32-BMI088-main/Src/Vectors.o 

C_DEPS += \
./Drivers/STM32-BMI088-main/Src/Accel.d \
./Drivers/STM32-BMI088-main/Src/Gyro.d \
./Drivers/STM32-BMI088-main/Src/IMU.d \
./Drivers/STM32-BMI088-main/Src/Vectors.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32-BMI088-main/Src/%.o: ../Drivers/STM32-BMI088-main/Src/%.c Drivers/STM32-BMI088-main/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/BMI088 Final/Drivers/STM32-BMI088-main" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32-2d-BMI088-2d-main-2f-Src

clean-Drivers-2f-STM32-2d-BMI088-2d-main-2f-Src:
	-$(RM) ./Drivers/STM32-BMI088-main/Src/Accel.d ./Drivers/STM32-BMI088-main/Src/Accel.o ./Drivers/STM32-BMI088-main/Src/Gyro.d ./Drivers/STM32-BMI088-main/Src/Gyro.o ./Drivers/STM32-BMI088-main/Src/IMU.d ./Drivers/STM32-BMI088-main/Src/IMU.o ./Drivers/STM32-BMI088-main/Src/Vectors.d ./Drivers/STM32-BMI088-main/Src/Vectors.o

.PHONY: clean-Drivers-2f-STM32-2d-BMI088-2d-main-2f-Src

