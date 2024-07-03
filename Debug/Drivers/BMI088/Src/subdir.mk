################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BMI088/Src/Accel.c \
../Drivers/BMI088/Src/Gyro.c \
../Drivers/BMI088/Src/IMU.c \
../Drivers/BMI088/Src/Vectors.c 

OBJS += \
./Drivers/BMI088/Src/Accel.o \
./Drivers/BMI088/Src/Gyro.o \
./Drivers/BMI088/Src/IMU.o \
./Drivers/BMI088/Src/Vectors.o 

C_DEPS += \
./Drivers/BMI088/Src/Accel.d \
./Drivers/BMI088/Src/Gyro.d \
./Drivers/BMI088/Src/IMU.d \
./Drivers/BMI088/Src/Vectors.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BMI088/Src/%.o: ../Drivers/BMI088/Src/%.c Drivers/BMI088/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/rivxb/STM32CubeIDE/workspace_1.8.0/BMI088 Final/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BMI088-2f-Src

clean-Drivers-2f-BMI088-2f-Src:
	-$(RM) ./Drivers/BMI088/Src/Accel.d ./Drivers/BMI088/Src/Accel.o ./Drivers/BMI088/Src/Gyro.d ./Drivers/BMI088/Src/Gyro.o ./Drivers/BMI088/Src/IMU.d ./Drivers/BMI088/Src/IMU.o ./Drivers/BMI088/Src/Vectors.d ./Drivers/BMI088/Src/Vectors.o

.PHONY: clean-Drivers-2f-BMI088-2f-Src

