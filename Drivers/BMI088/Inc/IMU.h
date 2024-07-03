#ifndef __BMI088_IMU 
#define __BMI088_IMU 

#include <BMI088/Inc/Accel.h>
#include <BMI088/Inc/Gyro.h>
#include "main.h"


void IMU_INIT(SPI_HandleTypeDef* spiHandler);

void IMU_SETUP_FOR_LOGGING();
void IMU_ENABLE_ALL();
int IMU_READY();

#endif