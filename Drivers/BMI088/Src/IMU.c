
#include <BMI088/Inc/Accel.h>
#include <BMI088/Inc/Gyro.h>
#include <BMI088/Inc/IMU.h>
#include "stm32f4xx_hal.h"
extern SPI_HandleTypeDef hspi1;




void SPI1_Init(void)
{
    // Configure the SPI1 peripheral
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        // Initialization error
        Error_Handler();
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (spiHandle->Instance == SPI1)
    {
        // Enable SPI1 clock
        __HAL_RCC_SPI1_CLK_ENABLE();

        // Enable GPIO clocks
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /** SPI1 GPIO Configuration
         * PA5  ------> SPI1_SCK
         * PA6  ------> SPI1_MISO
         * PA7  ------> SPI1_MOSI
         */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;  // Set the alternate function

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}



void IMU_INIT(SPI_HandleTypeDef* spiHandle){
    ACCEL_INIT(spiHandle);
    GYRO_INIT(spiHandle);
}

void IMU_SETUP_FOR_LOGGING(){
    ACCEL_GOOD_SETTINGS();
    GYRO_GOOD_SETTINGS();
}

void IMU_ENABLE_ALL(){
    ACCEL_WRITE_PWR_ACTIVATE();
    ACCEL_WRITE_ACCEL_ENABLE();
    GYRO_SET_POWERMODE(GYRO_PWR_NORMAL);
    HAL_Delay(100);
}

int IMU_READY(){
    int ready = 0;
    if(!ACCEL_SELF_TEST()){
        ready = -1;
    }
    if(!GYRO_SELF_TEST()){
        ready -= 2;
    }
    return ready == 0 ? 1 : ready;
}
