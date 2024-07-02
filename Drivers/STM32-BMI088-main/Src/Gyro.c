#include "Gyro.h"
#include <math.h>
#include <stdlib.h>

// GPIO connectivity
#define GYRO_CS_PORT GPIOC
#define GYRO_CS_PIN GPIO_PIN_12
#define GPIO_HIGH GPIO_PIN_SET
#define GPIO_LOW GPIO_PIN_RESET

// Address space
#define ADDR_CHIP_ID 0x00
#define ADDR_RATE_X_LSB 0x02
#define ADDR_RATE_X_MSB 0x03
#define ADDR_RATE_Y_LSB 0x04
#define ADDR_RATE_Y_MSB 0x05
#define ADDR_RATE_Z_LSB 0x06
#define ADDR_RATE_Z_MSB 0x07
#define ADDR_INT_STAT_1 0x0A
#define ADDR_FIFO_STATUS 0x0E
#define ADDR_RANGE 0x0F
#define ADDR_BANDWIDTH 0x10
#define ADDR_LPM1 0x11
#define ADDR_SOFTRESET 0x14
#define ADDR_INT_CTRL 0x15
#define ADDR_INT3_INT4_IO_CONF 0x16
#define ADDR_INT3_INT4_IO_MAP 0x18
#define ADDR_FIFO_WM_EN 0x1E
#define ADDR_FIFO_EXT_INT_S 0x34
#define ADDR_GYRO_SELF_TEST 0x3C
#define ADDR_FIFO_CONFIG_0 0x3D
#define ADDR_FIFO_CONFIG_1 0x3E
#define ADDR_FIFO_DATA 0x3F

// Conversion factors
#define MAX_2K_TO_RADS ((M_PI * 2000.0 / 180.0) / 0x7FFF)
#define MAX_1K_TO_RADS ((M_PI * 1000.0 / 180.0) / 0x7FFF)
#define MAX_500_TO_RADS ((M_PI * 500.0 / 180.0) / 0x7FFF)
#define MAX_250_TO_RADS ((M_PI * 250.0 / 180.0) / 0x7FFF)
#define MAX_125_TO_RADS ((M_PI * 125.0 / 180.0) / 0x7FFF)

// FIFO
#define FIFO_FRAME_SIZE 6
#define FIFO_MAX_FRAMES 100
#define FIFO_MAX_BYTES (FIFO_FRAME_SIZE * FIFO_MAX_FRAMES)

// SPI read/write
#define SPI_READ 0x80
#define SPI_WRITE 0x00

// Static variables
static SPI_HandleTypeDef* gyro_spi_handle;
static uint8_t gyro_range;
static uint8_t gyro_odr;

// Function prototypes
static void gyro_select(void);
static void gyro_unselect(void);
static void gyro_read_address(uint8_t address, uint8_t* buffer, int length);
static void gyro_write_address(uint8_t address, uint8_t data);
static Vector3 gyro_parse_raw_data(uint8_t* raw_data);

// Public functions

void GYRO_INIT(SPI_HandleTypeDef* spi_handle) {
    gyro_unselect();
    gyro_spi_handle = spi_handle;
    GYRO_RELOAD_SETTINGS();
}

void GYRO_APPLY_SETTINGS(void) {
    GYRO_SET_RANGE(GYRO_RANGE_DPS_1K);
    GYRO_SET_OUTPUT_DATA_RATE(GYRO_ODR_1K__BW_116);
    GYRO_SET_FIFO_MODE(GYRO_FIFO_STREAM);
    GYRO_RELOAD_SETTINGS();
}

uint8_t GYRO_READ_ID(void) {
    uint8_t id = 0;
    gyro_select();
    gyro_read_address(ADDR_CHIP_ID, &id, 1);
    gyro_unselect();
    return id;
}

Vector3 GYRO_READ_RATES(void) {
    uint8_t raw_values[6];
    gyro_select();
    gyro_read_address(ADDR_RATE_X_LSB, raw_values, 6);
    gyro_unselect();
    return gyro_parse_raw_data(raw_values);
}

void GYRO_RELOAD_SETTINGS(void) {
    uint8_t raw_values[2];
    gyro_select();
    gyro_read_address(ADDR_RANGE, raw_values, 2);
    gyro_unselect();
    gyro_range = raw_values[0];
    gyro_odr = raw_values[1] & 0x7F; // Ignore bit 7
}

uint8_t GYRO_SELF_TEST(void) {
    uint8_t result = 0;
    uint8_t count = 0;
    gyro_select();
    gyro_write_address(ADDR_GYRO_SELF_TEST, 0x01); // Set bit 0
    gyro_unselect();
    while (!(result & 0x02) && count < 10) { // Bit 1 must be 1 to exit
        HAL_Delay(100);
        gyro_select();
        gyro_read_address(ADDR_GYRO_SELF_TEST, &result, 1);
        gyro_unselect();
        count++;
    }
    // Test completed
    if (result & 0x02) {
        return !(result & 0x04); // Return NOT failed (bit 2 is 1 if test failed)
    } else { // Count reached 10 and test did not complete
        return 0;
    }
}

GyroDataBuffer GYRO_READ_FIFO(void) {
    uint8_t raw_buffer[FIFO_MAX_BYTES] = {0};
    int i = 0;
    GyroDataBuffer output;
    output.length = 0;
    output.array = malloc(sizeof(Vector3) * FIFO_MAX_FRAMES);

    gyro_select();
    gyro_read_address(ADDR_FIFO_DATA, raw_buffer, FIFO_MAX_BYTES);
    gyro_unselect();

    // Process FIFO data
    while (i < FIFO_MAX_BYTES && !(raw_buffer[i] == 0 && raw_buffer[i + 1] == 128 &&
                                   raw_buffer[i + 2] == 0 && raw_buffer[i + 3] == 128 &&
                                   raw_buffer[i + 4] == 0 && raw_buffer[i + 5] == 128)) {
        output.array[output.length] = gyro_parse_raw_data(raw_buffer + i);
        output.length++;
        i += 6;
    }

    output.array = realloc(output.array, output.length * sizeof(Vector3));
    return output;
}

void GYRO_SET_POWERMODE(uint8_t power_mode) {
    gyro_select();
    gyro_write_address(ADDR_LPM1, power_mode);
    gyro_unselect();
}

void GYRO_SET_RANGE(uint8_t range) {
    gyro_select();
    gyro_range = range;
    gyro_write_address(ADDR_RANGE, range);
    gyro_unselect();
}

void GYRO_SET_OUTPUT_DATA_RATE(uint8_t odr) {
    gyro_select();
    gyro_write_address(ADDR_BANDWIDTH, odr);
    gyro_unselect();
}

void GYRO_SET_FIFO_MODE(uint8_t fifo_mode) {
    gyro_select();
    gyro_write_address(ADDR_FIFO_CONFIG_1, fifo_mode);
    gyro_unselect();
}

// Private functions

static void gyro_select(void) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_LOW);
}

static void gyro_unselect(void) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_HIGH);
}

static void gyro_read_address(uint8_t address, uint8_t* buffer, int length) {
    uint8_t addr = SPI_READ | address;
    HAL_SPI_Transmit(gyro_spi_handle, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(gyro_spi_handle, buffer, length, HAL_MAX_DELAY);
}

static void gyro_write_address(uint8_t address, uint8_t data) {
    uint8_t message[] = {SPI_WRITE | address, data};
    HAL_SPI_Transmit(gyro_spi_handle, message, 2, HAL_MAX_DELAY);
}

static Vector3 gyro_parse_raw_data(uint8_t* raw_data) {
    Vector3 output;
    output.x = (int16_t)(raw_data[1] << 8 | raw_data[0]);
    output.y = (int16_t)(raw_data[3] << 8 | raw_data[2]);
    output.z = (int16_t)(raw_data[5] << 8 | raw_data[4]);

    // Convert units to rad/s based on current range
    switch (gyro_range) {
        case GYRO_RANGE_DPS_2K:
            V_MUL(output, MAX_2K_TO_RADS);
            break;
        case GYRO_RANGE_DPS_1K:
            V_MUL(output, MAX_1K_TO_RADS);
            break;
        case GYRO_RANGE_DPS_500:
            V_MUL(output, MAX_500_TO_RADS);
            break;
        case GYRO_RANGE_DPS_250:
            V_MUL(output, MAX_250_TO_RADS);
            break;
        case GYRO_RANGE_DPS_125:
            V_MUL(output, MAX_125_TO_RADS);
            break;
        default:
            output.x = 0;
            output.y = 0;
            output.z = 0;
            break;
    }

    return output;
}
