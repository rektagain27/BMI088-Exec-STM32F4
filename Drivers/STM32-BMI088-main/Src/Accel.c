#include "Accel.h"
#include <stdlib.h>
#include <math.h>

// GPIO connectivity
#define ACCEL_CS_PORT GPIOC // Define according to your hardware configuration
#define ACCEL_CS_PIN GPIO_PIN_12 // Define according to your hardware configuration
#define GPIO_HIGH GPIO_PIN_SET
#define GPIO_LOW GPIO_PIN_RESET

// Address space
#define ADDR_CHIP_ID 0x00
#define ADDR_ERR_REG 0x02
#define ADDR_STATUS 0x03
#define ADDR_ACC_X_LSB 0x12
#define ADDR_ACC_X_MSB 0x13
#define ADDR_ACC_Y_LSB 0x14
#define ADDR_ACC_Y_MSB 0x15
#define ADDR_ACC_Z_LSB 0x16
#define ADDR_ACC_Z_MSB 0x17
#define ADDR_SENSORTIME_0 0x18
#define ADDR_SENSORTIME_1 0x19
#define ADDR_SENSORTIME_2 0x1A
#define ADDR_INT_STAT_1 0x1D
#define ADDR_TEMP_MSB 0x22
#define ADDR_TEMP_LSB 0x23
#define ADDR_FIFO_LENGTH_0 0x24
#define ADDR_FIFO_LENGTH_1 0x25
#define ADDR_FIFO_DATA 0x26
#define ADDR_ACC_CONF 0x40
#define ADDR_ACC_RANGE 0x41
#define ADDR_FIFO_DOWNS 0x45
#define ADDR_FIFO_WTM_0 0x46
#define ADDR_FIFO_WTM_1 0x47
#define ADDR_FIFO_CONFIG_0 0x48
#define ADDR_FIFO_CONFIG_1 0x49
#define ADDR_INT1_IO_CTRL 0x53
#define ADDR_INT2_IO_CTRL 0x54
#define ADDR_INT_MAP_DATA 0x58
#define ADDR_ACC_SELF_TEST 0x6D
#define ADDR_ACC_PWR_CONF 0x7C
#define ADDR_ACC_PWR_CTRL 0x7D
#define ADDR_ACC_SOFTRESET 0x7E

// Sensor attributes
#define FIFO_MAX_BUFFER_BYTES 1024
#define FIFO_DATA_FRAME_SIZE_BYTES 7

// SPI Commands
#define SPI_READ 0x80
#define SPI_WRITE 0x00

// Constants for calculations
#define GRAV 9.80665f
#define ACCEL_PWR_ACTIVE 0x00
#define ACCEL_PWR_SUSPEND 0x03
#define ACCEL_ACCEL_ENABLED 0x04
#define ACCEL_ACCEL_DISABLED 0x00
#define ACCEL_FIFO_ENABLED 0x40
#define ACCEL_FIFO_DOWNSAMP_NONE 0x00
#define ACCEL_FIFO_MODE_STREAM 0x80

// Self-test configuration
#define ACCEL_SELF_TEST_POSITIVE 0x0D
#define ACCEL_SELF_TEST_NEGATIVE 0x09
#define ACCEL_SELF_TEST_RESET 0x00
#define SELF_TEST_DELAY_MS 55
#define SELF_TEST_TEMP_OFFSET 23.0f
#define SELF_TEST_TEMP_SCALE 0.125f

// Accelerometer ranges
#define ACCEL_RANGE_3G 0x00
#define ACCEL_RANGE_6G 0x01
#define ACCEL_RANGE_12G 0x02
#define ACCEL_RANGE_24G 0x03

// Accelerometer configurations
#define ACCEL_OSR_NORMAL 0x02
#define ACCEL_ODR_400 0x0A
#define ACCEL_ODR_1600 0x0E

// FIFO Frame Headers
#define FIFO_FRAME_H_DATA 0x84
#define FIFO_FRAME_H_SKIP 0x40
#define FIFO_FRAME_H_SENSORTIME 0x44
#define FIFO_FRAME_H_CONFIG 0x48
#define FIFO_FRAME_H_DROP 0x50
#define FIFO_FRAME_H_END 0x80

// Static variables
static SPI_HandleTypeDef* accel_spi_handle;
static uint8_t accel_max_range_bits;
static double accel_max_range_real;
static uint8_t accel_bandwidth_param;
static uint8_t accel_output_data_rate;

// Function prototypes
static void accel_select(void);
static void accel_unselect(void);
static void accel_read_address(uint8_t address, uint8_t* buffer, int length);
static void accel_write_address(uint8_t address, uint8_t data);
static void accel_set_range_memory(uint8_t range);
static Vector3 accel_parse_raw_data(uint8_t* raw_data);

// Public functions

void ACCEL_INIT(SPI_HandleTypeDef* spi_handler) {
    accel_unselect();
    accel_spi_handle = spi_handler;
    ACCEL_READ_ID(); // Dummy read to ensure communication
    ACCEL_RELOAD_SETTINGS();
}

void ACCEL_GOOD_SETTINGS(void) {
    ACCEL_SET_RANGE(ACCEL_RANGE_24G);
    ACCEL_SET_CONFIG(ACCEL_OSR_NORMAL, ACCEL_ODR_400);
    ACCEL_WRITE_FIFO_DOWNSAMP(ACCEL_FIFO_DOWNSAMP_NONE);
    ACCEL_WRITE_FIFO_MODE(ACCEL_FIFO_MODE_STREAM);
    ACCEL_WRITE_FIFO_ENABLED(ACCEL_FIFO_ENABLED);
}

uint8_t ACCEL_SELF_TEST(void) {
    Vector3 positive;
    Vector3 negative;
    Vector3 difference;
    uint8_t is_good = 1;

    // Setup
    ACCEL_SET_RANGE(ACCEL_RANGE_24G);
    ACCEL_SET_CONFIG(ACCEL_OSR_NORMAL, ACCEL_ODR_1600);
    HAL_Delay(5);

    // Positive test
    accel_select();
    accel_write_address(ADDR_ACC_SELF_TEST, ACCEL_SELF_TEST_POSITIVE);
    accel_unselect();
    HAL_Delay(SELF_TEST_DELAY_MS);
    positive = ACCEL_READ_ACCELERATION();

    // Negative test
    accel_select();
    accel_write_address(ADDR_ACC_SELF_TEST, ACCEL_SELF_TEST_NEGATIVE);
    accel_unselect();
    HAL_Delay(SELF_TEST_DELAY_MS);
    negative = ACCEL_READ_ACCELERATION();

    // Reset
    accel_select();
    accel_write_address(ADDR_ACC_SELF_TEST, ACCEL_SELF_TEST_RESET);
    accel_unselect();
    HAL_Delay(SELF_TEST_DELAY_MS);

    // Calculate results
    difference = vector_subtract(positive, negative);
    is_good = (difference.x >= GRAV) &&
              (difference.y >= GRAV) &&
              (difference.z >= GRAV / 2);

    return is_good;
}

void ACCEL_RELOAD_SETTINGS(void) {
    uint8_t raw_data[2];
    accel_select();
    accel_read_address(ADDR_ACC_CONF, raw_data, 2);
    accel_unselect();

    accel_bandwidth_param = raw_data[0] >> 4; // First 4 bits
    accel_output_data_rate = raw_data[0] & 0x0F; // Last 4 bits

    raw_data[1] = raw_data[1] & 0x03; // Last 2 bits
    accel_set_range_memory(raw_data[1]);
}

uint8_t ACCEL_READ_ID(void) {
    uint8_t id = 0;
    accel_select();
    accel_read_address(ADDR_CHIP_ID, &id, 1);
    accel_unselect();
    return id;
}

Vector3 ACCEL_READ_ACCELERATION(void) {
    uint8_t raw_values[6];
    accel_select();
    accel_read_address(ADDR_ACC_X_LSB, raw_values, 6);
    accel_unselect();
    return accel_parse_raw_data(raw_values);
}

float ACCEL_READ_TEMPERATURE(void) {
    uint8_t raw_values[2];
    int16_t raw_value;
    accel_select();
    accel_read_address(ADDR_TEMP_MSB, raw_values, 2);
    accel_unselect();
    raw_value = (raw_values[0] << 3) | (raw_values[1] >> 5);
    raw_value = raw_value > 1023 ? raw_value - 2048 : raw_value; // Convert to signed 11-bit
    return SELF_TEST_TEMP_OFFSET + (((float)raw_value) * SELF_TEST_TEMP_SCALE);
}

uint32_t ACCEL_READ_SENSORTIME(void) {
    uint8_t values[3];
    uint32_t sensor_time;
    accel_select();
    accel_read_address(ADDR_SENSORTIME_0, values, 3);
    accel_unselect();
    sensor_time = values[0] | (values[1] << 8) | (values[2] << 16);
    return sensor_time;
}

AccelError ACCEL_READ_ERROR(void) {
    uint8_t error_value;
    AccelError error;
    accel_select();
    accel_read_address(ADDR_ERR_REG, &error_value, 1);
    accel_unselect();

    error.isFatal = error_value & 0x01;
    error.errorCode = (error_value & 0x1C) >> 2;
    return error;
}

uint8_t ACCEL_READ_PWR_MODE(void) {
    uint8_t power_mode;
    accel_select();
    accel_read_address(ADDR_ACC_PWR_CONF, &power_mode, 1);
    accel_unselect();
    return power_mode;
}

uint8_t ACCEL_READ_ACCEL_ENABLED(void) {
    uint8_t enabled;
    accel_select();
    accel_read_address(ADDR_ACC_PWR_CTRL, &enabled, 1);
    accel_unselect();
    return enabled;
}

void ACCEL_SET_CONFIG(uint8_t oversampling_rate, uint8_t output_data_rate) {
    uint8_t config = (oversampling_rate << 4) | output_data_rate;
    accel_select();
    accel_write_address(ADDR_ACC_CONF, config);
    accel_unselect();
    accel_bandwidth_param = oversampling_rate;
    accel_output_data_rate = output_data_rate;
}

void ACCEL_SET_RANGE(uint8_t range) {
    accel_select();
    accel_write_address(ADDR_ACC_RANGE, range);
    accel_unselect();
    accel_set_range_memory(range);
}

void ACCEL_WRITE_PWR_MODE(uint8_t power_mode) {
    accel_select();
    accel_write_address(ADDR_ACC_PWR_CONF, power_mode);
    accel_unselect();
}

void ACCEL_WRITE_ACCEL_ENABLED(uint8_t enabled) {
    accel_select();
    accel_write_address(ADDR_ACC_PWR_CTRL, enabled);
    accel_unselect();
}

Buffer ACCEL_READ_FIFO(void) {
    uint8_t raw_buffer[FIFO_MAX_BUFFER_BYTES];
    uint8_t frame_type;
    int i = 0;

    Buffer data_buffer;
    data_buffer.len = 0;
    data_buffer.array = malloc(sizeof(Vector3) * (FIFO_MAX_BUFFER_BYTES / FIFO_DATA_FRAME_SIZE_BYTES));

    accel_select();
    accel_read_address(ADDR_FIFO_DATA, raw_buffer, FIFO_MAX_BUFFER_BYTES);
    accel_unselect();

    frame_type = raw_buffer[i] & 0xFC; // Ignore last 2 bits

    if (frame_type == FIFO_FRAME_H_SKIP) {
        data_buffer.skipped = raw_buffer[i + 1];
        i += 2;
        frame_type = raw_buffer[i] & 0xFC;
    } else {
        data_buffer.skipped = 0;
    }

    while (frame_type != FIFO_FRAME_H_END) {
        switch (frame_type) {
            case FIFO_FRAME_H_DATA:
                data_buffer.array[data_buffer.len] = accel_parse_raw_data(raw_buffer + i + 1);
                data_buffer.len++;
                i += FIFO_DATA_FRAME_SIZE_BYTES;
                break;
            case FIFO_FRAME_H_SENSORTIME:
                i += 4; // Ignore sensor time frame
                break;
            case FIFO_FRAME_H_CONFIG:
                i += 2; // Ignore config frame
                break;
            case FIFO_FRAME_H_DROP:
                data_buffer.array[data_buffer.len] = (Vector3) {NAN, NAN, NAN}; // Indicate dropped frame with NaNs
                data_buffer.len++;
                i += 2;
                break;
            default:
                goto end_loop; // Break out of the loop on unrecognized frame type
        }
        frame_type = raw_buffer[i] & 0xFC;
    }

    end_loop:
    data_buffer.array = realloc(data_buffer.array, data_buffer.len * sizeof(Vector3)); // Scale down to necessary memory
    return data_buffer;
}

void ACCEL_WRITE_FIFO_ENABLED(uint8_t enabled) {
    accel_select();
    accel_write_address(ADDR_FIFO_CONFIG_1, enabled);
    accel_unselect();
}

void ACCEL_WRITE_FIFO_MODE(uint8_t mode_fifo) {
    accel_select();
    accel_write_address(ADDR_FIFO_CONFIG_0, mode_fifo);
    accel_unselect();
}

void ACCEL_WRITE_FIFO_DOWNSAMP(uint8_t downsamp_fifo) {
    accel_select();
    accel_write_address(ADDR_FIFO_DOWNS, downsamp_fifo);
    accel_unselect();
}

// Static functions

static void accel_select(void) {
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_LOW);
}

static void accel_unselect(void) {
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_HIGH);
}

static void accel_read_address(uint8_t address, uint8_t* buffer, int length) {
    address = SPI_READ | address;
    HAL_SPI_Transmit(accel_spi_handle, &address, 1, 100);
    HAL_SPI_Receive(accel_spi_handle, buffer, length, 100);
}

static void accel_write_address(uint8_t address, uint8_t data) {
    uint8_t message[] = {SPI_WRITE | address, data};
    HAL_SPI_Transmit(accel_spi_handle, message, 2, 100);
}

static Vector3 accel_parse_raw_data(uint8_t* raw_data) {
    int16_t x = (raw_data[1] << 8) | raw_data[0];
    int16_t y = (raw_data[3] << 8) | raw_data[2];
    int16_t z = (raw_data[5] << 8) | raw_data[4];
    Vector3 result;

    result.x = ((float)x / 32768.0) * accel_max_range_real;
    result.y = ((float)y / 32768.0) * accel_max_range_real;
    result.z = ((float)z / 32768.0) * accel_max_range_real;

    return result;
}

static void accel_set_range_memory(uint8_t range) {
    accel_max_range_bits = range;
    switch (range) {
        case ACCEL_RANGE_3G:
            accel_max_range_real = 3 * GRAV;
            break;
        case ACCEL_RANGE_6G:
            accel_max_range_real = 6 * GRAV;
            break;
        case ACCEL_RANGE_12G:
            accel_max_range_real = 12 * GRAV;
            break;
        case ACCEL_RANGE_24G:
            accel_max_range_real = 24 * GRAV;
            break;
        default:
            accel_max_range_real = 0;
            break;
    }
}
