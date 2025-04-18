// mpu.c
#include "mpu.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "wifi_logger.h"
#include <stdio.h>
#include <math.h>

// ============================================================================
// Hardware Configuration
// ============================================================================
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 400000
#define MPU6050_I2C_ADDR 0x68

// ============================================================================
// MPU6050 Register Map
// ============================================================================
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define WHO_AM_I 0x75
#define TEMP_OUT_H 0x41

// ============================================================================
// Sensor Configuration
// ============================================================================
#define GYRO_FS_SEL 0x01           // ±500°/s range
#define ACCEL_FS_SEL 0x00          // ±2g
#define DLPF_CONFIG 0x01           // 184Hz bandwidth
#define GYRO_SENSITIVITY 65.5f     // LSB/°/s for ±500°/s
#define ACCEL_SENSITIVITY 16384.0f // LSB/g for ±2g
#define TEMP_SENSITIVITY 340.0f    // Temperature sensitivity
#define ROOM_TEMP_OFFSET 35.0f     // Room temperature offset

// ============================================================================
// Sensor Fusion Parameters
// ============================================================================
#define ALPHA 0.98f              // Complementary filter coefficient
#define GYRO_THRESHOLD 0.1f      // Threshold for yaw changes (°/s)
#define CALIBRATION_SAMPLES 2000 // Number of calibration samples
#define GYRO_SCALE 1.1f          // Yaw scaling factor

// ============================================================================
// State Variables
// ============================================================================
static struct
{
    float gyro[3];           // Current gyro readings
    float accel[3];          // Current accelerometer readings
    float angles[3];         // Current roll, pitch, yaw
    float gyro_bias[3];      // Gyroscope bias values
    float temp_bias;         // Temperature bias
    float last_temp;         // Last temperature reading
    uint64_t last_update_us; // Last update timestamp
    bool initialized;        // Initialization status
} mpu_state = {0};

static SemaphoreHandle_t mpu_mutex = NULL;
static TaskHandle_t mpu_task_handle = NULL;

// ============================================================================
// Helper Functions
// ============================================================================
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t mpu_write_reg(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu_read_regs(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (len == 0)
        return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static float read_temperature(void)
{
    uint8_t data[2];
    mpu_read_regs(TEMP_OUT_H, data, 2);
    int16_t raw_temp = (data[0] << 8) | data[1];
    return (float)raw_temp / TEMP_SENSITIVITY + ROOM_TEMP_OFFSET;
}

static void read_sensor_data(void)
{
    uint8_t data[14];

    // Read accelerometer data
    mpu_read_regs(ACCEL_XOUT_H, data, 6);
    mpu_state.accel[0] = (int16_t)((data[0] << 8) | data[1]) / ACCEL_SENSITIVITY;
    mpu_state.accel[1] = (int16_t)((data[2] << 8) | data[3]) / ACCEL_SENSITIVITY;
    mpu_state.accel[2] = (int16_t)((data[4] << 8) | data[5]) / ACCEL_SENSITIVITY;

    // Read gyroscope data
    mpu_read_regs(GYRO_XOUT_H, data + 6, 6);
    float raw_gyro[3] = {
        (int16_t)((data[6] << 8) | data[7]) / GYRO_SENSITIVITY - mpu_state.gyro_bias[0],
        (int16_t)((data[8] << 8) | data[9]) / GYRO_SENSITIVITY - mpu_state.gyro_bias[1],
        (int16_t)((data[10] << 8) | data[11]) / GYRO_SENSITIVITY - mpu_state.gyro_bias[2]};

    // Temperature compensation
    float current_temp = read_temperature();
    float temp_diff = current_temp - mpu_state.last_temp;
    float temp_compensation = temp_diff * 0.1f;

    // Map gyro axes to car's orientation with temperature compensation
    mpu_state.gyro[0] = -raw_gyro[1] - temp_compensation;
    mpu_state.gyro[1] = raw_gyro[0] - temp_compensation;
    mpu_state.gyro[2] = raw_gyro[2] - temp_compensation;

    // Apply threshold to reduce noise
    for (int i = 0; i < 3; i++)
    {
        if (fabs(mpu_state.gyro[i]) < GYRO_THRESHOLD)
            mpu_state.gyro[i] = 0;
    }

    mpu_state.last_temp = current_temp;
}

static void update_orientation(void)
{
    uint64_t current_time_us = esp_timer_get_time();
    float dt = (current_time_us - mpu_state.last_update_us) / 1000000.0f;
    if (dt <= 0 || dt > 0.1f)
        dt = 0.01f;
    mpu_state.last_update_us = current_time_us;

    // Calculate angles from accelerometer
    float accel_roll = atan2f(mpu_state.accel[0], mpu_state.accel[2]) * 180.0f / M_PI;
    float accel_pitch = atan2f(-mpu_state.accel[1], sqrtf(mpu_state.accel[0] * mpu_state.accel[0] +
                                                          mpu_state.accel[2] * mpu_state.accel[2])) *
                        180.0f / M_PI;

    // Complementary filter
    mpu_state.angles[0] = ALPHA * (mpu_state.angles[0] + mpu_state.gyro[1] * dt) +
                          (1 - ALPHA) * accel_roll;
    mpu_state.angles[1] = ALPHA * (mpu_state.angles[1] + mpu_state.gyro[0] * dt) +
                          (1 - ALPHA) * accel_pitch;

    // Update yaw with threshold and scaling
    float filtered_gyro_z = (fabsf(mpu_state.gyro[2]) < GYRO_THRESHOLD) ? 0.0f : mpu_state.gyro[2];
    mpu_state.angles[2] += filtered_gyro_z * dt * GYRO_SCALE;

    // Normalize yaw to [0, 360)
    mpu_state.angles[2] = fmodf(mpu_state.angles[2], 360.0f);
    if (mpu_state.angles[2] < 0.0f)
        mpu_state.angles[2] += 360.0f;
}

static void calibrate_gyro(void)
{
    float sum[3] = {0}, sum_temp = 0;
    uint8_t data[6];

    printf("Calibrating gyroscope - keep the sensor still...\n");

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        mpu_read_regs(GYRO_XOUT_H, data, 6);
        float temp = read_temperature();

        for (int j = 0; j < 3; j++)
        {
            sum[j] += (int16_t)((data[j * 2] << 8) | data[j * 2 + 1]);
        }
        sum_temp += temp;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for (int i = 0; i < 3; i++)
    {
        mpu_state.gyro_bias[i] = sum[i] / (CALIBRATION_SAMPLES * GYRO_SENSITIVITY);
    }
    mpu_state.temp_bias = sum_temp / CALIBRATION_SAMPLES - ROOM_TEMP_OFFSET;

    printf("Gyro calibration complete. Bias: X=%.3f, Y=%.3f, Z=%.3f deg/s, Temp=%.1f°C\n",
           mpu_state.gyro_bias[0], mpu_state.gyro_bias[1], mpu_state.gyro_bias[2],
           mpu_state.temp_bias + ROOM_TEMP_OFFSET);
}

static void mpu_task(void *pvParameters)
{
    while (1)
    {
        if (mpu_mutex != NULL && xSemaphoreTake(mpu_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            read_sensor_data();
            update_orientation();
            xSemaphoreGive(mpu_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz sampling rate
    }
}

// ============================================================================
// Public Interface Functions
// ============================================================================
int mpu_init(void)
{
    // Create mutex
    mpu_mutex = xSemaphoreCreateMutex();
    if (mpu_mutex == NULL)
    {
        printf("Failed to create MPU mutex!\n");
        return -1;
    }

    // Initialize I2C
    if (i2c_master_init() != ESP_OK)
    {
        printf("Failed to initialize I2C!\n");
        return -1;
    }

    // Wake up MPU6050
    mpu_write_reg(PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure MPU6050
    mpu_write_reg(SMPLRT_DIV, 0x07);    // 125Hz sampling rate
    mpu_write_reg(CONFIG, DLPF_CONFIG); // Digital low pass filter
    mpu_write_reg(GYRO_CONFIG, GYRO_FS_SEL << 3);
    mpu_write_reg(ACCEL_CONFIG, ACCEL_FS_SEL << 3);

    // Verify WHO_AM_I register
    uint8_t who_am_i;
    mpu_read_regs(WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x68)
    {
        printf("MPU6050 initialization failed! WHO_AM_I = 0x%02X\n", who_am_i);
        return -1;
    }

    // Perform gyroscope calibration
    vTaskDelay(pdMS_TO_TICKS(1000));
    calibrate_gyro();

    // Create MPU task
    if (xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 5, &mpu_task_handle) != pdPASS)
    {
        printf("Failed to create MPU task!\n");
        return -1;
    }

    mpu_state.initialized = true;
    printf("MPU6050 initialization complete with task-based reading.\n");
    return 0;
}

mpu_data_t mpu_get_orientation(void)
{
    mpu_data_t data = {0};

    if (!mpu_state.initialized)
    {
        printf("Warning: MPU6050 not initialized!\n");
        return data;
    }

    if (mpu_mutex != NULL && xSemaphoreTake(mpu_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        data.roll = mpu_state.angles[0];
        data.pitch = mpu_state.angles[1];
        data.yaw = mpu_state.angles[2];
        xSemaphoreGive(mpu_mutex);
    }
    else
    {
        printf("Warning: Failed to get MPU mutex!\n");
    }

    return data;
}

void mpu_log_orientation(float current_yaw, float last_yaw, uint32_t print_interval_ms)
{
    float yaw_change = fabsf(current_yaw - last_yaw);
    if (yaw_change > 180.0f)
        yaw_change = 360.0f - yaw_change;

    log_remote("Yaw: %.2f° (Change: %.2f°/s)",
               current_yaw,
               yaw_change * (1000.0f / print_interval_ms));
}
