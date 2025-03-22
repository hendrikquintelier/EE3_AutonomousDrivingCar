// mpu.c
#include "mpu.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <math.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// I2C pins from Excel
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 9 // SCL pin from Excel
#define I2C_MASTER_SDA_IO 8 // SDA pin from Excel
#define I2C_MASTER_FREQ_HZ 400000
#define MPU6050_I2C_ADDR 0x68 // AD0=GND from Excel

// MPU6050 Register Map
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define WHO_AM_I 0x75
#define TEMP_OUT_H 0x41

// MPU6050 Configuration
#define GYRO_FS_SEL 0x01  // Changed from 0x00 to 0x01 for ±500°/s range
#define ACCEL_FS_SEL 0x00 // ±2g
#define DLPF_CONFIG 0x01  // Changed from 0x03 to 0x01 for 184Hz bandwidth

// Sensor fusion parameters
#define ALPHA 0.98f                // Increased from 0.96f for more gyro influence
#define GYRO_SENSITIVITY 65.5f     // Changed from 131.0f for ±500°/s range
#define ACCEL_SENSITIVITY 16384.0f // LSB/g for ±2g range
#define CALIBRATION_SAMPLES 2000   // Increased from 1000 for better calibration
#define GYRO_THRESHOLD 0.1f        // Threshold for yaw changes (degrees/second)
#define TEMP_SENSITIVITY 340.0f    // Temperature sensitivity
#define ROOM_TEMP_OFFSET 35.0f     // Room temperature offset

// Global variables for sensor fusion
static float gyro_x = 0, gyro_y = 0, gyro_z = 0;
static float accel_x = 0, accel_y = 0, accel_z = 0;
static float roll = 0, pitch = 0, yaw = 0;
static uint64_t last_update_us = 0;
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
static float temp_bias = 0;
static float last_temp = 0;

// Add mutex for thread safety
static SemaphoreHandle_t mpu_mutex = NULL;
static bool mpu_initialized = false;
static TaskHandle_t mpu_task_handle = NULL;

// Forward declarations of static functions
static void read_sensor_data(void);
static void update_orientation(void);
static esp_err_t i2c_master_init(void);
static esp_err_t mpu_write_reg(uint8_t reg_addr, uint8_t data);
static esp_err_t mpu_read_reg(uint8_t reg_addr, uint8_t *data);
static esp_err_t mpu_read_regs(uint8_t reg_addr, uint8_t *data, size_t len);
static void calibrate_gyro(void);
static void mpu_task(void *pvParameters);
static float read_temperature(void);

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
    // Convert to match our orientation (Y=front, Z=up, X=side)
    accel_x = (int16_t)((data[0] << 8) | data[1]) / ACCEL_SENSITIVITY; // Side acceleration
    accel_y = (int16_t)((data[2] << 8) | data[3]) / ACCEL_SENSITIVITY; // Forward acceleration
    accel_z = (int16_t)((data[4] << 8) | data[5]) / ACCEL_SENSITIVITY; // Vertical acceleration

    // Read gyroscope data
    mpu_read_regs(GYRO_XOUT_H, data + 6, 6);
    // Convert and apply bias compensation, adjusting signs to match our orientation
    float raw_gyro_x = (int16_t)((data[6] << 8) | data[7]) / GYRO_SENSITIVITY - gyro_bias_x;
    float raw_gyro_y = (int16_t)((data[8] << 8) | data[9]) / GYRO_SENSITIVITY - gyro_bias_y;
    float raw_gyro_z = (int16_t)((data[10] << 8) | data[11]) / GYRO_SENSITIVITY - gyro_bias_z;

    // Read temperature and apply temperature compensation
    float current_temp = read_temperature();
    float temp_diff = current_temp - last_temp;
    float temp_compensation = temp_diff * 0.1f; // Temperature compensation factor

    // Map gyro axes to car's orientation with temperature compensation
    gyro_x = -raw_gyro_y - temp_compensation;
    gyro_y = raw_gyro_x - temp_compensation;
    gyro_z = raw_gyro_z - temp_compensation;

    // Apply threshold to reduce noise
    if (fabs(gyro_x) < GYRO_THRESHOLD)
        gyro_x = 0;
    if (fabs(gyro_y) < GYRO_THRESHOLD)
        gyro_y = 0;
    if (fabs(gyro_z) < GYRO_THRESHOLD)
        gyro_z = 0;

    last_temp = current_temp;
}

static void update_orientation(void)
{
    // Get high-resolution time in microseconds
    uint64_t current_time_us = esp_timer_get_time();
    float dt = (current_time_us - last_update_us) / 1000000.0f;
    if (dt <= 0 || dt > 0.1f)
        dt = 0.01f;
    last_update_us = current_time_us;

    // Calculate angles based on our orientation
    float accel_roll = atan2f(accel_x, accel_z) * 180.0f / M_PI;
    float accel_pitch = atan2f(-accel_y, sqrtf(accel_x * accel_x + accel_z * accel_z)) * 180.0f / M_PI;

    // Complementary filter
    roll = ALPHA * (roll + gyro_y * dt) + (1 - ALPHA) * accel_roll;
    pitch = ALPHA * (pitch + gyro_x * dt) + (1 - ALPHA) * accel_pitch;

    // Update yaw with threshold
    float filtered_gyro_z = (fabsf(gyro_z) < GYRO_THRESHOLD) ? 0.0f : gyro_z;
    const float GYRO_SCALE = 1.1f;
    yaw += filtered_gyro_z * dt * GYRO_SCALE;

    // Normalize yaw to [0, 360)
    yaw = fmodf(yaw, 360.0f);
    if (yaw < 0.0f)
        yaw += 360.0f;
}

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

static esp_err_t mpu_read_reg(uint8_t reg_addr, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu_read_regs(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (len == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void calibrate_gyro(void)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    float sum_temp = 0;
    uint8_t data[6];

    printf("Calibrating gyroscope - keep the sensor still...\n");

    // Collect samples with longer delay for stability
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        mpu_read_regs(GYRO_XOUT_H, data, 6);
        float temp = read_temperature();

        sum_x += (int16_t)((data[0] << 8) | data[1]);
        sum_y += (int16_t)((data[2] << 8) | data[3]);
        sum_z += (int16_t)((data[4] << 8) | data[5]);
        sum_temp += temp;

        vTaskDelay(pdMS_TO_TICKS(5)); // Increased from 2ms to 5ms for more stable readings
    }

    // Calculate average bias with improved precision
    gyro_bias_x = sum_x / (CALIBRATION_SAMPLES * GYRO_SENSITIVITY);
    gyro_bias_y = sum_y / (CALIBRATION_SAMPLES * GYRO_SENSITIVITY);
    gyro_bias_z = sum_z / (CALIBRATION_SAMPLES * GYRO_SENSITIVITY);
    temp_bias = sum_temp / CALIBRATION_SAMPLES - ROOM_TEMP_OFFSET;

    printf("Gyro calibration complete. Bias: X=%.3f, Y=%.3f, Z=%.3f deg/s, Temp=%.1f°C\n",
           gyro_bias_x, gyro_bias_y, gyro_bias_z, temp_bias + ROOM_TEMP_OFFSET);
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

void mpu_init(void)
{
    // Create mutex first
    mpu_mutex = xSemaphoreCreateMutex();
    if (mpu_mutex == NULL)
    {
        printf("Failed to create MPU mutex!\n");
        return;
    }

    // Initialize I2C
    i2c_master_init();

    // Wake up MPU6050
    mpu_write_reg(PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure MPU6050
    mpu_write_reg(SMPLRT_DIV, 0x07);    // Sample rate divider: 1kHz / (1 + 7) = 125Hz
    mpu_write_reg(CONFIG, DLPF_CONFIG); // Digital low pass filter
    mpu_write_reg(GYRO_CONFIG, GYRO_FS_SEL << 3);
    mpu_write_reg(ACCEL_CONFIG, ACCEL_FS_SEL << 3);

    // Verify WHO_AM_I register
    uint8_t who_am_i;
    mpu_read_reg(WHO_AM_I, &who_am_i);
    if (who_am_i != 0x68)
    {
        printf("MPU6050 initialization failed! WHO_AM_I = 0x%02X\n", who_am_i);
        return;
    }

    // Perform gyroscope calibration
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for sensor to stabilize
    calibrate_gyro();

    // Create MPU task
    BaseType_t xReturned = xTaskCreate(
        mpu_task,
        "mpu_task",
        4096,
        NULL,
        5,
        &mpu_task_handle);

    if (xReturned != pdPASS)
    {
        printf("Failed to create MPU task!\n");
        return;
    }

    mpu_initialized = true;
    printf("MPU6050 initialization complete with task-based reading.\n");
}

mpu_data_t mpu_get_orientation(void)
{
    mpu_data_t data = {0}; // Initialize to zero

    if (!mpu_initialized)
    {
        printf("Warning: MPU6050 not initialized!\n");
        return data;
    }

    if (mpu_mutex != NULL && xSemaphoreTake(mpu_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        data.roll = roll;
        data.pitch = pitch;
        data.yaw = yaw;
        xSemaphoreGive(mpu_mutex);
    }
    else
    {
        printf("Warning: Failed to get MPU mutex!\n");
    }

    return data;
}
