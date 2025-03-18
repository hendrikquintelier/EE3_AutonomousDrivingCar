// mpu.c
#include "mpu.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <math.h>

// Adjust these for your board's I2C
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 400000 // 400kHz if MPU6050 supports it
#define MPU6050_I2C_ADDR 0x68     // AD0=GND => 0x68

// Example register addresses for MPU6050
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_WHO_AM_I 0x75
// ... plus registers for accelerometer/gyro data

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

void mpu_init(void)
{
    // Initialize I2C
    i2c_master_init();

    // Wake up MPU6050
    // Write 0 to PWR_MGMT_1
    uint8_t buf[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    printf("MPU6050 initialized.\n");
}

mpu_data_t mpu_get_orientation(void)
{
    mpu_data_t data = {0};
    // TODO: read gyro/accel registers from MPU6050,
    // fuse them into a yaw/pitch/roll or heading.
    // For example:
    data.yaw = 0.0f;
    data.pitch = 0.0f;
    data.roll = 0.0f;
    return data;
}
