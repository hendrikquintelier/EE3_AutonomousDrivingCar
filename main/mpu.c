// mpu.c
#include "ultrasonic.h"
#include "mpu.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
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
#define MPU_INT_PIN 3  // GPIO pin for MPU6050 interrupt
#define INT_ENABLE 0x12  // MPU6050 register for interrupt enable
#define INT_STATUS 0x3A  // MPU6050 register for interrupt status

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
#define GYRO_THRESHOLD 0.01f      // Reduced threshold for more sensitivity
#define TEMP_COMPENSATION_FACTOR 0.05f  // Reduced temperature compensation
#define CALIBRATION_SAMPLES 2000 // Number of calibration samples
#define GYRO_SCALE 1.0f          // Removed scaling factor
#define DRIFT_COMPENSATION 0.001f // Added drift compensation factor

// ============================================================================
// Ultrasonic Filter Parameters
// ============================================================================
#define FRONTAL_DISTANCE_ALPHA 0.15f     // Reduced from 0.3f for smoother transitions
#define FRONTAL_DISTANCE_THRESHOLD 10.0f // Increased from 5.0f to handle larger changes
#define FRONTAL_DISTANCE_MAX 400.0f      // Maximum valid distance in cm
#define FRONTAL_DISTANCE_MIN 2.0f        // Minimum valid distance in cm
#define FRONTAL_DISTANCE_STEP_MAX 50.0f  // Maximum allowed step change in cm

// ============================================================================
// State Variables
// ============================================================================
static struct
{
    float gyro[3];                   // Current gyro readings
    float accel[3];                  // Current accelerometer readings
    float angles[3];                 // Current roll, pitch, yaw
    float gyro_bias[3];              // Gyroscope bias values
    float accel_bias[3];             // Accelerometer bias values
    float temp_bias;                 // Temperature bias
    float last_temp;                 // Last temperature reading
    uint64_t last_update_us;         // Last update timestamp
    bool initialized;                // Initialization status
    float filtered_frontal_distance; // Filtered frontal distance
    float last_frontal_distance;     // Last raw frontal distance
    float velocity_y;                // Current velocity in y direction
    float position_y;                // Current position in y direction
    float last_accel_y;              // Last acceleration in y direction
    bool movement_started;           // Flag to track if movement has started
    bool is_moving;                  // Flag to track if currently moving
    uint32_t still_counter;          // Counter for detecting stillness
    volatile bool data_ready;        // Flag for data ready interrupt
    volatile uint32_t sample_count;  // Count of samples taken
    volatile uint64_t last_sample_time;  // Timestamp of last sample
} mpu_state = {0};

static SemaphoreHandle_t mpu_mutex = NULL;

// Add queue for sensor data requests
static QueueHandle_t mpu_data_queue = NULL;

// Add a structure for sensor data requests
typedef struct {
    uint64_t timestamp;
    uint8_t data[14];
} mpu_data_request_t;

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

static void calibrate_gyro(void)
{
    float sum[3] = {0}, sum_temp = 0;
    uint8_t data[6];

    // printf("Calibrating gyroscope - keep the sensor still...\n");

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

    // Initialize yaw to 0 after calibration
    mpu_state.angles[2] = 0.0f;
    
    // Only log yaw-related bias
    printf("Gyro Z bias: %.3f deg/s\n", mpu_state.gyro_bias[2]);
}

static void calibrate_accel(void)
{
    float sum[3] = {0};
    uint8_t data[6];

    // printf("Calibrating accelerometer - keep the sensor still...\n");

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        mpu_read_regs(ACCEL_XOUT_H, data, 6);

        for (int j = 0; j < 3; j++)
        {
            sum[j] += (int16_t)((data[j * 2] << 8) | data[j * 2 + 1]);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for (int i = 0; i < 3; i++)
    {
        mpu_state.accel_bias[i] = sum[i] / (CALIBRATION_SAMPLES * ACCEL_SENSITIVITY);
    }

    // printf("Accel calibration complete. Bias: X=%.3f, Y=%.3f, Z=%.3f g\n",
    //        mpu_state.accel_bias[0], mpu_state.accel_bias[1], mpu_state.accel_bias[2]);
}

// Add a simple interrupt counter for testing
static volatile uint32_t g_test_interrupt_count = 0;

// Simplified interrupt handler with no blocking operations
static void IRAM_ATTR mpu_interrupt_handler(void* arg) {
    // Simply increment the counter and send a request to the queue
    g_test_interrupt_count++;
    
    // Create a data request
    mpu_data_request_t request;
    request.timestamp = esp_timer_get_time();
    
    // Send the request to the queue
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(mpu_data_queue, &request, &xHigherPriorityTaskWoken);
    
    // If a higher priority task was woken, yield
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Enhanced data task with detailed debugging
static void mpu_data_task(void *pvParameter) {
    printf("\n=== MPU DATA TASK STARTED ===\n");
    
    uint64_t last_print_time = esp_timer_get_time();
    uint32_t last_interrupt_count = 0;
    uint32_t samples_in_last_second = 0;
    
    while (1) {
        mpu_data_request_t request;
        
        // Wait for a data request
        if (xQueueReceive(mpu_data_queue, &request, portMAX_DELAY) == pdTRUE) {
            // Read sensor data
            uint8_t data[14];
            esp_err_t ret = mpu_read_regs(ACCEL_XOUT_H, data, 14);
            
            // Update sample count
            samples_in_last_second++;
            
            // Print detailed debug info every second
            uint64_t current_time = esp_timer_get_time();
            if (current_time - last_print_time > 1000000) { // 1 second
                uint32_t current_interrupt_count = g_test_interrupt_count;
                uint32_t interrupts_in_last_second = current_interrupt_count - last_interrupt_count;
                
                printf("\n=== MPU DATA TASK UPDATE ===\n");
                printf("Total interrupts: %lu\n", (unsigned long)current_interrupt_count);
                printf("Interrupts in last second: %lu\n", (unsigned long)interrupts_in_last_second);
                printf("Samples processed in last second: %lu\n", (unsigned long)samples_in_last_second);
                printf("Expected rate (200Hz): 200\n");
                printf("Queue items waiting: %lu\n", (unsigned long)uxQueueMessagesWaiting(mpu_data_queue));
                printf("==========================\n");
                
                last_print_time = current_time;
                last_interrupt_count = current_interrupt_count;
                samples_in_last_second = 0;
            }
            
            if (ret != ESP_OK) {
                printf("Failed to read sensor data\n");
            }
        }
    }
}

// Enhanced monitor task with detailed debugging
static void monitor_mpu_task(void *pvParameter) {
    uint8_t last_int_status = 0;
    int last_gpio_level = -1;
    uint64_t last_print_time = esp_timer_get_time();
    
    printf("\n=== MPU MONITOR TASK STARTED ===\n");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms
        
        // Check GPIO level
        int gpio_level = gpio_get_level(MPU_INT_PIN);
        
        // Check interrupt status
        uint8_t int_status;
        mpu_read_regs(INT_STATUS, &int_status, 1);
        
        // Print if anything changed or every second
        uint64_t current_time = esp_timer_get_time();
        if (gpio_level != last_gpio_level || int_status != last_int_status || 
            current_time - last_print_time > 1000000) {
            
            printf("\n=== MPU STATUS UPDATE ===\n");
            printf("GPIO %d Level: %d\n", MPU_INT_PIN, gpio_level);
            printf("INT_STATUS: 0x%02X\n", int_status);
            printf("Interrupt Count: %lu\n", (unsigned long)g_test_interrupt_count);
            printf("Queue items waiting: %lu\n", (unsigned long)uxQueueMessagesWaiting(mpu_data_queue));
            printf("Time since last update: %llu us\n", current_time - last_print_time);
            printf("========================\n");
            
            last_gpio_level = gpio_level;
            last_int_status = int_status;
            last_print_time = current_time;
        }
    }
}

// Modify test_interrupt_task to use last_print_time
static void test_interrupt_task(void *pvParameter) {
    uint32_t last_count = 0;
    uint64_t last_print_time = esp_timer_get_time();
    
    printf("\n=== TEST INTERRUPT TASK STARTED ===\n");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Run every second
        
        uint32_t current_count = g_test_interrupt_count;
        uint32_t interrupts_in_last_second = current_count - last_count;
        uint64_t current_time = esp_timer_get_time();
        uint64_t time_since_last_print = current_time - last_print_time;
        
        printf("\n=== INTERRUPT TEST RESULTS ===\n");
        printf("Total interrupts: %lu\n", (unsigned long)current_count);
        printf("Interrupts in last second: %lu\n", (unsigned long)interrupts_in_last_second);
        printf("Time since last print: %llu us\n", time_since_last_print);
        printf("Expected interrupts (200Hz): 200\n");
        printf("============================\n");
        
        last_count = current_count;
        last_print_time = current_time;
    }
}

// Add a function to continuously check interrupt status
void mpu_check_interrupt_status(void) {
    static uint64_t last_check_time = 0;
    uint64_t current_time = esp_timer_get_time();
    
    // Check every 100ms
    if (current_time - last_check_time > 100000) {
        uint8_t int_status;
        mpu_read_regs(INT_STATUS, &int_status, 1);
        if (int_status & 0x01) {
            printf("MPU Interrupt Active! Status: 0x%02X\n", int_status);
        }
        last_check_time = current_time;
    }
}

static esp_err_t check_mpu_interrupt_status(void) {
    uint8_t int_status;
    mpu_read_regs(INT_STATUS, &int_status, 1);
    printf("MPU Interrupt Status: 0x%02X\n", int_status);
    return ESP_OK;
}

// Add a function to check if the interrupt handler is registered
static void check_interrupt_handler_registration(void) {
    printf("\n=== CHECKING INTERRUPT HANDLER REGISTRATION ===\n");
    
    // Check GPIO level
    int gpio_level = gpio_get_level(MPU_INT_PIN);
    printf("Current GPIO %d Level: %d\n", MPU_INT_PIN, gpio_level);
    
    // Check MPU interrupt status
    uint8_t int_status;
    mpu_read_regs(INT_STATUS, &int_status, 1);
    printf("MPU Interrupt Status: 0x%02X\n", int_status);
    
    printf("===========================================\n\n");
}

// Add a function to check GPIO configuration
static void check_gpio_config(void) {
    printf("\n=== CHECKING GPIO CONFIGURATION ===\n");
    
    // Get GPIO configuration using available functions
    uint32_t pin_bit_mask = (1ULL << MPU_INT_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = pin_bit_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    
    printf("GPIO %d Configuration:\n", MPU_INT_PIN);
    printf("  Input Enable: %d\n", io_conf.mode & GPIO_MODE_INPUT);
    printf("  Output Enable: %d\n", io_conf.mode & GPIO_MODE_OUTPUT);
    printf("  Open Drain: %d\n", io_conf.mode & GPIO_MODE_OUTPUT_OD);
    printf("  Pull-up: %d\n", io_conf.pull_up_en);
    printf("  Pull-down: %d\n", io_conf.pull_down_en);
    printf("  Interrupt Type: %d\n", io_conf.intr_type);
    
    // Check current GPIO level
    int gpio_level = gpio_get_level(MPU_INT_PIN);
    printf("Current GPIO Level: %d\n", gpio_level);
    
    printf("==================================\n\n");
}

static esp_err_t configure_mpu_interrupt(void) {
    printf("\n=== CONFIGURING MPU INTERRUPT ===\n");
    
    // Install GPIO ISR service if not already installed
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        printf("Installing GPIO ISR service...\n");
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        isr_service_installed = true;
        printf("GPIO ISR service installed successfully\n");
    }

    // First, disable any existing interrupt
    printf("Removing any existing interrupt handler...\n");
    gpio_isr_handler_remove(MPU_INT_PIN);
    printf("Existing interrupt handler removed\n");
    
    // Configure GPIO for interrupt
    printf("Configuring GPIO %d for interrupt...\n", MPU_INT_PIN);
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Falling edge for active low interrupt
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << MPU_INT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Explicitly enable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    printf("GPIO %d configured for interrupt\n", MPU_INT_PIN);
    
    // Check GPIO configuration
    check_gpio_config();

    // Check GPIO level before configuring MPU
    int initial_gpio_level = gpio_get_level(MPU_INT_PIN);
    printf("Initial GPIO %d Level: %d\n", MPU_INT_PIN, initial_gpio_level);

    // Configure MPU6050 interrupt pin
    printf("Configuring MPU6050 interrupt pin...\n");
    
    // Try different INT_PIN_CFG values
    uint8_t int_pin_cfg_values[] = {0xB8, 0x38, 0x30};  // Try different configurations
    const char* cfg_names[] = {"Active Low (0xB8)", "Active High (0x38)", "Basic (0x30)"};
    
    for (int i = 0; i < 3; i++) {
        printf("\nTrying configuration %s...\n", cfg_names[i]);
        
        // Write the configuration
        ESP_ERROR_CHECK(mpu_write_reg(0x37, int_pin_cfg_values[i]));
        printf("MPU6050 interrupt pin configured: 0x%02X\n", int_pin_cfg_values[i]);

        // Verify the configuration was written correctly
        uint8_t read_back_cfg;
        mpu_read_regs(0x37, &read_back_cfg, 1);
        printf("Read back INT_PIN_CFG: 0x%02X\n", read_back_cfg);
        
        // Check GPIO level after configuration
        int gpio_level = gpio_get_level(MPU_INT_PIN);
        printf("GPIO %d Level after config: %d\n", MPU_INT_PIN, gpio_level);
        
        // Wait a bit to see if the level changes
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_level = gpio_get_level(MPU_INT_PIN);
        printf("GPIO %d Level after delay: %d\n", MPU_INT_PIN, gpio_level);
        
        // If this configuration seems to work, use it
        if (gpio_level == 1) {  // Pin should be high when idle
            printf("Using configuration 0x%02X\n", int_pin_cfg_values[i]);
            break;
        }
    }

    // Add the interrupt handler
    printf("Adding interrupt handler...\n");
    ESP_ERROR_CHECK(gpio_isr_handler_add(MPU_INT_PIN, mpu_interrupt_handler, NULL));
    printf("Interrupt handler added successfully\n");

    // Configure MPU6050 interrupt
    printf("Enabling MPU6050 data ready interrupt...\n");
    ESP_ERROR_CHECK(mpu_write_reg(INT_ENABLE, 0x01));  // Enable data ready interrupt
    printf("MPU6050 interrupt enabled\n");
    
    // Verify interrupt enable was written correctly
    uint8_t read_back_enable;
    mpu_read_regs(INT_ENABLE, &read_back_enable, 1);
    printf("Read back INT_ENABLE: 0x%02X\n", read_back_enable);
    
    // Check initial interrupt status
    uint8_t int_status;
    mpu_read_regs(INT_STATUS, &int_status, 1);
    printf("Initial MPU Interrupt Status: 0x%02X\n", int_status);
    
    // Check GPIO level after configuration
    int final_gpio_level = gpio_get_level(MPU_INT_PIN);
    printf("Final GPIO %d Level: %d\n", MPU_INT_PIN, final_gpio_level);
    
    // Check interrupt handler registration
    check_interrupt_handler_registration();
    
    printf("=== MPU INTERRUPT CONFIGURATION COMPLETE ===\n\n");
    
    return ESP_OK;
}

// Add sampling rate function
float mpu_get_sampling_rate(void) {
    if (mpu_state.sample_count < 2) return 0.0f;
    
    uint64_t time_diff = mpu_state.last_sample_time - mpu_state.last_update_us;
    if (time_diff == 0) return 0.0f;
    
    return (float)mpu_state.sample_count * 1000000.0f / time_diff;
}

// Add function to trigger first read
static void trigger_first_read(void) {
    printf("\n=== TRIGGERING FIRST MPU READ ===\n");
    
    // Check initial interrupt status
    uint8_t int_status;
    mpu_read_regs(INT_STATUS, &int_status, 1);
    printf("Interrupt status before read: 0x%02X\n", int_status);
    
    // Read all sensor data in a single transaction
    uint8_t data[14];
    esp_err_t read_status = mpu_read_regs(ACCEL_XOUT_H, data, 14);  // Read all 14 bytes at once
    printf("Sensor data read status: %s\n", (read_status == ESP_OK) ? "OK" : "FAIL");
    
    // Check interrupt status after read
    mpu_read_regs(INT_STATUS, &int_status, 1);
    printf("Interrupt status after read: 0x%02X\n", int_status);
    
    // If interrupt is still set, try to clear it by reading INT_STATUS
    if (int_status & 0x01) {
        printf("Trying to clear interrupt by reading INT_STATUS...\n");
        mpu_read_regs(INT_STATUS, &int_status, 1);
        printf("Interrupt status after reading INT_STATUS: 0x%02X\n", int_status);
    }
    
    // If interrupt is still set, try to clear it by disabling and re-enabling the interrupt
    if (int_status & 0x01) {
        printf("Trying to clear interrupt by disabling and re-enabling...\n");
        mpu_write_reg(INT_ENABLE, 0x00);  // Disable interrupt
        vTaskDelay(pdMS_TO_TICKS(10));
        mpu_write_reg(INT_ENABLE, 0x01);  // Re-enable interrupt
        mpu_read_regs(INT_STATUS, &int_status, 1);
        printf("Interrupt status after re-enabling: 0x%02X\n", int_status);
    }
    
    // Check GPIO level
    int gpio_level = gpio_get_level(MPU_INT_PIN);
    printf("GPIO %d level after read: %d\n", MPU_INT_PIN, gpio_level);
    
    printf("===============================\n");
}

// ============================================================================
// Public Interface Functions
// ============================================================================
int mpu_init(void)
{
    printf("\n=== MPU INITIALIZATION START ===\n");
    
    // Create mutex first
    mpu_mutex = xSemaphoreCreateMutex();
    if (mpu_mutex == NULL)
    {
        printf("Failed to create MPU mutex!\n");
        return -1;
    }
    printf("MPU mutex created\n");

    // Create the data queue
    mpu_data_queue = xQueueCreate(10, sizeof(mpu_data_request_t));
    if (mpu_data_queue == NULL) {
        printf("Failed to create MPU data queue!\n");
        return -1;
    }
    printf("MPU data queue created\n");

    // Initialize I2C
    if (i2c_master_init() != ESP_OK)
    {
        printf("Failed to initialize I2C!\n");
        return -1;
    }
    printf("I2C initialized\n");

    // Wake up MPU6050
    mpu_write_reg(PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("MPU6050 woken up\n");

    // Configure MPU6050 for higher sampling rate
    mpu_write_reg(SMPLRT_DIV, 0x04);    // Set sample rate to 200Hz (1kHz / (1 + 4))
    mpu_write_reg(CONFIG, 0x03);        // Enable DLPF with bandwidth of 44Hz
    mpu_write_reg(GYRO_CONFIG, GYRO_FS_SEL << 3);
    mpu_write_reg(ACCEL_CONFIG, ACCEL_FS_SEL << 3);
    printf("MPU6050 configured\n");
    
    // Wait for the configuration to take effect
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check the configuration
    uint8_t config_reg;
    mpu_read_regs(CONFIG, &config_reg, 1);
    printf("MPU6050 CONFIG register: 0x%02X\n", config_reg);
    
    uint8_t smplrt_div;
    mpu_read_regs(SMPLRT_DIV, &smplrt_div, 1);
    printf("MPU6050 SMPLRT_DIV register: 0x%02X\n", smplrt_div);

    // Configure interrupt
    if (configure_mpu_interrupt() != ESP_OK) {
        printf("Failed to configure MPU interrupt!\n");
        return -1;
    }
    printf("MPU interrupt configured\n");

    // Verify WHO_AM_I register
    uint8_t who_am_i;
    mpu_read_regs(WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x68)
    {
        printf("MPU6050 initialization failed! WHO_AM_I = 0x%02X\n", who_am_i);
        return -1;
    }
    printf("WHO_AM_I verified: 0x%02X\n", who_am_i);

    // Initialize timing
    mpu_state.last_update_us = esp_timer_get_time();
    mpu_state.initialized = true;
    printf("Timing initialized\n");

    // Trigger first read to start interrupt cycle
    printf("Calling trigger_first_read...\n");
    trigger_first_read();
    printf("trigger_first_read completed\n");

    // Wait a bit for the MPU to start generating data
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Perform gyroscope and accelerometer calibration
    printf("Starting calibration...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    calibrate_gyro();
    calibrate_accel();
    printf("Calibration complete\n");
    
    // Check interrupt status after initialization
    printf("Checking final MPU interrupt status...\n");
    check_mpu_interrupt_status();
    
    // Create monitor task
    BaseType_t monitor_task_created = xTaskCreatePinnedToCore(
        monitor_mpu_task,  // Task function
        "mpu_monitor",    // Task name
        4096,             // Stack size
        NULL,             // Task parameters
        10,               // Priority (higher than default)
        NULL,             // Task handle
        1                 // Core 1
    );
    
    if (monitor_task_created != pdPASS) {
        printf("Failed to create monitor task!\n");
    } else {
        printf("Monitor task created successfully\n");
    }
    
    // Create test interrupt task
    BaseType_t test_task_created = xTaskCreatePinnedToCore(
        test_interrupt_task,  // Task function
        "mpu_test",          // Task name
        4096,                // Stack size
        NULL,                // Task parameters
        10,                  // Priority (higher than default)
        NULL,                // Task handle
        1                    // Core 1
    );
    
    if (test_task_created != pdPASS) {
        printf("Failed to create test interrupt task!\n");
    } else {
        printf("Test interrupt task created successfully\n");
    }
    
    // Create data task
    BaseType_t data_task_created = xTaskCreatePinnedToCore(
        mpu_data_task,     // Task function
        "mpu_data",        // Task name
        4096,              // Stack size
        NULL,              // Task parameters
        10,                // Priority (higher than default)
        NULL,              // Task handle
        1                  // Core 1
    );
    
    if (data_task_created != pdPASS) {
        printf("Failed to create data task!\n");
    } else {
        printf("Data task created successfully\n");
    }
    
    printf("=== MPU INITIALIZATION COMPLETE ===\n\n");
    return 0;
}

mpu_data_t mpu_get_orientation(void)
{
    mpu_data_t data = {0};

    if (!mpu_state.initialized)
    {
        // printf("Warning: MPU6050 not initialized!\n");
        return data;
    }

    if (mpu_mutex != NULL && xSemaphoreTake(mpu_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        data.roll = mpu_state.angles[0];
        data.pitch = mpu_state.angles[1];
        data.yaw = mpu_state.angles[2];
        
        // Temporarily comment out yaw logging
        // printf("Yaw Gyro: %.3f deg/s, Current Yaw: %.3f°\n", 
        //        mpu_state.gyro[2], data.yaw);
        
        xSemaphoreGive(mpu_mutex);
    }
    else
    {
        // printf("Warning: Failed to get MPU mutex!\n");
    }

    return data;
}

void mpu_log_orientation(float current_yaw, float last_yaw, uint32_t print_interval_ms)
{
    float yaw_change = fabsf(current_yaw - last_yaw);
    if (yaw_change > 180.0f)
        yaw_change = 360.0f - yaw_change;

    // Comment out the warning message
    // log_remote("Yaw: %.2f° (Change: %.2f°/s)",
    //            current_yaw,
    //            yaw_change * (1000.0f / print_interval_ms));
}

float mpu_get_filtered_frontal_distance(void)
{
    if (!mpu_state.initialized)
    {
        printf("Warning: MPU6050 not initialized!\n");
        return 0.0f;
    }

    // Return the calculated position (in centimeters)
    return mpu_state.position_y * 100.0f; // Convert from meters to centimeters
}

// Add new function to reset movement tracking
void mpu_reset_movement(void)
{
    mpu_state.movement_started = false;
    mpu_state.velocity_y = 0.0f;
    mpu_state.position_y = 0.0f;
}

// Temporarily disable the monitoring function
void mpu_monitor_interrupts(void) {
    // Disabled for testing
    return;
}
