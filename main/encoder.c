#include "encoder.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_mac.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include <stdio.h>
#include "esp_timer.h"
#include "wifi_logger.h"

// Add spinlock definition
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// --- Pin definitions ---
#define ENCODER1_PIN_A 6
#define ENCODER1_PIN_B 7
#define ENCODER2_PIN_A 15
#define ENCODER2_PIN_B 16

// --- Encoder and wheel parameters ---
#define ENCODER_PPR 24         // Pulses per revolution (for one channel)
#define WHEEL_DIAMETER_CM 8.0f // Wheel diameter in centimeters
#define WHEEL_CIRCUMFERENCE_CM (3.14159f * WHEEL_DIAMETER_CM)
#define DISTANCE_PER_PULSE (WHEEL_CIRCUMFERENCE_CM / (float)ENCODER_PPR)

// --- Volatile encoder counts ---
static volatile int32_t encoder1_count = 0;
static volatile int32_t encoder2_count = 0;

// --- Shared state variables for quadrature decoding ---
// These variables hold the last known state for each encoder and are shared by both ISRs.
static volatile int encoder1_last_state = 0;
static volatile int encoder2_last_state = 0;

// Add debug counter for ISR triggers
static volatile uint32_t isr1_count = 0;
static volatile uint32_t isr2_count = 0;

// --- Interrupt Service Routines ---
// Use a single shared state per encoder for both channel ISRs.

// Encoder 1 ISR for channel A and channel B
#define DEBOUNCE_THRESHOLD_US 1000 // Ignore transitions less than 1ms apart

// Global variables for debouncing (one per encoder)
static int64_t last_time_encoder1 = 0;
static int64_t last_time_encoder2 = 0;

static void IRAM_ATTR encoder1_isr(void *arg)
{
    int64_t now = esp_timer_get_time(); // current time in microseconds
    if (now - last_time_encoder1 < DEBOUNCE_THRESHOLD_US)
    {
        return; // Ignore this transition as it is too close to the last one
    }
    last_time_encoder1 = now;

    int a_level = gpio_get_level(ENCODER1_PIN_A);
    int b_level = gpio_get_level(ENCODER1_PIN_B);
    int current_state = (a_level << 1) | b_level;

    // Existing quadrature decoding state machine for Encoder 1
    if (encoder1_last_state == 0 && current_state == 2)
        encoder1_count++;
    else if (encoder1_last_state == 2 && current_state == 3)
        encoder1_count++;
    else if (encoder1_last_state == 3 && current_state == 1)
        encoder1_count++;
    else if (encoder1_last_state == 1 && current_state == 0)
        encoder1_count++;
    else if (encoder1_last_state == 0 && current_state == 1)
        encoder1_count--;
    else if (encoder1_last_state == 1 && current_state == 3)
        encoder1_count--;
    else if (encoder1_last_state == 3 && current_state == 2)
        encoder1_count--;
    else if (encoder1_last_state == 2 && current_state == 0)
        encoder1_count--;

    encoder1_last_state = current_state;
}

static void IRAM_ATTR encoder2_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - last_time_encoder2 < DEBOUNCE_THRESHOLD_US)
    {
        return;
    }
    last_time_encoder2 = now;

    int a_level = gpio_get_level(ENCODER2_PIN_A);
    int b_level = gpio_get_level(ENCODER2_PIN_B);
    int current_state = (a_level << 1) | b_level;

    // Existing quadrature decoding state machine for Encoder 2
    if (encoder2_last_state == 0 && current_state == 2)
        encoder2_count++;
    else if (encoder2_last_state == 2 && current_state == 3)
        encoder2_count++;
    else if (encoder2_last_state == 3 && current_state == 1)
        encoder2_count++;
    else if (encoder2_last_state == 1 && current_state == 0)
        encoder2_count++;
    else if (encoder2_last_state == 0 && current_state == 1)
        encoder2_count--;
    else if (encoder2_last_state == 1 && current_state == 3)
        encoder2_count--;
    else if (encoder2_last_state == 3 && current_state == 2)
        encoder2_count--;
    else if (encoder2_last_state == 2 && current_state == 0)
        encoder2_count--;

    encoder2_last_state = current_state;
}

// Add a debug task to monitor encoder operation
static void encoder_debug_task(void *pvParameters)
{
    uint32_t last_isr1_count = 0;
    uint32_t last_isr2_count = 0;
    int32_t last_count1 = 0;
    int32_t last_count2 = 0;

    while (1)
    {
        // Get current values
        uint32_t current_isr1 = isr1_count;
        uint32_t current_isr2 = isr2_count;
        int32_t current_count1 = encoder_get_count1();
        int32_t current_count2 = encoder_get_count2();

        // Calculate changes
        int32_t isr1_diff = current_isr1 - last_isr1_count;
        int32_t isr2_diff = current_isr2 - last_isr2_count;
        int32_t count1_diff = current_count1 - last_count1;
        int32_t count2_diff = current_count2 - last_count2;

        // Read current pin states
        int a1_level = gpio_get_level(ENCODER1_PIN_A);
        int b1_level = gpio_get_level(ENCODER1_PIN_B);
        int a2_level = gpio_get_level(ENCODER2_PIN_A);
        int b2_level = gpio_get_level(ENCODER2_PIN_B);

        // Log debug information
        // log_remote("Encoder Debug:\n");
        // log_remote("Encoder1 - ISRs: %ld, Count: %ld, Pins(A,B): (%d,%d)\n",
        //           isr1_diff, count1_diff, a1_level, b1_level);
        // log_remote("Encoder2 - ISRs: %ld, Count: %ld, Pins(A,B): (%d,%d)\n",
        //           isr2_diff, count2_diff, a2_level, b2_level);

        // Update last values
        last_isr1_count = current_isr1;
        last_isr2_count = current_isr2;
        last_count1 = current_count1;
        last_count2 = current_count2;

        vTaskDelay(pdMS_TO_TICKS(500)); // Print every 500ms
    }
}

void encoder_init(void)
{
    // Configure GPIO pins for encoder 1
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << ENCODER1_PIN_A) | (1ULL << ENCODER1_PIN_B)),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    // Configure GPIO pins for encoder 2
    io_conf.pin_bit_mask = ((1ULL << ENCODER2_PIN_A) | (1ULL << ENCODER2_PIN_B));
    gpio_config(&io_conf);

    // Install GPIO ISR service with default configuration
    gpio_install_isr_service(0);

    // Attach the same ISR to both channels for each encoder
    gpio_isr_handler_add(ENCODER1_PIN_A, encoder1_isr, NULL);
    gpio_isr_handler_add(ENCODER1_PIN_B, encoder1_isr, NULL);
    gpio_isr_handler_add(ENCODER2_PIN_A, encoder2_isr, NULL);
    gpio_isr_handler_add(ENCODER2_PIN_B, encoder2_isr, NULL);

    // Reset encoder counts and states
    encoder_reset();

    // Create debug task
    xTaskCreate(encoder_debug_task, "encoder_debug", 4096, NULL, 1, NULL);

    // Log initialization
    printf("Encoder initialized with pins:\n");
    printf("Encoder 1: A=%d, B=%d\n", ENCODER1_PIN_A, ENCODER1_PIN_B);
    printf("Encoder 2: A=%d, B=%d\n", ENCODER2_PIN_A, ENCODER2_PIN_B);

    // Read and log initial pin states
    int a1 = gpio_get_level(ENCODER1_PIN_A);
    int b1 = gpio_get_level(ENCODER1_PIN_B);
    int a2 = gpio_get_level(ENCODER2_PIN_A);
    int b2 = gpio_get_level(ENCODER2_PIN_B);
    printf("Initial pin states:\n");
    printf("Encoder 1: A=%d, B=%d\n", a1, b1);
    printf("Encoder 2: A=%d, B=%d\n", a2, b2);
}

void encoder_reset(void)
{
    portENTER_CRITICAL(&spinlock);
    encoder1_count = 0;
    encoder2_count = 0;
    // Optionally reset last_state if needed:
    encoder1_last_state = (gpio_get_level(ENCODER1_PIN_A) << 1) | gpio_get_level(ENCODER1_PIN_B);
    encoder2_last_state = (gpio_get_level(ENCODER2_PIN_A) << 1) | gpio_get_level(ENCODER2_PIN_B);
    portEXIT_CRITICAL(&spinlock);
}

int32_t encoder_get_count1(void)
{
    int32_t count;
    portENTER_CRITICAL(&spinlock);
    count = encoder1_count;
    portEXIT_CRITICAL(&spinlock);
    return count;
}

int32_t encoder_get_count2(void)
{
    int32_t count;
    portENTER_CRITICAL(&spinlock);
    count = encoder2_count;
    portEXIT_CRITICAL(&spinlock);
    return count;
}

float encoder_get_distance(void)
{
    // Only use encoder B (right encoder) for distance calculation
    int32_t count2 = encoder_get_count2();

    // Convert count to distance (in centimeters)
    float distance = (float)count2 * DISTANCE_PER_PULSE;

    return distance;
}
