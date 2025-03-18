// encoder.c
#include "encoder.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdio.h>
#include "esp_attr.h"

// Example definitions
#define ENC1_A_PIN 6
#define ENC1_B_PIN 7
#define ENC2_A_PIN 15
#define ENC2_B_PIN 16

// We'll store encoder counts in static variables
static volatile int encoder1_count = 0;
static volatile int encoder2_count = 0;

// Declare ISR handler function before using it
static void IRAM_ATTR encoder_isr_handler(void *arg);

void encoder_init(void)
{
    // Configure pins as inputs with pullups
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pin_bit_mask = ((1ULL << ENC1_A_PIN) |
                         (1ULL << ENC1_B_PIN) |
                         (1ULL << ENC2_A_PIN) |
                         (1ULL << ENC2_B_PIN))};
    gpio_config(&io_conf);

    // Install ISR service
    gpio_install_isr_service(0);

    // Add ISR handlers (Fix incorrect function name and ensure proper formatting)
    gpio_isr_handler_add(ENC1_A_PIN, encoder_isr_handler, (void *)ENC1_A_PIN);
    gpio_isr_handler_add(ENC1_B_PIN, encoder_isr_handler, (void *)ENC1_B_PIN);
    gpio_isr_handler_add(ENC2_A_PIN, encoder_isr_handler, (void *)ENC2_A_PIN);
    gpio_isr_handler_add(ENC2_B_PIN, encoder_isr_handler, (void *)ENC2_B_PIN);

    printf("Encoders initialized.\n");
}

// Interrupt handler function (must be defined properly)
static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    uint32_t pin = (uint32_t)arg;

    if (pin == ENC1_A_PIN || pin == ENC1_B_PIN)
    {
        encoder1_count++; // You should check A/B state for direction
    }
    else
    {
        encoder2_count++;
    }
}

int encoder_get_count(int encoder_id)
{
    if (encoder_id == 1)
    {
        return encoder1_count;
    }
    else
    {
        return encoder2_count;
    }
}

void encoder_reset(int encoder_id)
{
    if (encoder_id == 1)
    {
        encoder1_count = 0;
    }
    else
    {
        encoder2_count = 0;
    }
}
