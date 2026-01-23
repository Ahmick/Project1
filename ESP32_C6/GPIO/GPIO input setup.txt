#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define INPUT_PIN 4

static void configure_input_pin(void)
{
    /* Create a configuration structure and zero‑initialize it */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INPUT_PIN),   // Bit mask for the pin we want
        .mode = GPIO_MODE_INPUT,               // Set as input
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Enable internal pull‑up (optional)
        .intr_type = GPIO_INTR_DISABLE         // No interrupt for this simple demo
    };

    /* Apply the configuration */
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void app_main(void)
{
    /* 1️⃣ Initialise the GPIO pin */
    configure_input_pin();

    /* 2️⃣ Print a friendly banner */
    printf("\n=== GPIO4 Input Demo ===\n");
    printf("Reading the level of GPIO4 every second.\n");

    /* 3️⃣ Loop forever, reading the pin */
    while (true) {
        int level = gpio_get_level(INPUT_PIN);   // 0 = low, 1 = high
        printf("GPIO4 level: %d\n", level);

        /* Delay 1 second (1000 ms). FreeRTOS works in ticks, so we convert. */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}