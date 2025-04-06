#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

#define TAG "MAIN"

// GPIOs for ESC signal lines
#define MOTOR1_GPIO 18
#define MOTOR2_GPIO 19
#define MOTOR3_GPIO 21
#define MOTOR4_GPIO 22

void init_motor_pwm()
{
    // Initialize GPIOs for MCPWM outputs
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR1_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR2_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR3_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR4_GPIO);

    // Basic config for each PWM timer
    mcpwm_config_t pwm_config = {
        .frequency = 50,  // 50Hz for typical ESCs (20ms period)
        .cmpr_a = 0.0,
        .cmpr_b = 0.0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0, // Active high
    };

    // Init two timers (2 channels per timer)
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

// Helper to convert microseconds to % duty for 50Hz (20ms)
float us_to_duty_percent(uint32_t us)
{
    return ((float)us / 20000.0f) * 100.0f;
}

// Set PWM for a motor
void set_motor_us(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op, uint32_t us)
{
    float duty = us_to_duty_percent(us);
    mcpwm_set_duty(unit, timer, op, duty);
    mcpwm_set_duty_type(unit, timer, op, MCPWM_DUTY_MODE_0);
    ESP_LOGI(TAG, "Motor [%d:%d:%d] set to %.2f%% (%.0f us)", unit, timer, op, duty, (float)us);
}

void app_main()
{
    printf("Hello from ESP32 (printf)\n");

    ESP_LOGI(TAG, "Initializing motors...");
    init_motor_pwm();

    // Let ESCs boot up (usually 1–2 seconds)
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Spin up motors to 10% (e.g., ~1100us)
    uint32_t throttle_us = 1100;

    set_motor_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, throttle_us); // MOTOR1
    set_motor_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, throttle_us); // MOTOR2
    set_motor_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, throttle_us); // MOTOR3
    set_motor_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, throttle_us); // MOTOR4

    ESP_LOGI(TAG, "Motors spinning at ~10%% throttle.");
}
