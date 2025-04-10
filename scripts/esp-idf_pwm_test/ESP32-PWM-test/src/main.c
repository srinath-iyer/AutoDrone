#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

// PWM Configuration
#define PWM_FREQUENCY_HZ    50          // 50Hz for standard servo/ESC motors
#define PWM_RESOLUTION      LEDC_TIMER_13_BIT  // 13-bit resolution (0-8191)

// Motor GPIO pins
#define MOTOR1_GPIO        32
#define MOTOR2_GPIO        33
#define MOTOR3_GPIO        25
#define MOTOR4_GPIO        26

// Motor channels
#define MOTOR1_CHANNEL     LEDC_CHANNEL_0
#define MOTOR2_CHANNEL     LEDC_CHANNEL_1
#define MOTOR3_CHANNEL     LEDC_CHANNEL_2
#define MOTOR4_CHANNEL     LEDC_CHANNEL_3

// PWM pulse widths for motor control
#define PULSE_WIDTH_MIN    1000        // 1000 microseconds - minimum throttle
#define PULSE_WIDTH_LOW    1100        // 1100 microseconds - low throttle
#define PULSE_WIDTH_MID    1100        // 1500 microseconds - neutral/mid throttle
#define PULSE_WIDTH_HIGH   1100        // 1900 microseconds - high throttle
#define TRANSITION_TIME_MS 6000        // 6 seconds

// Convert microseconds to duty cycle value
uint32_t microseconds_to_duty(uint32_t microseconds) {
    // For 50Hz frequency, period = 20,000 microseconds
    // For 13-bit resolution, max duty = 8191
    // duty = (microseconds / 20000) * 8191
    return (microseconds * 8191) / 20000;
}

void configure_motors(void) {
    // PWM timer configuration
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // Channel configuration for motor 1
    ledc_channel_config_t channel_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = MOTOR1_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR1_GPIO,
        .duty = microseconds_to_duty(PULSE_WIDTH_MIN),
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    // Channel configuration for motor 2
    channel_conf.channel = MOTOR2_CHANNEL;
    channel_conf.gpio_num = MOTOR2_GPIO;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    // Channel configuration for motor 3
    channel_conf.channel = MOTOR3_CHANNEL;
    channel_conf.gpio_num = MOTOR3_GPIO;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    // Channel configuration for motor 4
    channel_conf.channel = MOTOR4_CHANNEL;
    channel_conf.gpio_num = MOTOR4_GPIO;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

void set_motor_pwm(ledc_channel_t channel, uint32_t pulse_width_us) {
    uint32_t duty = microseconds_to_duty(pulse_width_us);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void set_all_motors_pwm(uint32_t pulse_width_us) {
    set_motor_pwm(MOTOR1_CHANNEL, pulse_width_us);
    set_motor_pwm(MOTOR2_CHANNEL, pulse_width_us);
    set_motor_pwm(MOTOR3_CHANNEL, pulse_width_us);
    set_motor_pwm(MOTOR4_CHANNEL, pulse_width_us);
    
    // printf("Set all motors to %u microseconds (duty: %u)\n", 
        //    pulse_width_us, microseconds_to_duty(pulse_width_us));
}

// Fade motor speed smoothly
void fade_motors(uint32_t start_pulse, uint32_t end_pulse, uint32_t fade_time_ms) {
    int step_count = 100; // Number of steps for smooth transition
    int delay_ms = fade_time_ms / step_count;
    
    for (int i = 0; i <= step_count; i++) {
        // Calculate current pulse width using linear interpolation
        uint32_t current_pulse = start_pulse + ((end_pulse - start_pulse) * i) / step_count;
        set_all_motors_pwm(current_pulse);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    // Initialize PWM for motor control
    configure_motors();
    
    // Start with minimum throttle
    set_all_motors_pwm(PULSE_WIDTH_MIN);
    printf("Motors initialized with minimum throttle (%u μs)\n", PULSE_WIDTH_MIN);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // First transition: Smoothly increase to mid throttle
    printf("Fading to mid throttle over %d seconds...\n", TRANSITION_TIME_MS / 1000);
    fade_motors(PULSE_WIDTH_MIN, PULSE_WIDTH_MID, TRANSITION_TIME_MS);
    printf("Motors at mid throttle (%u μs)\n", PULSE_WIDTH_MID);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Second transition: Smoothly increase to high throttle
    printf("Fading to high throttle over %d seconds...\n", TRANSITION_TIME_MS / 1000);
    fade_motors(PULSE_WIDTH_MID, PULSE_WIDTH_HIGH, TRANSITION_TIME_MS);
    printf("Motors at high throttle (%u μs)\n", PULSE_WIDTH_HIGH);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Third transition: Smoothly decrease back to low throttle
    printf("Fading back to low throttle over %d seconds...\n", TRANSITION_TIME_MS / 1000);
    fade_motors(PULSE_WIDTH_HIGH, PULSE_WIDTH_LOW, TRANSITION_TIME_MS);
    printf("Motors at low throttle (%u μs)\n", PULSE_WIDTH_LOW);
    
    // Loop to maintain operation
    while (1) {
        // Cycle between low and mid throttle every 10 seconds
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        fade_motors(PULSE_WIDTH_LOW, PULSE_WIDTH_MID, 3000);
        
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        fade_motors(PULSE_WIDTH_MID, PULSE_WIDTH_LOW, 3000);
    }
}