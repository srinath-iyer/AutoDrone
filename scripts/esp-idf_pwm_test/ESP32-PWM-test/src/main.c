#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"

// ── USER CONFIG ────────────────────────────────────────────────────────────────
// Change these to the GPIOs wired to your ESC signal wires:
#define MOTOR1_GPIO    4
#define MOTOR2_GPIO    5
#define MOTOR3_GPIO    18
#define MOTOR4_GPIO    19

// ESCs expect ~50 Hz, pulses of 1–2 ms out of 20 ms → duty = 5%–10%
#define PWM_FREQUENCY_HZ   50
#define MIN_PULSE_US     1000    // 1 ms
#define MAX_PULSE_US     2000    // 2 ms

// LEDC configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT  // 13‑bit resolution (0–8191)
#define LEDC_MAX_DUTY           ((1 << 13) - 1)    // 8191

// Which LEDC channels to use for each motor
#define MOTOR1_CH               LEDC_CHANNEL_0
#define MOTOR2_CH               LEDC_CHANNEL_1
#define MOTOR3_CH               LEDC_CHANNEL_2
#define MOTOR4_CH               LEDC_CHANNEL_3
// ────────────────────────────────────────────────────────────────────────────────

// Convert a pulse width in µs into an LEDC duty count (0..LEDC_MAX_DUTY)
static uint32_t pulse_us_to_duty(uint32_t pulse_us) {
    const float period_us = 1e6f / PWM_FREQUENCY_HZ;  // e.g. 20 000 µs
    float duty_perc = (float)pulse_us / period_us;    // 0.05–0.10
    return (uint32_t)(duty_perc * LEDC_MAX_DUTY);
}

// Initialize LEDC timer and all 4 channels
static void ledc_init_output(void)
{
    // 1) configure the timer for 50 Hz PWM
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = PWM_FREQUENCY_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // 2) configure each channel
    const struct {
        ledc_channel_t ch;
        int gpio;
    } motors[4] = {
        { MOTOR1_CH, MOTOR1_GPIO },
        { MOTOR2_CH, MOTOR2_GPIO },
        { MOTOR3_CH, MOTOR3_GPIO },
        { MOTOR4_CH, MOTOR4_GPIO },
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_cfg = {
            .speed_mode     = LEDC_MODE,
            .channel        = motors[i].ch,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motors[i].gpio,
            .duty           = 0,            // start at 0% (off)
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }
}

/// Set one motor’s speed with a pulse width [MIN_PULSE_US .. MAX_PULSE_US].
/// motor_id: 1..4, pulse_us: 1000 (stop) to 2000 (full throttle)
void set_motor_speed(int motor_id, uint32_t pulse_us)
{
    if (pulse_us < MIN_PULSE_US) pulse_us = MIN_PULSE_US;
    if (pulse_us > MAX_PULSE_US) pulse_us = MAX_PULSE_US;

    uint32_t duty = pulse_us_to_duty(pulse_us);

    ledc_channel_t ch;
    switch (motor_id) {
        case 1: ch = MOTOR1_CH; break;
        case 2: ch = MOTOR2_CH; break;
        case 3: ch = MOTOR3_CH; break;
        case 4: ch = MOTOR4_CH; break;
        default:
            printf("Invalid motor id %d\n", motor_id);
            return;
    }

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ch, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ch));
}

// Demo task: arms ESCs, then ramps all motors up and down
static void motor_sweep_task(void *arg){
    // give ESCs time to arm
    printf("Waiting 2s for ESC arming...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    const uint32_t step = 100;  // µs step
    // ramp-up

    for (int m = 1; m <= 4; m++) {
        set_motor_speed(m, MIN_PULSE_US);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));


    for (uint32_t pulse = MIN_PULSE_US; pulse <= 1200; pulse += step) {
        for (int m = 1; m <= 4; m++) {
            set_motor_speed(m, pulse);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    // ramp-down

    // then stop
    for (int m = 1; m <= 4; m++) {
        set_motor_speed(m, MIN_PULSE_US);
    }
    printf("Sweep complete.\n");
    vTaskDelete(NULL);
}

// ESC Calibration Task
// static void motor_sweep_task(void *arg){
//     printf("Motor set to 2000.\n");
//     set_motor_speed(2, 2000); // Start motor 1 at minimum speed
//     printf("Button: %d\n", gpio_get_level(23));
//     while(gpio_get_level(23)==0){
//         // Wait for the switch to be pressed
//         vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100 ms
//     }
//     printf("Switch pressed, stopping motor.\n");
//     set_motor_speed(2, 1000); // Stop motor 1
//     vTaskDelete(NULL); // Delete the task after stopping the motor
// }



void app_main(void)
{
    ledc_init_output();
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 23),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    xTaskCreate(motor_sweep_task, "motor_sweep", 4096, NULL, 5, NULL);
}
