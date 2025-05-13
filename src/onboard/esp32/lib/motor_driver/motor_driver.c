#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

#include <motor_driver.h>
#include <constants.h>

void init_motor(void)
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

// Convert a pulse width in µs into an LEDC duty count (0..LEDC_MAX_DUTY)
static uint32_t pulse_us_to_duty(uint32_t pulse_us) {
    const float period_us = 1e6f / PWM_FREQUENCY_HZ;  // e.g. 20 000 µs
    float duty_perc = (float)pulse_us / period_us;    // 0.05–0.10
    return (uint32_t)(duty_perc * LEDC_MAX_DUTY);
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
