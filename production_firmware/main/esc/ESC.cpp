#include "ESC.hpp"

#include <algorithm>

#include "esp_err.h"

ESC::ESC(gpio_num_t pwm_pin)
{
    // LEDC is enough because the ESC wants an RC-servo-style PWM signal.
    ledc_timer_config_t timer_config = {};

    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.timer_num = PWM_TIMER;
    timer_config.duty_resolution = PWM_RESOLUTION;
    timer_config.freq_hz = PWM_FREQ_HZ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = static_cast<int>(pwm_pin);
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = PWM_CHANNEL;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = PWM_TIMER;
    channel_config.duty = pulse_us_to_duty(NEUTRAL_US);
    channel_config.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    set_torque(0.0f);
}

void ESC::set_torque(float torque)
{
    // Clamp so bugs elsewhere cannot command outside the calibrated range.
    torque = std::clamp(torque, -1.0f, 1.0f);

    float pulse_us = NEUTRAL_US;
    if (torque > 0.0f)
    {
        pulse_us += (FULL_FORWARD_US - NEUTRAL_US) * torque;
    }
    else if (torque < 0.0f)
    {
        pulse_us += (NEUTRAL_US - FULL_REVERSE_US) * torque;
    }

    ESP_ERROR_CHECK(ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        PWM_CHANNEL,
        pulse_us_to_duty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL));
}

uint32_t ESC::pulse_us_to_duty(float pulse_us)
{
    pulse_us = std::clamp(pulse_us, FULL_REVERSE_US, FULL_FORWARD_US);
    return static_cast<uint32_t>((pulse_us / PWM_PERIOD_US) * PWM_DUTY_MAX);
}
