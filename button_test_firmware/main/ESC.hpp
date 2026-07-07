#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/ledc.h"

class ESC
{
public:
    explicit ESC(gpio_num_t pwm_pin);

    void set_torque(float torque);

private:
    static constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
    static constexpr ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

    static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_14_BIT;
    static constexpr uint32_t PWM_DUTY_MAX = (1u << 14) - 1u;

    static constexpr uint32_t PWM_FREQ_HZ = 60;
    static constexpr float PWM_PERIOD_US = 1000000.0f / PWM_FREQ_HZ;

    static constexpr float FULL_REVERSE_US = 1048.0f;
    static constexpr float NEUTRAL_US = 1479.0f;
    static constexpr float FULL_FORWARD_US = 1910.0f;

    static uint32_t pulse_us_to_duty(float pulse_us);
};
