/** 
This ESC is an absolute fucking disaster of overcomplication. We’ve got a real control problem, something
that should be handled cleanly and deterministically, and instead the “solution” is: generate a fake RC servo
pulse and pray the sealed little brick interprets it correctly. That’s the interface. That’s it, nothing else.

Calling the function set_torque() is borderline dishonest. We’re nudging a pulse width between 1 ms and 2 ms 
and hoping the ESC’s internal black-box firmware decides to do something vaguely proportional. 

This class is nothing but a shitty abstraction layered over a legacy signal standard that should have
died decades ago. Indirect, lossy and completely detached from the physical quantity we actually care 
about. We are stuck feeding the ESC a glorified guess and letting it figure things out behind closed doors.

The most ironic part of all of this is that we arent working with some ancient hardware. Its a modern brushless ESC 
with sensors, configurability and probably more processing power than it took to put man on the moon. And yet, 
despite all this, the control interface is some prehistoric RC pulse garbage. 

I personally apologise for being complicit in this deceit by naming the set_torque() function as such and I hope
no poor soul ever has to revisit this and discover that arguably the most safety critical custom board on the IC car 
is being driven by a glorified servo pulse and a blind leap of faith. 

If you do really need to be here, the best I can do is point you in the direction of FULL_REVERSE_US, NEUTRAL_US
and FULL_FORWARD_US. These parameters adjust the "torque command" sent to the ESC.

~ James 
*/



#pragma once

#include <algorithm>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

class ESC
{
public:
    explicit ESC(gpio_num_t pwm_pin)                                                        
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

    //   -1.0 means full reverse pulse width
    //    0.0 means neutral pulse width
    //    1.0 means full forward pulse width
    void set_torque(float torque)
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

private:
    // Fixed LEDC timer/channel for the ESC PWM output.
    static constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
    static constexpr ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

    static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_14_BIT;
    static constexpr uint32_t PWM_DUTY_MAX = (1u << 14) - 1u;

    // RC-style PWM.
    static constexpr uint32_t PWM_FREQ_HZ = 60;
    static constexpr float PWM_PERIOD_US = 1000000.0f / PWM_FREQ_HZ;

    // Calibrated pulse widths. Tune these after bench testing.
    static constexpr float FULL_REVERSE_US = 1048.0f;
    static constexpr float NEUTRAL_US = 1479.0f;
    static constexpr float FULL_FORWARD_US = 1910.0f;

    static uint32_t pulse_us_to_duty(float pulse_us)
    {
        pulse_us = std::clamp(pulse_us, FULL_REVERSE_US, FULL_FORWARD_US);
        return static_cast<uint32_t>((pulse_us / PWM_PERIOD_US) * PWM_DUTY_MAX);
    }
};
