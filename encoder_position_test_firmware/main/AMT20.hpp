#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/spi_master.h"

class AMT20
{
public:
    AMT20(spi_host_device_t spi_host,
          gpio_num_t cs_pin,
          gpio_num_t mosi_pin,
          gpio_num_t miso_pin,
          gpio_num_t sclk_pin);

    bool read_position(uint16_t &position);

    static bool position_is_near(uint16_t position,
                                 uint16_t target,
                                 uint16_t tolerance);

private:
    bool setup();
    bool transfer_byte(uint8_t tx, uint8_t &rx);

    spi_host_device_t spi_host_;
    gpio_num_t cs_pin_;
    gpio_num_t mosi_pin_;
    gpio_num_t miso_pin_;
    gpio_num_t sclk_pin_;

    spi_device_handle_t spi_device_ = nullptr;
    bool ready_ = false;
    int64_t last_read_us_ = 0;

    static constexpr uint8_t NOP = 0x00;
    static constexpr uint8_t WAIT = 0xA5;
    static constexpr uint8_t READ_POSITION = 0x10;

    static constexpr int SPI_CLOCK_HZ = 1000 * 1000;
    static constexpr int MAX_WAIT_BYTES = 8;
    static constexpr uint32_t INTER_BYTE_DELAY_US = 20;
    static constexpr int64_t MIN_READ_SPACING_US = 20;
};
