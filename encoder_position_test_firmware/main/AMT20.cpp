#include "AMT20.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static const char *TAG_AMT20 = "AMT20";

AMT20::AMT20(spi_host_device_t spi_host,
             gpio_num_t cs_pin,
             gpio_num_t mosi_pin,
             gpio_num_t miso_pin,
             gpio_num_t sclk_pin)
    : spi_host_(spi_host),
      cs_pin_(cs_pin),
      mosi_pin_(mosi_pin),
      miso_pin_(miso_pin),
      sclk_pin_(sclk_pin)
{
}

bool AMT20::read_position(uint16_t &position)
{
    if (!setup())
    {
        return false;
    }

    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_us = now_us - last_read_us_;
    if (elapsed_us < MIN_READ_SPACING_US)
    {
        esp_rom_delay_us(static_cast<uint32_t>(MIN_READ_SPACING_US - elapsed_us));
    }

    uint8_t rx = 0;
    if (!transfer_byte(READ_POSITION, rx))
    {
        return false;
    }

    bool acknowledged = false;
    for (int i = 0; i < MAX_WAIT_BYTES; ++i)
    {
        if (!transfer_byte(NOP, rx))
        {
            return false;
        }

        if (rx == READ_POSITION)
        {
            acknowledged = true;
            break;
        }

        if (rx != WAIT)
        {
            ESP_LOGW(TAG_AMT20, "Unexpected response while reading position: 0x%02X", rx);
            return false;
        }
    }

    if (!acknowledged)
    {
        ESP_LOGW(TAG_AMT20, "Timed out waiting for read-position acknowledgement");
        return false;
    }

    uint8_t msb = 0;
    uint8_t lsb = 0;
    if (!transfer_byte(NOP, msb) || !transfer_byte(NOP, lsb))
    {
        return false;
    }

    last_read_us_ = esp_timer_get_time();
    position = static_cast<uint16_t>(((msb & 0x0F) << 8) | lsb);
    return true;
}

bool AMT20::position_is_near(uint16_t position,
                             uint16_t target,
                             uint16_t tolerance)
{
    position &= 0x0FFF;
    target &= 0x0FFF;

    const uint16_t forward_delta = static_cast<uint16_t>((position - target) & 0x0FFF);
    const uint16_t reverse_delta = static_cast<uint16_t>((target - position) & 0x0FFF);
    const uint16_t shortest_delta = forward_delta < reverse_delta ? forward_delta : reverse_delta;

    return shortest_delta <= tolerance;
}

bool AMT20::setup()
{
    if (ready_)
    {
        return true;
    }

    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = mosi_pin_;
    bus_config.miso_io_num = miso_pin_;
    bus_config.sclk_io_num = sclk_pin_;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 1;

    esp_err_t err = spi_bus_initialize(spi_host_, &bus_config, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG_AMT20, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t device_config = {};
    device_config.clock_speed_hz = SPI_CLOCK_HZ;
    device_config.mode = 0;
    device_config.spics_io_num = cs_pin_;
    device_config.queue_size = 1;

    err = spi_bus_add_device(spi_host_, &device_config, &spi_device_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_AMT20, "SPI device add failed: %s", esp_err_to_name(err));
        return false;
    }

    ready_ = true;
    return true;
}

bool AMT20::transfer_byte(uint8_t tx, uint8_t &rx)
{
    spi_transaction_t transaction = {};
    transaction.length = 8;
    transaction.tx_buffer = &tx;
    transaction.rx_buffer = &rx;

    const esp_err_t err = spi_device_polling_transmit(spi_device_, &transaction);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_AMT20, "SPI transfer failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}
