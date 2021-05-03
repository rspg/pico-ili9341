#include <stdio.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include "ILI9341.h"


static volatile int s_dma_tx = 0;
static volatile ILI9341* s_active_device = nullptr;

static void waitDma()
{
    dma_channel_wait_for_finish_blocking(s_dma_tx);
}



void ILI9341::initialize(spi_inst_t* spi, int sck, int rx, int tx, int cs, int dc, int reset)
{
    m_spi = spi;
    m_pin_cs = cs;
    m_pin_dc = dc;
    m_pin_reset = reset;

    spi_init(spi, 100*MHZ);
    gpio_set_function(sck,  GPIO_FUNC_SPI);
    gpio_set_function(rx,   GPIO_FUNC_SPI);
    gpio_set_function(tx,   GPIO_FUNC_SPI);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    gpio_init(cs);
    gpio_init(dc);
    gpio_init(reset);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_set_dir(dc, GPIO_OUT);
    gpio_set_dir(reset, GPIO_OUT);
    gpio_put(cs, true);  

    if(s_dma_tx == 0)
        s_dma_tx = dma_claim_unused_channel(true);
    auto config = dma_channel_get_default_config(s_dma_tx);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_dreq(&config, spi_get_index(m_spi) ? DREQ_SPI1_TX : DREQ_SPI0_TX);
    dma_channel_configure(s_dma_tx, &config,
                          &spi_get_hw(m_spi)->dr, // write address
                          nullptr, // read address
                          0, // element count (each element is of size transfer_data_size)
                          false); // don't start yet
    dma_channel_set_irq0_enabled(s_dma_tx, true);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, true);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    initDriver();
}

void ILI9341::setRange(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1)
{
    setColumnRange(x0, x1);
    setRowRange(y0, y1);
}

void ILI9341::setColumnRange(uint16_t x0, uint16_t x1)
{
    const uint8_t argbytes[4] = {
        (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xff),
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xff)
    };
    sendCommand(cmd_column_address_set, argbytes, sizeof(argbytes));
}

void ILI9341::setRowRange(uint16_t y0, uint16_t y1)
{
    const uint8_t argbytes[4] = {
        (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xff),
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xff)
    };
    sendCommand(cmd_page_address_set, argbytes, sizeof(argbytes));
}

void ILI9341::writePixels(const uint16_t* pixels, int count, bool continuous)
{
    sendCommand(continuous ? cmd_write_memory_continue : cmd_memory_write, pixels, count*2);
}

void ILI9341::writePixelsDma(const uint16_t* pixels, int count, bool continuous)
{
    sendCommandDma(continuous ? cmd_write_memory_continue : cmd_memory_write, pixels, count*2);
}

void ILI9341::sendCommand(uint8_t cmd, std::initializer_list<const uint8_t> data)
{
    sendCommand(cmd, data.begin(), data.size());
}

void ILI9341::sendCommand(uint8_t cmd)
{
    sendCommand(cmd, nullptr, 0);
}

void ILI9341::sendCommand(uint8_t cmd, const void* data, int databytes)
{
    waitDma();

    gpio_put(m_pin_cs, false);

    gpio_put(m_pin_dc, false);
    write(cmd);

    gpio_put(m_pin_dc, true);
    write(data, databytes);

    gpio_put(m_pin_cs, true);
}

void ILI9341::sendCommandDma(uint8_t cmd, const void* data, int databytes)
{
    waitDma();

    gpio_put(m_pin_cs, false);

    gpio_put(m_pin_dc, false);
    write(cmd);

    gpio_put(m_pin_dc, true);

    s_active_device = this;
    dma_channel_transfer_from_buffer_now(s_dma_tx, const_cast<void*>(data), databytes);
}

void ILI9341::readCommand(uint8_t cmd, void* receive, int receivebytes)
{
    waitDma();

    gpio_put(m_pin_cs, false);

    gpio_put(m_pin_dc, false);
    write(cmd);

    gpio_put(m_pin_dc, true);
    write(receive, receivebytes);

    gpio_put(m_pin_cs, true);
}

void ILI9341::initDriver()
{
    gpio_put(m_pin_reset, true);
    sleep_ms(1);
    gpio_put(m_pin_reset, false);
    sleep_ms(10);
    gpio_put(m_pin_reset, true);
    sleep_ms(120);

    sendCommand(0xEF, {0x03, 0x80, 0x02});
    sendCommand(cmd_power_control_b, {0x00,0XC1,0X30});
    sendCommand(cmd_power_on_sequence_control, {0x64,0x03,0X12,0X81});
    sendCommand(cmd_driver_timing_control_a_0, {0x85,0x00,0x78});
    sendCommand(cmd_power_control_a, {0x39, 0x2C, 0x00, 0x34, 0x02});
    sendCommand(cmd_pump_ratio_control, {0x20});
    sendCommand(cmd_driver_timing_control_b, {0x00,0x00});
    sendCommand(cmd_power_control_1, {0x23});
    sendCommand(cmd_power_control_2, {0x10});
    sendCommand(cmd_vcom_control_1, {0x3e, 0x28});
    sendCommand(cmd_vcom_control_2, {0x86});
    sendCommand(cmd_memory_access_control, {0x48});
    sendCommand(cmd_vertical_scrolling_start_address, {0x00});
    sendCommand(cmd_pixel_format_set, {0x55});
    sendCommand(cmd_rame_rate_control_in_normal_mode, {0x00, 0x1f});
    sendCommand(cmd_rame_rate_control_in_partial_mode, {0x00, 0x1f});
    sendCommand(cmd_isplay_function_control, {0x08, 0x82, 0x27});
    sendCommand(cmd_backlight_control_7, {0x00});
    sendCommand(cmd_enable_3g, {0x00});
    sendCommand(cmd_gamma_set, {0x01});
    sendCommand(cmd_positive_gamma_correction, {0x0F,0x3a,0x36,0x0b,0x0d,0x06,0x4c,0x91,0x31,0x08,0x10,0x04,0x11,0x0c,0x00});
    sendCommand(cmd_negative_gamma_correction, {0x00,0x06,0x0a,0x05,0x12,0x09,0x2c,0x92,0x3f,0x08,0x0e,0x0b,0x2e,0x33,0x0F});
    sendCommand(cmd_sleep_out);
    sleep_ms(120);
    sendCommand(cmd_display_on);
    sendCommand(cmd_rame_rate_control_in_normal_mode, {0x00, 0x10});
    sendCommand(cmd_rame_rate_control_in_partial_mode, {0x00, 0x10});
}

void ILI9341::write(uint8_t value)
{
    spi_write_blocking(m_spi, &value, 1);
}

int ILI9341::write(const void *src, int bytes)
{
    return spi_write_blocking(m_spi, reinterpret_cast<const uint8_t*>(src), bytes);
}

uint8_t ILI9341::read()
{
    uint8_t read = 0;
    spi_read_blocking(m_spi, 0x00, &read, 1);
    return read;
}

int ILI9341::read(void* dst, int bytes)
{
    return spi_read_blocking(m_spi, 0x00, reinterpret_cast<uint8_t*>(dst), bytes);
}

void ILI9341::dma_handler()
{
    if(s_active_device != nullptr)
    {
        auto hw = spi_get_hw(s_active_device->m_spi);
        while((hw->sr>>4)&1)
            tight_loop_contents();
        gpio_put(s_active_device->m_pin_cs, true);
    }
    dma_hw->ints0 = 1u << s_dma_tx;
}

