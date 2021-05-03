#pragma once

#include <hardware/spi.h>
#include <hardware/dma.h>
#include <initializer_list>

class ILI9341
{
public:
    static constexpr uint8_t cmd_nop = 0x00;
    static constexpr uint8_t cmd_software_reset = 0x01;
    static constexpr uint8_t cmd_read_display_identification_information = 0x04;
    static constexpr uint8_t cmd_read_display_status = 0x09;
    static constexpr uint8_t cmd_read_display_power_mode = 0x0A;
    static constexpr uint8_t cmd_read_display_madctl = 0x0B;
    static constexpr uint8_t cmd_read_display_pixel_format = 0x0C;
    static constexpr uint8_t cmd_read_display_image_format = 0x0D;
    static constexpr uint8_t cmd_read_display_signal_mode = 0x0E;
    static constexpr uint8_t cmd_read_display_self_diagnostic_result = 0x0F;
    static constexpr uint8_t cmd_enter_sleep_mode = 0x10;
    static constexpr uint8_t cmd_sleep_out = 0x11;
    static constexpr uint8_t cmd_partial_mode_on = 0x12;
    static constexpr uint8_t cmd_normal_display_mode_on = 0x13;
    static constexpr uint8_t cmd_display_inversion_off = 0x20;
    static constexpr uint8_t cmd_display_inversion_on = 0x21;
    static constexpr uint8_t cmd_gamma_set = 0x26;
    static constexpr uint8_t cmd_display_off = 0x28;
    static constexpr uint8_t cmd_display_on = 0x29;
    static constexpr uint8_t cmd_column_address_set = 0x2A;
    static constexpr uint8_t cmd_page_address_set = 0x2B;
    static constexpr uint8_t cmd_memory_write = 0x2C;
    static constexpr uint8_t cmd_color_set = 0x2D;
    static constexpr uint8_t cmd_memory_read = 0x2E;
    static constexpr uint8_t cmd_partial_area = 0x30;
    static constexpr uint8_t cmd_vertical_scrolling_definition = 0x33;
    static constexpr uint8_t cmd_tearing_effect_line_off = 0x34;
    static constexpr uint8_t cmd_tearing_effect_line_on = 0x35;
    static constexpr uint8_t cmd_memory_access_control = 0x36;
    static constexpr uint8_t cmd_vertical_scrolling_start_address = 0x37;
    static constexpr uint8_t cmd_idle_mode_off = 0x38;
    static constexpr uint8_t cmd_idle_mode_on = 0x39;
    static constexpr uint8_t cmd_pixel_format_set = 0x3A;
    static constexpr uint8_t cmd_write_memory_continue = 0x3C;
    static constexpr uint8_t cmd_read_memory_continue = 0x3E;
    static constexpr uint8_t cmd_set_tear_scanline = 0x44;
    static constexpr uint8_t cmd_get_scanline = 0x45;
    static constexpr uint8_t cmd_write_display_brightness = 0x51;
    static constexpr uint8_t cmd_read_display_brightness = 0x52;
    static constexpr uint8_t cmd_write_ctrl_display = 0x53;
    static constexpr uint8_t cmd_read_ctrl_display = 0x54;
    static constexpr uint8_t cmd_write_content_adaptive_brightness_control = 0x55;
    static constexpr uint8_t cmd_read_content_adaptive_brightness_control = 0x56;
    static constexpr uint8_t cmd_write_cabc_minimum_brightness = 0x5E;
    static constexpr uint8_t cmd_read_cabc_minimum_brightness = 0x5F;
    static constexpr uint8_t cmd_read_id1 = 0xDA;
    static constexpr uint8_t cmd_read_id2 = 0xDB;
    static constexpr uint8_t cmd_read_id3 = 0xDC;
    static constexpr uint8_t cmd_gb_interface_signal_control = 0xB0;
    static constexpr uint8_t cmd_rame_rate_control_in_normal_mode = 0xB1;
    static constexpr uint8_t cmd_rame_rate_control_in_idle_mode = 0xB2;
    static constexpr uint8_t cmd_rame_rate_control_in_partial_mode = 0xB3;
    static constexpr uint8_t cmd_isplay_inversion_control = 0xB4;
    static constexpr uint8_t cmd_lanking_porch_control = 0xB5;
    static constexpr uint8_t cmd_isplay_function_control = 0xB6;
    static constexpr uint8_t cmd_ntry_mode_set = 0xB7;
    static constexpr uint8_t cmd_acklight_control_1 = 0xB8;
    static constexpr uint8_t cmd_backlight_control_2 = 0xB9;
    static constexpr uint8_t cmd_backlight_control_3 = 0xBA;
    static constexpr uint8_t cmd_backlight_control_4 = 0xBB;
    static constexpr uint8_t cmd_backlight_control_5 = 0xBC;
    static constexpr uint8_t cmd_backlight_control_7 = 0xBE;
    static constexpr uint8_t cmd_backlight_control_8 = 0xBF;
    static constexpr uint8_t cmd_power_control_1 = 0xC0;
    static constexpr uint8_t cmd_power_control_2 = 0xC1;
    static constexpr uint8_t cmd_vcom_control_1 = 0xC5;
    static constexpr uint8_t cmd_vcom_control_2 = 0xC7;
    static constexpr uint8_t cmd_nv_memory_write = 0xD0;
    static constexpr uint8_t cmd_nv_memory_protection_key = 0xD1;
    static constexpr uint8_t cmd_nv_memory_status_read = 0xD2;
    static constexpr uint8_t cmd_read_id4 = 0xD3;
    static constexpr uint8_t cmd_positive_gamma_correction = 0xE0;
    static constexpr uint8_t cmd_negative_gamma_correction = 0xE1;
    static constexpr uint8_t cmd_digital_gamma_control_1 = 0xE2;
    static constexpr uint8_t cmd_digital_gamma_control_2 = 0xE3;
    static constexpr uint8_t cmd_interface_control = 0xF6;
    static constexpr uint8_t cmd_power_control_a = 0xCB;
    static constexpr uint8_t cmd_power_control_b = 0xCF;
    static constexpr uint8_t cmd_driver_timing_control_a_0 = 0xE8;
    static constexpr uint8_t cmd_driver_timing_control_a_1 = 0xE9;
    static constexpr uint8_t cmd_driver_timing_control_b = 0xEA;
    static constexpr uint8_t cmd_power_on_sequence_control = 0xED;
    static constexpr uint8_t cmd_enable_3g = 0xF2;
    static constexpr uint8_t cmd_pump_ratio_control = 0xF7;

    void initialize(spi_inst_t* spi, int sck, int rx, int tx, int cs, int dc, int reset);

    void setRange(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1);
    void setColumnRange(uint16_t x0, uint16_t x1);
    void setRowRange(uint16_t y0, uint16_t y1);
    void writePixels(const uint16_t* pixels, int count, bool continuous);
    void writePixelsDma(const uint16_t* pixels, int count, bool continuous);

    void sendCommand(uint8_t cmd, const void* data, int databytes);
    void sendCommand(uint8_t cmd, std::initializer_list<const uint8_t> data);
    void sendCommand(uint8_t cmd);
    void sendCommandDma(uint8_t cmd, const void* data, int databytes);
    void readCommand(uint8_t cmd, void* receive, int receivebytes);
private:
    int m_pin_cs = 0;
    int m_pin_dc = 0;
    int m_pin_reset = 0;
    spi_inst_t* m_spi = nullptr;

    void initDriver();
    void write(uint8_t);
    int write(const void *src, int bytes);
    uint8_t read();
    int read(void* dst, int bytes);

    static void dma_handler();
};