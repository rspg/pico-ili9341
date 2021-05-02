#include <stdio.h>
#include <initializer_list>
#include <utility>
#include <algorithm>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include "sdcard.h"
#include "print.h"

#include "SDFat/ExFatLib/ExFatVolume.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_TFT_PORT spi0
#define PIN_TFT_RX   4
#define PIN_TFT_CS   5
#define PIN_TFT_SCK  2
#define PIN_TFT_TX   3
#define PIN_TFT_DC   0

#define SPI_SDC_PORT spi1
#define PIN_SDC_SCK  10
#define PIN_SDC_TX   11
#define PIN_SDC_RX   12
#define PIN_SDC_CS   13

#define PIN_RESET   15

#define HARDWARE_SPI        (1)

SDCard sdcard;
ExFatVolume filesys;

void spiWrite(uint8_t value)
{
#if HARDWARE_SPI
    spi_write_blocking(SPI_TFT_PORT, &value, 1);
#else
    for(int i = 0; i < 8; ++i)
    {
        gpio_put(PIN_TFT_TX, (value&0x80) ? true : false);
        gpio_put(PIN_TFT_SCK, true);
        value <<= 1;
        gpio_put(PIN_TFT_SCK, false);
    }
#endif
}
void spiWrite(const uint8_t *values, int bytes)
{
#if HARDWARE_SPI
    spi_write_blocking(SPI_TFT_PORT, values, bytes);
#else
    for(int i = 0; i < bytes; ++i)
        spiWrite(values[i]);
#endif
}

uint8_t spiRead()
{
    uint8_t read = 0;
#if HARDWARE_SPI
    spi_read_blocking(SPI_TFT_PORT, 0x00, &read, 1);
#else
    for(int i = 0; i < 8; ++i)
    {
        gpio_put(PIN_TFT_SCK, true);
        read <<= 1;
        read |= gpio_get(PIN_TFT_RX) ? 1 : 0;
        gpio_put(PIN_TFT_SCK, false);
    }
#endif
    return read;
}
void spiReadArray(uint8_t* receive, int bytes)
{
#if HARDWARE_SPI
    spi_read_blocking(SPI_TFT_PORT, 0x00, receive, bytes);
#else
    for(int i = 0; i < bytes; ++i)
        receive[i] = spiRead();
#endif
}

void spiDummyClock()
{
#if !HARDWARE_SPI
    gpio_put(PIN_TFT_SCK, true);
    gpio_put(PIN_TFT_SCK, false);
#endif
}

void sendCommand(uint8_t cmd, const uint8_t* data, int databytes)
{
    gpio_put(PIN_TFT_CS, false);

    gpio_put(PIN_TFT_DC, false);
    spiWrite(cmd);

    gpio_put(PIN_TFT_DC, true);
    spiWrite(data, databytes);

    gpio_put(PIN_TFT_CS, true);
}

void sendCommand(uint8_t cmd, std::initializer_list<const uint8_t> data)
{
    sendCommand(cmd, data.begin(), data.size());
}

void sendCommand(uint8_t cmd)
{
    sendCommand(cmd, nullptr, 0);
}

void readCommand8(uint8_t cmd, uint8_t* receive, int receivebytes)
{
    gpio_put(PIN_TFT_CS, false);

    gpio_put(PIN_TFT_DC, false);
    spiWrite(cmd);

    gpio_put(PIN_TFT_DC, true);
    spiReadArray(receive, receivebytes);

    gpio_put(PIN_TFT_CS, true);
}

void tftInit()
{
    gpio_put(PIN_RESET, true);
    sleep_ms(1);
    gpio_put(PIN_RESET, false);
    sleep_ms(10);
    gpio_put(PIN_RESET, true);
    sleep_ms(120);

    sendCommand(0xEF, {0x03, 0x80, 0x02});
    sendCommand(0xCF, {0x00,0XC1,0X30});
    sendCommand(0xED, {0x64,0x03,0X12,0X81});
    sendCommand(0xE8, {0x85,0x00,0x78});
    sendCommand(0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02});
    sendCommand(0xF7, {0x20});
    sendCommand(0xEA, {0x00,0x00});
    sendCommand(0xC0, {0x23});
    sendCommand(0xC1, {0x10});
    sendCommand(0xC5, {0x3e, 0x28});
    sendCommand(0xC7, {0x86});
    sendCommand(0x36, {0x48});
    sendCommand(0x37, {0x00});
    sendCommand(0x3A, {0x55});
    sendCommand(0xB1, {0x00, 0x1f});
    sendCommand(0xB3, {0x00, 0x1f});
    sendCommand(0xB6, {0x08, 0x82, 0x27});
    sendCommand(0xBE, {0x00});
    sendCommand(0xF2, {0x00});
    sendCommand(0x26, {0x01});
    sendCommand(0xE0, {0x0F,0x3a,0x36,0x0b,0x0d,0x06,0x4c,0x91,0x31,0x08,0x10,0x04,0x11,0x0c,0x00});
    sendCommand(0xE1, {0x00,0x06,0x0a,0x05,0x12,0x09,0x2c,0x92,0x3f,0x08,0x0e,0x0b,0x2e,0x33,0x0F});
    sendCommand(0x11);
    sleep_ms(120);
    sendCommand(0x29);
}

void sdcInit()
{
    sdcard.initialize(SPI_SDC_PORT, 
        PIN_SDC_SCK,
        PIN_SDC_RX,
        PIN_SDC_TX,
        PIN_SDC_CS);

    PrintStdio print;
    print.println("TEST");

    bool ret;
    if(!filesys.begin(&sdcard))
        printf("filesys begin failed.\n");
        
    filesys.printFat(&print);
    

#if 0
    printf("sectors = %u\n", sdcard.sectorCount());

    uint8_t block[512];

    std::fill(std::begin(block), std::end(block), 0xe4);

    sdcard.writeSectors(0x1000, block, 1);

    std::fill(std::begin(block), std::end(block), 0x9e);

    
    if(sdcard.readSectors(0x1000, block, 1))
    {
        for(auto& i : block)
        {
            if((std::distance(block, &i)&511) == 0)
                printf("\n");
            printf("%02x", i);
        }
        printf("\n");
    }
#endif
    sleep_ms(1000);
}

uint16_t color_r[240];
uint16_t color_g[240];
uint16_t color_b[240];

int main()
{
    stdio_init_all();

    // clock_configure(clk_sys,
    //                 CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
    //                 CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    //                 48 * MHZ,
    //                 48 * MHZ);

#if HARDWARE_SPI
    spi_init(SPI_TFT_PORT, 100*MHZ);
    gpio_set_function(PIN_TFT_RX,   GPIO_FUNC_SPI);
    gpio_set_function(PIN_TFT_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_TFT_TX,   GPIO_FUNC_SPI);
    spi_set_format(SPI_TFT_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
#else
    gpio_init(PIN_TFT_SCK);
    gpio_init(PIN_TFT_TX);
    gpio_init(PIN_TFT_RX);
    gpio_set_dir(PIN_TFT_SCK, GPIO_OUT);
    gpio_set_dir(PIN_TFT_TX, GPIO_OUT);
    gpio_set_dir(PIN_TFT_RX, GPIO_IN);
#endif    
    gpio_init(PIN_TFT_CS);
    gpio_init(PIN_TFT_DC);
    gpio_set_dir(PIN_TFT_CS, GPIO_OUT);
    gpio_set_dir(PIN_TFT_DC, GPIO_OUT);
    gpio_put(PIN_TFT_CS, true);  
    
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    
    std::fill(std::begin(color_r), std::end(color_r), 0x00f8);
    std::fill(std::begin(color_g), std::end(color_g), 0xe007);
    std::fill(std::begin(color_b), std::end(color_b), 0x1f00);

    sleep_ms(1000);

    printf("clocks = %u\n", clock_get_hz(clk_sys));

    // reset
    tftInit();

    sdcInit();
    
    sendCommand(0x2A, {0x00, 0x64, 0x00, 0x96});
    sendCommand(0x2B, {0x00, 0x96, 0x00, 0xc8});
    sendCommand(0x2C, {0xf7, 0xdc, 0x00, 0x00, 0x00, 0x00});

    //sendCommand(0xB5, {0x20, 0x00, 0x00, 0x00 });
    sendCommand(0xB1, {0x00, 0x1f});
    sendCommand(0xB3, {0x00, 0x1f});

#if 1
    int vsyncout = 0;
    int count = 0;
    int frames = 0;
    uint32_t us = time_us_64();
    int pregts = 0;
    while(true)
    {

#if 0
        uint8_t receive[5] = {0};
        readCommand8(0x45, receive, 3);

        // printf("%02x%02x%02x ", 
        //     receive[0], receive[1], receive[2]);
        const int gts = ((((int)receive[0] << 16) | ((int)receive[1] << 8) | receive[2]) >> 7)&0x3ff;
        //printf("%d ", gts);
        if(gts < 320)
        {
            pregts = gts;
            ++vsyncout;
            continue;
        }

        if(!vsyncout)
            continue;
#endif

        uint8_t* color = nullptr;
        switch(count>>1)
        {
            case 0: color = reinterpret_cast<uint8_t*>(color_r); break;
            case 1: color = reinterpret_cast<uint8_t*>(color_g); break;
            case 2: color = reinterpret_cast<uint8_t*>(color_b); break;
        }

        uint64_t transfarSpent = time_us_64();
        if(color)
        {
            auto cursor = [](int x1, int y1)
            {
                int x2 = x1 + 240;
                int y2 = y1 + 8;
                sendCommand(0x2A, {(uint8_t)(x1>>8), (uint8_t)(x1&0xff), (uint8_t)(x2>>8), (uint8_t)(x2&0xff)});
                sendCommand(0x2B, {(uint8_t)(y1>>8), (uint8_t)(y1&0xff), (uint8_t)(y2>>8), (uint8_t)(y2&0xff)});
            };


            for(int y = count&1; y < 320; y += 2)
            {
                cursor(0, y);
                sendCommand(0x2C, color, 240*2);
            }

            transfarSpent = time_us_64() - transfarSpent;
        }
        
        if(++count >= 6)
            count = 0;
        
        if(++frames > 60)
            frames = 0;
        auto now = time_us_64();
        //printf("%u(%u) %d %d\n", (uint32_t)(now - us), (uint32_t)transfarSpent, pregts, frames);
        us = now;

        vsyncout = 0;

        //sleep_ms(1000);
    }
#else
    bool flip = false;
    while(true)
    {
        uint8_t receive[16] = {0};

        readCommand8(0x09, receive, 5);
        printf("%02x %02x %02x %02x %02x \n", 
             receive[0], receive[1], receive[2], receive[3], receive[4]);

        readCommand8(0x45, receive, 8);
        printf("%02x %02x %02x %02x %02x %02x %02x %02x \n", 
             receive[0], receive[1], receive[2], receive[3], receive[4], receive[5], receive[6], receive[7]);

        flip = !flip;
        sendCommand(0x36, { (uint8_t)(flip ? 0x48 : 0x00) });

        sleep_ms(1000);
    }
#endif

    return 0;
}
