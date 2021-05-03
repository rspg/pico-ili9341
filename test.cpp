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
#include "ili9341.h"

#include "SDFat/ExFatLib/ExFatVolume.h"
#include "libspng/spng.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_TFT_PORT spi0
#define PIN_TFT_RX   4
#define PIN_TFT_CS   5
#define PIN_TFT_SCK  2
#define PIN_TFT_TX   3
#define PIN_TFT_DC   0
#define PIN_TFT_RESET   15

#define SPI_SDC_PORT spi1
#define PIN_SDC_SCK  10
#define PIN_SDC_TX   11
#define PIN_SDC_RX   12
#define PIN_SDC_CS   13

#define HARDWARE_SPI        (1)

SDCard sdcard;
ExFatVolume filesys;
ILI9341 ili9341;


void tftInit()
{
    ili9341.initialize(SPI_TFT_PORT,
        PIN_TFT_SCK,
        PIN_TFT_RX,
        PIN_TFT_TX,
        PIN_TFT_CS,
        PIN_TFT_DC,
        PIN_TFT_RESET);
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

    filesys.printVolInfo(&print);

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


void dispPngImage()
{
    ili9341.setRange(0, 240, 0, 320);

    printf("open image\n");
    auto file = filesys.open("lion.png");

    auto png = spng_ctx_new(0);
    
    spng_set_png_stream(png, +[](spng_ctx *ctx, void *user, void *dest, size_t length){
        auto file = reinterpret_cast<ExFile*>(user);
        file->read(dest, length);
        return 0;
    }, &file);

    spng_decode_image(png, nullptr, 0, SPNG_FMT_RGB8, SPNG_DECODE_PROGRESSIVE);

    uint8_t outbuf[3*240];
    for(int i = 0; i < 320; ++i)
    {
        printf("decode %d\n", i);
        auto ret = spng_decode_scanline(png, outbuf, std::size(outbuf));
        if(ret != 0)
            break;

        uint8_t scanline[240*2];
        for(int j = 0; j < 240; ++j)
        {
            uint16_t c 
                = (((uint16_t)outbuf[j*3 + 0] << 8) & 0xf800) 
                | (((uint16_t)outbuf[j*3 + 1] << 3) & 0x07e0) 
                | (((uint16_t)outbuf[j*3 + 2] >> 3) & 0x001f);
            scanline[j*2 + 0] = c>>8;
            scanline[j*2 + 1] = c&0xff;               
        }
        ili9341.writePixelsDma(reinterpret_cast<uint16_t*>(scanline), 240, i!=0);
    }

    printf("done\n");
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
    
#else
    gpio_init(PIN_TFT_SCK);
    gpio_init(PIN_TFT_TX);
    gpio_init(PIN_TFT_RX);
    gpio_set_dir(PIN_TFT_SCK, GPIO_OUT);
    gpio_set_dir(PIN_TFT_TX, GPIO_OUT);
    gpio_set_dir(PIN_TFT_RX, GPIO_IN);
#endif    

    
    std::fill(std::begin(color_r), std::end(color_r), 0x00f8);
    std::fill(std::begin(color_g), std::end(color_g), 0xe007);
    std::fill(std::begin(color_b), std::end(color_b), 0x1f00);

    sleep_ms(1000);

    printf("clocks = %u\n", clock_get_hz(clk_sys));

    // reset
    tftInit();

    sdcInit();

    dispPngImage();
    
    //sendCommand(0xB5, {0x20, 0x00, 0x00, 0x00 });

#if 1
    int vsyncout = 0;
    int count = 0;
    int frames = 0;
    uint32_t us = time_us_64();
    int pregts = 0;
    while(false)
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
            for(int y = count&1; y < 320; y += 2)
            {
                ili9341.setRowRange(y, 8);
                ili9341.writePixels(reinterpret_cast<uint16_t*>(color), 240, false);
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

    while (true)
        tight_loop_contents();

    return 0;
}
