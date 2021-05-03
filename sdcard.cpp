#include <stdio.h>
#include <pico/time.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include "sdcard.h"

constexpr uint8_t spi_fill_byte  = 0xff;

constexpr uint8_t r1_no_received = (1<<7);
constexpr uint8_t r1_parameter_error = (1<<6);
constexpr uint8_t r1_address_error = (1<<5);
constexpr uint8_t r1_erase_sequence_error = (1<<4);
constexpr uint8_t r1_com_crc_error = (1<<3);
constexpr uint8_t r1_illegal_command = (1<<2);
constexpr uint8_t r1_erase_reset = (1<<1);
constexpr uint8_t r1_in_idle = (1<<0);

constexpr uint8_t cmd_start_bit = 0x40;

constexpr uint8_t write_status_mask = 0xe;
constexpr uint8_t write_data_accepted = 0x4;
constexpr uint8_t write_crc_error = 0xa;
constexpr uint8_t write_rejected = 0xb;

constexpr uint8_t read_token_start_block = 0xfe;
constexpr uint8_t write_token_start_block = 0xfc;
constexpr uint8_t write_token_stop_block = 0xfd;

constexpr uint32_t block_size = 512;


#if 0
#define debug_print(...)    printf(__VA_ARGS__)
#else
#define debug_print(...)    
#endif


struct CSDV1 {
  // byte 0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1
  unsigned char taac;
  // byte 2
  unsigned char nsac;
  // byte 3
  unsigned char tran_speed;
  // byte 4
  unsigned char ccc_high;
  // byte 5
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6
  unsigned char c_size_high : 2;
  unsigned char reserved2 : 2;
  unsigned char dsr_imp : 1;
  unsigned char read_blk_misalign : 1;
  unsigned char write_blk_misalign : 1;
  unsigned char read_bl_partial : 1;
  // byte 7
  unsigned char c_size_mid;
  // byte 8
  unsigned char vdd_r_curr_max : 3;
  unsigned char vdd_r_curr_min : 3;
  unsigned char c_size_low : 2;
  // byte 9
  unsigned char c_size_mult_high : 2;
  unsigned char vdd_w_cur_max : 3;
  unsigned char vdd_w_curr_min : 3;
  // byte 10
  unsigned char sector_size_high : 6;
  unsigned char erase_blk_en : 1;
  unsigned char c_size_mult_low : 1;
  // byte 11
  unsigned char wp_grp_size : 7;
  unsigned char sector_size_low : 1;
  // byte 12
  unsigned char write_bl_len_high : 2;
  unsigned char r2w_factor : 3;
  unsigned char reserved3 : 2;
  unsigned char wp_grp_enable : 1;
  // byte 13
  unsigned char reserved4 : 5;
  unsigned char write_partial : 1;
  unsigned char write_bl_len_low : 2;
  // byte 14
  unsigned char reserved5: 2;
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  /** Indicates the file format on the card */
  unsigned char file_format_grp : 1;
  // byte 15
  unsigned char always1 : 1;
  unsigned char crc : 7;
} __attribute__((packed));

//==============================================================================
/**
 * \class CSDV2
 * \brief CSD register for version 2.00 cards.
 */
struct  CSDV2 {
  // byte 0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1
  /** fixed to 0X0E */
  unsigned char taac;
  // byte 2
  /** fixed to 0 */
  unsigned char nsac;
  // byte 3
  unsigned char tran_speed;
  // byte 4
  unsigned char ccc_high;
  // byte 5
  /** This field is fixed to 9h, which indicates READ_BL_LEN=512 Byte */
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6
  /** not used */
  unsigned char reserved2 : 4;
  unsigned char dsr_imp : 1;
  /** fixed to 0 */
  unsigned char read_blk_misalign : 1;
  /** fixed to 0 */
  unsigned char write_blk_misalign : 1;
  /** fixed to 0 - no partial read */
  unsigned char read_bl_partial : 1;
  // byte 7
  /** high part of card size */
  unsigned char c_size_high : 6;
  /** not used */
  unsigned char reserved3 : 2;
  // byte 8
  /** middle part of card size */
  unsigned char c_size_mid;
  // byte 9
  /** low part of card size */
  unsigned char c_size_low;
  // byte 10
  /** sector size is fixed at 64 KB */
  unsigned char sector_size_high : 6;
  /** fixed to 1 - erase single is supported */
  unsigned char erase_blk_en : 1;
  /** not used */
  unsigned char reserved4 : 1;
  // byte 11
  unsigned char wp_grp_size : 7;
  /** sector size is fixed at 64 KB */
  unsigned char sector_size_low : 1;
  // byte 12
  /** write_bl_len fixed for 512 byte sectors */
  unsigned char write_bl_len_high : 2;
  /** fixed value of 2 */
  unsigned char r2w_factor : 3;
  /** not used */
  unsigned char reserved5 : 2;
  /** fixed value of 0 - no write protect groups */
  unsigned char wp_grp_enable : 1;
  // byte 13
  unsigned char reserved6 : 5;
  /** always zero - no partial sector read*/
  unsigned char write_partial : 1;
  /** write_bl_len fixed for 512 byte sectors */
  unsigned char write_bl_len_low : 2;
  // byte 14
  unsigned char reserved7: 2;
  /** Do not use always 0 */
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  /** Do not use always 0 */
  unsigned char file_format_grp : 1;
  // byte 15
  /** not used always 1 */
  unsigned char always1 : 1;
  /** checksum */
  unsigned char crc : 7;
} __attribute__((packed));
//==============================================================================
/**
 * \class csd_t
 * \brief Union of old and new style CSD register.
 */
union CSDV {
  CSDV1 v1;
  CSDV2 v2;
};


struct cs_active_scoped
{
    cs_active_scoped(int cs)
        : cs(cs)
    {
        gpio_put(cs, false);
    }
    ~cs_active_scoped()
    {
        gpio_put(cs, true);
    }

    int cs = 0;
};

template<typename Fn> bool timeout(Fn&& fn, uint32_t us)
{
    auto start_time = get_absolute_time();
    do{
        if(fn()) 
            return false;
    }while(absolute_time_diff_us(start_time, get_absolute_time()) < us);

    debug_print("timeouted\n");

    return true;
}


bool SDCard::initialize(spi_inst_t* spi, int sck, int rx, int tx, int cs)
{
    m_spi = spi;
    m_pin_cs = cs;

    spi_init(spi, 1*MHZ);
    gpio_set_function(rx,   GPIO_FUNC_SPI);
    gpio_set_function(sck,  GPIO_FUNC_SPI);
    gpio_set_function(tx,   GPIO_FUNC_SPI);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, true);  

    return initialize_card();
}

bool SDCard::isBusy()
{
    return false;
}

bool SDCard::readSector(uint32_t sector, uint8_t* dst)
{
    return readSectors(sector, dst, 1);
}

bool SDCard::readSectors(uint32_t sector, uint8_t* dst, size_t ns)
{
    if(!m_sdhc)
        sector <<= 9;

    cs_active_scoped active(m_pin_cs);

    if(sendCmd(18, sector) != 0x00)
        return false;       

    for(int i = 0; i < ns; ++i, dst += block_size)
    {
        if(receivePacket(dst, block_size) == 0)
            return false;
    }

    stop();

    return true;
}

uint32_t SDCard::sectorCount() 
{
    cs_active_scoped active(m_pin_cs);

    if(sendCmd(9, 0) != 0)
        return 0;

    CSDV csd;
    if(!receivePacket(reinterpret_cast<uint8_t*>(&csd), 16))
        return 0;

    stop();

    if (csd.v1.csd_ver == 0) 
    {
        uint8_t read_bl_len = csd.v1.read_bl_len;
        uint16_t c_size = (csd.v1.c_size_high << 10)
                        | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
        uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
                            | csd.v1.c_size_mult_low;
        return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
    }
    else if (csd.v2.csd_ver == 1) 
    {
        return (((uint32_t)csd.v2.c_size_high << 16) +
            ((uint16_t)csd.v2.c_size_mid << 8) + csd.v2.c_size_low + 1) << 10;
    }
    
    return 0;
}

bool SDCard::syncDevice() 
{
    return true;
}

bool SDCard::writeSector(uint32_t sector, const uint8_t* src) 
{
    return writeSectors(sector, src, 1);
}

bool SDCard::writeSectors(uint32_t sector, const uint8_t* src, size_t ns) 
{
    if(!m_sdhc)
        sector <<= 9;

    cs_active_scoped active(m_pin_cs);

    if(sendCmd(25, sector) != 0)
        return false;

    for(int i = 0; i < ns; ++i, src += block_size)
    {
        send(write_token_start_block);
        send(src, block_size);
        send({0xFF, 0xFF});
        
        uint8_t status = 0;
        timeout([&]()
        {
            status = read()&write_status_mask;
            return (status == write_data_accepted || status == write_crc_error || status == write_rejected);
        }, 1000000);

        if(status != write_data_accepted)
            return false;
    }

    send(write_token_stop_block);

    timeout([&]()
    {
        return read() == 0xFF;
    }, 1000000);

    stop();

    return true;
}

int SDCard::send(uint8_t byte)
{
    return spi_write_blocking(m_spi, &byte, 1);
}

int SDCard::send(std::initializer_list<uint8_t> bytes)
{
    return spi_write_blocking(m_spi, bytes.begin(), bytes.size());
}

int SDCard::send(const uint8_t* src, size_t bytes)
{
    return spi_write_blocking(m_spi, src, bytes);
}

int SDCard::receiveResponse()
{
    uint8_t data;
    for(int i = 0; i < m_receive_retry_times; ++i)
    {
        data = read();
        if(!(data&r1_no_received))
            break;
    }

    debug_print("recv : %02x\n", data);

    return data;
}

int SDCard::receivePacket(uint8_t* dst, int bytes)
{
    bool timeouted = timeout([&]()
    {
        return read() == read_token_start_block;
    }, 1000000);

    if(timeouted)
        return 0;

    auto readBytes = read(dst, bytes);

    read();
    read();

    return readBytes;
}

bool SDCard::read(uint8_t* dst, int bytes)
{
    return spi_read_blocking(m_spi, spi_fill_byte, dst, bytes) != 0;
}

uint8_t SDCard::read()
{   
    uint8_t dst;
    read(&dst, 1);
    return dst;
}

void SDCard::stop()
{
    sendCmd(12, 0);
}

int SDCard::sendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc = 0xff;
    switch(cmd)
    {
        case 0: crc = 0x95; break;
        case 8: crc = 0x87; break;
    }

    uint8_t response = 0xff;
    for(int i = 0; i < m_cmd_retry_times; ++i)
    {
        uint8_t packet[6] = {
            (uint8_t)(cmd | cmd_start_bit)
            , (uint8_t)((arg>>24)&0xff)
            , (uint8_t)((arg>>16)&0xff)
            , (uint8_t)((arg>> 8)&0xff)
            , (uint8_t)((arg>> 0)&0xff)
            , crc
        };
        send({ packet[0], packet[1], packet[2], packet[3], packet[4], packet[5] });

        debug_print("cmd : %02x %02x %02x %02x %02x %02x\n",
            packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);

        response = receiveResponse();
        if(response == 0x00 || response == r1_in_idle)
            return response;

        sleep_us(100);
    }
    return response;
}

bool SDCard::initialize_card()
{
    spi_set_baudrate(m_spi, 100*1000);

    for(int i = 0; i < 0x10; ++i)
        send(spi_fill_byte);

    cs_active_scoped active(m_pin_cs);

    uint8_t response = 0;

    response = sendCmd(0, 0);
    if(response != r1_in_idle)
        return false;

    m_sdhc = false;
    response = sendCmd(8, 0x1AA);
    if(response == r1_in_idle)
        m_sdhc = true;

    auto timeouted = timeout([&](){
        return sendCmd(1, m_sdhc ? (1<<30) : 0) == 0x00;
    }, 1000000);
    if(timeouted)
        return false;

    if(!m_sdhc)
    {
        if(sendCmd(16, block_size) != 0x00)
            return false;
    }

    spi_set_baudrate(m_spi, 1000*1000);

    return true;
}
