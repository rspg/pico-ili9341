#pragma once

#include <hardware/spi.h>
#include <initializer_list>
#include "SDFat/common/BlockDeviceInterface.h"


class SDCard : public BlockDeviceInterface
{
public:
    bool initialize(spi_inst_t* spi, int sck, int rx, int tx, int cs);

    virtual bool isBusy() override;
    virtual bool readSector(uint32_t sector, uint8_t* dst) override;
    virtual bool readSectors(uint32_t sector, uint8_t* dst, size_t ns) override;
    virtual uint32_t sectorCount() override;
    virtual bool syncDevice() override;
    virtual bool writeSector(uint32_t sector, const uint8_t* src) override;
    virtual bool writeSectors(uint32_t sector, const uint8_t* src, size_t ns) override; 

private:
    spi_inst_t* m_spi = nullptr;
    int m_pin_cs = 0;
    int m_receive_retry_times = 8;
    int m_cmd_retry_times = 3;
    bool m_sdhc = false;

    int send(uint8_t byte);
    int send(std::initializer_list<uint8_t> bytes);
    int send(const uint8_t* src, size_t bytes);
    int receiveResponse();
    int receivePacket(uint8_t* dst, int bytes);
    bool read(uint8_t* dst, int bytes);
    uint8_t read();
    void stop();

    int sendCmd(uint8_t cmd, uint32_t arg);

    bool initialize_card();
};