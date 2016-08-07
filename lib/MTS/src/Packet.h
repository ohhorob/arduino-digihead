
#ifndef DIGIHEAD_PACKET_H
#define DIGIHEAD_PACKET_H

//#define PACKETDEBUG

#include "ByteBuffer.h"

#define MTS_HEADERMAGIC      0xA280
#define MTS_HEADERTYPEBITS   0b0001000000000000
#define MTS_RECORDINGBITS    0b0100000000000000
#define MTS_HEADERLEN_HIBITS 0b0000000100000000
#define MTS_HEADERLEN_LOBITS 0b0000000001111111

#define MTS_FUNCTIONMAGIC 0x4200
#define MTS_FUNCTIONBITS  0b0001110000000000

#define MTS_AFR_HIBITS 0b0000000100000000
#define MTS_AFR_LOBITS 0b0000000001111111

#define MTS_CMD_HIBITS 0b0111111100000000
#define MTS_CMD_LOBITS 0b0000000001111111

// Actual Max is 32, but let's not get too carried away just yet
#define MTS_MAXCHANNEL 8

// TODO: enum function codes from docs
// TODO: cover methods for errorCode, tenthPercent and countdown
namespace MTS {
    enum Type : uint8_t {
        RESPONSE,
        SENSOR
    };
    enum Funtion : uint8_t {
        NORMAL,
        O2TENTHS,
        FREEAIR_CALIBRATION_STARTED,
        FREEAIR_CALIBRATION_NEEDED,
        WARMUP_STARTED,
        HEATER_CALIBRATION_STARTED,
        ERROR,
        RESERVED
    };
}

class Packet {
public:
    static uint8_t PacketLength(uint16_t header);

    Packet();

    // There must be enough bytes in the buffer to decode the whole packet
    void build(byte *buffer);

    MTS::Type type = MTS::Type::RESPONSE;

    // Sensor

    boolean recording = false;
    uint16_t lambda = 0;
    uint8_t channelCount = 0;
    uint32_t channel[MTS_MAXCHANNEL] = {0,0,0,0,0,0,0,0};

    // Command

    uint16_t command = 0;
    byte *response = nullptr;

private:

    void _buildSensorPacket(byte *buffer, uint8_t len);
    void _buildResponsePacket(byte *buffer, uint8_t len);

    MTS::Funtion _function = MTS::Funtion::RESERVED;
    uint8_t _afrMultiplier = 0;
};


#endif //DIGIHEAD_PACKET_H
