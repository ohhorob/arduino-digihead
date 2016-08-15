
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

#define MTS_AUX_MAX 0x3FF

#define MTS_CMD_HIBITS 0b0111111100000000
#define MTS_CMD_LOBITS 0b0000000001111111

// Documented, but not serviced by SSI-4
//#define MTS_CMD_NAME        0xCE
//#define MTS_CMD_DEVICE_TYPE 0xF3
#define MTS_CMD_NOOP 0xFF
// Sniffed from LM Programmer
#define MTS_CMD_SETUP_START 0x53
// ANSWER IS 0x10 0F 53 53 49 34 05 04 FC 00 00 00 05 00 00
//                    S  S  I  4
//           VERSION ID<<<<<<<<< RESERVED<<<<<<<<<<<<<<<<<<
//                               (LM Programmer fields)
#define MTS_CMD_SETUP_STOP 's'

#define MTS_CMD_CONFIG_GET 0x63
// 0x53_53 63_6E CE_F3 53_63 6E_CE F3_53 63_6E CE F3 53 63 6E CE F3 53 63 6E CE 6D 66 CE 66 65 4E D3 34 66 FE 1B 0F 00 00 00 CB 89 00 40 FF 39 00 FF 39 60 29 E0 69 E0 19 D4 06 F8 03 F0 09 F0 09
//   S  S  c  n        S              S              S              S
#define MTS_CMD_CONFIG_SET 0x43
// and send complete config for all channels

#define MTS_CMD_NAME_GET 0x6E
// Name
// ANSWER IS 0x53 53 49 34 00 00 00 00
//             S  S  I   4 .. .. .. .. (padded to 8 bytes)
#define MTS_CMD_NAME_SET 0x4E
// and send 8 ascii bytes; no terminator
// no response; query name to confirm change

#define MTS_CMD_LASTPACKET 'b'

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

    static Packet name();

    Packet();

    // There must be enough bytes in the buffer to decode the whole packet
    void build(byte *buffer);

    MTS::Type type = MTS::Type::RESPONSE;

    // Sensor
    boolean recording = false;

    MTS::Funtion function = MTS::Funtion::RESERVED;
    uint16_t lambda = 0;
    uint8_t channelCount = 0;

    uint32_t channel[MTS_MAXCHANNEL] = {0,0,0,0,0,0,0,0};

    // Command
    uint16_t command = 0;

    byte *response = nullptr;

private:
    void _buildSensorPacket(byte *buffer, uint8_t len);

    void _buildResponsePacket(byte *buffer, uint8_t len);
    uint8_t _afrMultiplier = 0;
};


#endif //DIGIHEAD_PACKET_H
