
#ifndef DIGIHEAD_PACKET_H
#define DIGIHEAD_PACKET_H

#include "PacketBuffer.h"


#define MTS_HEADERMAGIC 0xA280
#define MTS_MAXCHANNEL 12

// TODO: enum function codes from docs
// TODO: cover methods for errorCode, tenthPercent and countdown

class Packet {
public:
    Packet(PacketBuffer *buffer);
    bool build();

    uint16_t lambda = 0;
    uint8_t channelCount = 0;
    uint32_t channel[MTS_MAXCHANNEL] = {0,0,0,0,0,0,0,0,0,0,0,0};

private:
    void _decode();
    PacketBuffer *_buffer; // Reference to parent ring-buffer

    uint8_t _byteLength = 0;

    uint8_t _function = 0;
    uint8_t _afrMultiplier = 0;
};


#endif //DIGIHEAD_PACKET_H
