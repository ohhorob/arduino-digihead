
#ifndef DIGIHEAD_DRIVE_H
#define DIGIHEAD_DRIVE_H

#include "Packet.h"
#include "PacketBuffer.h"

#define MTSSERIAL_BAUD 19200
#define MTSSERIAL_PACKET_INTERVAL 81920

//#define MTS10BIT_MAX 0b0000001111111111
// Maximum of 10 bit number
// (1 << 10) - 1
#define MTS10BIT_MAX 0x3FF
#define MTS13BIT_MAX 0b0001111111111111

class Drive {
public:
    Drive();
    void addBytes(char *buffer, uint8_t len);
    Packet *nextPacket();

    uint32_t elapsedMillis();

private:
    PacketBuffer *_packetBuffer;
    Packet *_currentPacket;

    uint32_t _durationSeconds;
    uint64_t _durationNanos;
    void _incrementNanos(uint8_t packetCount);

    void _debugPacket();
};


#endif //DIGIHEAD_DRIVE_H
