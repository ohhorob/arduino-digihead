
#include "Packet.h"


MTS::Funtion &
operator++( MTS::Funtion& target )
{
    target = static_cast<MTS::Funtion >(target + 1);
    return target ;
}

Packet::Packet() {
}

uint8_t Packet::PacketLength(uint16_t header) {
    uint8_t wordLength = ((header & MTS_HEADERLEN_HIBITS) >> 1) | (header & MTS_HEADERLEN_LOBITS);
    return (wordLength * 2) + 2; // Double for packets; add two for header word inclusive
}

Packet Packet::name() {
    Packet command = Packet();
    command.command = MTS_CMD_NAME_GET;
    return command;
}

void Packet::build(byte *buffer) {
    // Should only be called when the buffer has enough bytes to decode the packet
    // Header: recording, length
    uint8_t b = 0;
    uint16_t header = buffer[b++] << 8 | buffer[b++];
    uint8_t len = PacketLength(header);

    recording = (header & MTS_RECORDINGBITS) == MTS_RECORDINGBITS;

    type = static_cast<MTS::Type >((header & MTS_HEADERTYPEBITS) >> 12);

    switch (type) {
        case MTS::Type ::RESPONSE:
            _buildResponsePacket(&buffer[b], len - 2);
            break;
        case MTS::Type ::SENSOR:
            _buildSensorPacket(&buffer[b], len - 2);
            break;
    }
}

void Packet::_buildSensorPacket(byte *buffer, uint8_t len) {

    channelCount = 0;

    uint8_t b = 0;
    while (b < len) {
        uint16_t w = buffer[b++] << 8 | buffer[b++];
        if ((w & MTS_FUNCTIONMAGIC) == MTS_FUNCTIONMAGIC) {
            // LC-2 function/status word
            function = static_cast<MTS::Funtion >((w & MTS_FUNCTIONBITS) >> 10);
            _afrMultiplier  = (w & MTS_AFR_HIBITS) >> 1;
            _afrMultiplier |= (w & MTS_AFR_LOBITS);

            // consume next word as LC-2 lambda
            lambda = buffer[b++] << 7 | buffer[b++]; // bit 7 is empty
#ifdef PACKETDEBUG
            Serial.print(" Function="); Serial.print(_function, BIN);
            Serial.print(" AFR="); Serial.print(_afrMultiplier);
            Serial.print(" Lambda="); Serial.println(lambda);
#endif // PACKETDEBUG
        } else {
            // Aux words
            channel[channelCount++] = (w & 0xFF00) >> 1 | (w & 0x00FF); // bit 7 is empty
#ifdef PACKETDEBUG
            Serial.print(" AUX"); Serial.print(channelCount - 1); Serial.print("=");
            Serial.print(channel[channelCount - 1]);
#endif // PACKETDEBUG
        }
    }
#ifdef PACKETDEBUG
    Serial.println();
#endif
}

void Packet::_buildResponsePacket(byte *buffer, uint8_t len) {
    uint16_t w = buffer[0] << 8 | buffer[1];
    command = ((w & MTS_CMD_HIBITS) >> 1) | (w & MTS_CMD_LOBITS);

}