

#include "Packet.h"

Packet::Packet(PacketBuffer *buffer) {
    _buffer = buffer;
}

bool Packet::build() {
    // Should only be called when the buffer has enough bytes to decode the packet
//    Serial.println("!! BUILD !!");
    _decode();
    return true;
}

void Packet::_decode() {
    // Header: recording, length
    uint16_t header = _buffer->readWord();
    _byteLength = PacketBuffer::PacketLength(header);
//    Serial.print("_decoding bytes len="); Serial.println(_byteLength);

    uint8_t b = 2;

    // Status: function, afr multiplier
    uint16_t status = _buffer->readWord();
    if ((status & 0x4200) == 0x4200) {
        _function =       (status & 0b0001110000000000) >> 10;
        _afrMultiplier  = (status & 0b0000000100000000) >>  8; // High bytes
        _afrMultiplier |= (status & 0b0000000001111111);       // Low bytes
        b += 2;

        // Lambda: 13 bit value with clear bit @7
        // h=00LLLLLL l=0LLLLLLLL
        // shift high down one, or simply bring it up by 7
        lambda  = _buffer->read() << 7; // High
        lambda |= _buffer->read();
        b += 2;

    } else {
        _function = 0;
        _afrMultiplier = 0;
        lambda = 0;
    }

    // Word by word each additional channel if present
    channelCount = 0;
    for(; b < _byteLength; b+=2, channelCount++) {
        // Aux n: 13 bit value
        uint16_t auxword = _buffer->readWord();
        uint32_t value = (auxword & 0xFF00) >> 1;
//        if (channelCount == 2) {
//            Serial.print(value, HEX); Serial.print(" + "); Serial.print(auxword & 0x00FF, HEX);
//        }
        value |= (auxword & 0x00FF);
        channel[channelCount] =  value;
//        if (channelCount == 2) {
//            Serial.print("; ch[");
//            Serial.print(channelCount);
//            Serial.print("] = ");
//
//            if (channel[channelCount] <= 0x0FFF) Serial.print("0");
//            if (channel[channelCount] <= 0x00FF) Serial.print("0");
//            if (channel[channelCount] <= 0x000F) Serial.print("0");
//            Serial.println(channel[channelCount], HEX);
//        }
    }
}