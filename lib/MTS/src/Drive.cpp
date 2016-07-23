//
// Created by Rob Wills on 7/16/16.
//

#include "Drive.h"

Drive::Drive() {
    _packetBuffer = new PacketBuffer();
    _currentPacket = new Packet(_packetBuffer);
}

void Drive::addBytes(char *buffer, uint8_t len) {
    // Accumulate bytes into the packet buffer
    // It contains logic for knowing when a packet header is found, and how many bytes are required to build it
    _packetBuffer->write((byte *)buffer, len);
}

// Simple single-packet strategy for nextPacket()
// TODO: find a way to have a packet queue without having to do memory management?
// TODO: maybe keep a list of pointers to the packetbuffer where packets start?
Packet *Drive::nextPacket() {
    Packet *result = nullptr;
    // Buffer knows when it has available all the bytes for a full packet
    if (_packetBuffer->hasPacket()) {
        if (_currentPacket->build()) { // consumes from packetbuffer, extracting the values
            _incrementNanos(1);
            result = _currentPacket;
        }
    }
    return result;
}

// 1000 micros make a milli
const uint32_t DURATIONMILLI = 1000;
// 1000 millis make a full second
const uint32_t DURATIONSECOND = 1000 * DURATIONMILLI;
// 60 seconds make a minute
const uint32_t DURATIONMINUTE = 60 * DURATIONSECOND;

uint32_t Drive::elapsedMillis() {
    return (_durationSeconds * 1000) + (_durationMicros / DURATIONMILLI);
}

// MTSSERIAL_PACKET_INTERVAL is in micro seconds (1/1000 milli)
// - 81.92 millis == 81920 micros

const uint32_t DURATIONMICROROLL = 10 * DURATIONSECOND;
void Drive::_incrementNanos(uint8_t packetCount) {
    _durationMicros += (packetCount * MTSSERIAL_PACKET_INTERVAL);
    // Every minute, re-shuffle into seconds
    if (_durationMicros >= DURATIONMICROROLL ) {
        uint32_t seconds = _durationMicros / DURATIONSECOND;

//        Serial.print(_durationMicros);
//        Serial.print(": seconds=");
//        Serial.print(seconds);

        _durationSeconds += seconds;
        _durationMicros = (_durationMicros % DURATIONSECOND);

//        Serial.print(" micros=");
//        Serial.println(_durationMicros);
    }
}