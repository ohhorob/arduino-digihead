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
            result = _currentPacket;
        }
    }
    return result;
}