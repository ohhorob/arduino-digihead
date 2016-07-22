/* Teensy 3.x, LC ADC library
 * https://github.com/pedvide/ADC
 * Copyright (c) 2015 Pedro Villanueva
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "PacketBuffer.h"
#include "Packet.h"

int PacketBuffer::PacketLength(uint16_t header) {
    uint8_t high = (header & 0b0000000100000000) >> 1;
    uint8_t low  = (header & 0b0000000001111111);
    uint8_t wordLength = high | low;
    return (wordLength * 2) + 2; // Double for packets; add two for header word inclusive
}

PacketBuffer::PacketBuffer()
{
    //ctor
    for (int i = 0; i < MTS_PACKETBUFFER_DEFAULT_BUFFER_SIZE; i++) {
        elems[i] = 0;
    }
}

PacketBuffer::~PacketBuffer()
{
    // dtor
}

int PacketBuffer::isFull() {
    return (b_end == (b_start ^ b_size));
}

int PacketBuffer::isEmpty() {
    return (b_end == b_start);
}

// TODO: change to a strategy of discarding new values that are not part of a packet
// TODO: instead of keeping it and advancing the start pointer
void PacketBuffer::write(int value) {
    // Track existing start byte for packet detection
    int previousbyte = elems[b_start];

    // Normal `write()` behaviour
//    elems[b_end&(b_size-1)] = value;
    elems[b_end] = value;
    if (isFull()) { /* full, overwrite moves start pointer */
        Serial.println("FULL");
        b_start = increase(b_start);
    }
    b_end = increase(b_end);
//    Serial.print("END="); Serial.println(b_end);

    // Packet header detection only runs if no packet is currently being accumulated
    if (p_len == 0) {
        // No packet header detected yet
        // Test if adding this value found a valid header
        // TODO: clean this up to be low-bytes only

        uint16_t startword = (previousbyte << 8) | value;
        if ((startword & MTS_HEADERMAGIC) == MTS_HEADERMAGIC) {
//            Serial.print(startword, HEX);
            p_len = PacketLength(startword); // add the header word bytes to the length
//            Serial.print(" p_len="); Serial.println(p_len, DEC);
        } else {
            // skip past the added value because it can't ever be used
            b_start = b_end - 1;
//            Serial.print("!! SKIP !! b_start = ");
//            Serial.println(b_start);
        }
    } // Call `hasPacket()` if you want to know if a complete packet is available
}

void PacketBuffer::write(byte *values, int len) {
    for (int i = 0; i < len; i++) {
//        Serial.print("<< "); Serial.println(values[i], HEX);
        write(values[i]);
    }
//    Serial.print("Start= "); Serial.print(b_start); Serial.print(" End= "); Serial.print(b_end);
//    Serial.print(" Available= "); Serial.println(_available(), DEC);

//    Serial.print("<< added "); Serial.println(len);
}

int PacketBuffer::read() {
    int result = elems[b_start&(b_size-1)];
    b_start = increase(b_start);
    return result;
}

uint16_t PacketBuffer::readWord() {
    uint16_t result = read() << 8;
    result |= read();
//    Serial.print("readWord() = 0x");
//    if (result <= 0x0FFF) Serial.print("0");
//    if (result <= 0x00FF) Serial.print("0");
//    if (result <= 0x000F) Serial.print("0");
//    Serial.println(result, HEX);
    return result;
}

const uint8_t ERASE_LINE[] = {0x1B, 0x5B, '2', 'K', '9', '9', '9', 'D'};
int _oops = 0;
int PacketBuffer::hasPacket() {
//    Serial.print("start, end: "); Serial.print(b_start); Serial.print(", "); Serial.println(b_end);
    if (elems[b_start] & 0xA2 && elems[increase(b_start)] & 0x80) {
        if (PacketLength(elems[b_start] << 8 | elems[increase(b_start)]) <= _available()) {
//            Serial.print("b_start is header; all bytes present; start=");
//            Serial.print(b_start); Serial.print(" end="); Serial.println(b_end);
//            if (_oops) {
//                Serial.println();
//            }
//            _oops = 0;
            return true;
//        } else if (_oops > 10) {
////            Serial.write(ERASE_LINE, 8);
//            Serial.print(_oops); Serial.print("!");
//            Serial.print("b_start = "); Serial.print(b_start);
//            Serial.print(" b_end = "); Serial.println(b_end);
//            _oops = 1;
//        } else if (_oops == 0) {
//            Serial.println("b_start is header; need more bytes");
        }
//        _oops++;
//    } else {
//        _oops = 0;
    }
//    Serial.print("b_start is not header. 0x");
//    Serial.println(elems[b_start], HEX);
    return false;
    // the length is set, and the head has reached or is past the len
//    return  (p_len > 0) && (_available() >= p_len);
}

// increases the pointer modulo 2*size-1
int PacketBuffer::increase(int p) {
    int newP = p + 1;
    if (newP > b_size-1) {
        newP -= (b_size - 1);
//        Serial.println("..wrapped pointer..");
    }
//    return (p + 1)&(2*b_size-1);
    return newP;
}

// == Wrapped availability ==
// 0 1 2 3 4 5 6 7 // 8 value buffer
// x x E x x S x x // start=5, end=2
// 3 4 5 - - 0 1 2 // valid value sequence; len=6
// len = 2 - 5           4 - 238
//     = -3              -234
// so, add size-1
//     = -3 + 8 - 1      -234 + 128
//     = 6

int PacketBuffer::_available() {
    // Difference between head and tail
    int len = b_end - b_start;
    if (len < 0) {
        // end has wrapped around
        len += b_size - 1;
    }
    return len;
}

void PacketBuffer::_debugPacket() {

    Serial.print("Packet: ");
    int b = b_start;
    for (uint8_t i = 0; i < p_len; i++, increase(b)) {
        Serial.print(elems[b], HEX);
        i++;
        increase(b);
        Serial.print(elems[b], HEX);
        Serial.print(' ');
    }
    Serial.println();
}
