#include "PacketBuffer.h"


PacketBuffer::PacketBuffer() {
    //ctor
    for (int i = 0; i < MTS_PACKETBUFFER_DEFAULT_BUFFER_SIZE; i++) {
        elems[i] = new Packet();
    }
    _bytes = (ByteBuffer*)malloc(sizeof(ByteBuffer));
    _bytes->init(64);
    p_len = 0;
}

PacketBuffer::~PacketBuffer() {
    // dtor
}


int PacketBuffer::isFull() {
    return (b_end == (b_start ^ b_size));
}


int PacketBuffer::isEmpty() {
    return (b_end == b_start);
}

void _showBuffer(ByteBuffer *buffer) {
    Serial.println(); Serial.print('{');
    for (uint8_t i = 0; i < buffer->getSize(); i++) {
        uint8_t b = buffer->peek(i);
        Serial.print(b < 0xF ? " 0" : " "); Serial.print(b, HEX);
    }
    Serial.println(" }");
}

void PacketBuffer::newByte(uint8_t value) {

    _bytes->put(value);

#ifdef PBDEBUG
    _showBuffer(_bytes);
#endif

    if (p_len == 0 && _bytes->getSize() > 1) {
        // Scanning to find the start of a packet (header word)

        // Assess the buffer for header word
        uint16_t startword = _bytes->peek(0) << 8 | value;
        if ((startword & MTS_HEADERMAGIC) == MTS_HEADERMAGIC) {
            p_len = Packet::PacketLength(startword); // packet length includes the two header bytes
#ifdef PBDEBUG
            Serial.print(startword, HEX);
            Serial.print(" p_len="); Serial.print(p_len, DEC);
#endif
        } else {
            // First word is not a header, reset the buffer
            _bytes->clear();
            // And initialise first byte to this one
            _bytes->put(value);
        }
    } else if (p_len > 0 && p_len <= _bytes->getSize()) {
        // The buffer contains enough bytes to decode a packet
        Packet *p = nextWrite();
        p->build(_bytes->raw());
        write(p);
        // Reset to start again
        _bytes->clear();
        p_len = 0;
    }
}

Packet *PacketBuffer::read() {
    Packet *result = elems[b_start&(b_size-1)];
    b_start = increase(b_start);
    return result;
}

// Return the pointer of the next write position
Packet *PacketBuffer::nextWrite() {
    int next = (b_end & (b_size-1));
    if (isFull()) {
        b_start = increase(b_start);
    }
    return elems[next];
}

void PacketBuffer::write(Packet *value) {
    elems[b_end&(b_size-1)] = value;
    if (isFull()) { /* full, overwrite moves start pointer */
        b_start = increase(b_start);
    }
    b_end = increase(b_end);
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
