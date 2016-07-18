/* PacketBuffer is a copy of ADC/RingBuffer with buffer size increased
 *
 * Teensy 3.x, LC ADC library
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

#ifndef MTS_PACKETBUFFER_H
#define MTS_PACKETBUFFER_H

// include new and delete
#include <Arduino.h>

// THE SIZE MUST BE A POWER OF 2!!
#define MTS_PACKETBUFFER_DEFAULT_BUFFER_SIZE 128


/** Class PacketBuffer implements a circular buffer of fixed size (must be power of 2)
*   Code adapted from http://en.wikipedia.org/wiki/Circular_buffer#Mirroring
*/
class PacketBuffer {
public:
    static int PacketLength(uint16_t header);

    //! Default constructor, buffer has a size DEFAULT_BUFFER_SIZE
    PacketBuffer();

    /** Default destructor */
    virtual ~PacketBuffer();

    //! Returns 1 (true) if the buffer is full
    int isFull();

    //! Returns 1 (true) if the buffer is empty
    int isEmpty();

    //! Write a value into the buffer
    void write(int value);

    //! Convenience to dump a few bytes into the buffer
    void write(byte *values, int len);

    //! Read a value from the buffer
    int read();

    uint16_t readWord();

    int hasPacket();

protected:
private:

    int increase(int p);

    int _available();

    int b_size = MTS_PACKETBUFFER_DEFAULT_BUFFER_SIZE;
    int b_start = 0;
    int b_end = 0;
    //int *elems;
    // TODO: check if this element array can be changed to uint8_t or bytes to avoid wasting memory
    int elems[MTS_PACKETBUFFER_DEFAULT_BUFFER_SIZE];

    int p_len = 0;
    int p_avilable = false;

    void _debugPacket();
};


#endif // MTS_PACKETBUFFER_H
