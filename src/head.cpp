#include <Arduino.h>
#include <Display.h>

// Pins in use
/**
 *  00 RX1 ..(CP2102)
 *  01 TX1 ..
 *  02                        * 23 ENC_B - Rotary B
 *  03                        * 22 ENC_A - Rotary A
 *  04                        * 21 Potentiometer
 *  05                        * 20 PWM - TFT:LITE <grey?>
 *  06 ENC_BTN <orange>       * 19 LED_GREEN <purple> 330ohm
 *  07 RX3 MTS <green>        * 18 LED_RED   <blue>   220ohm
 *  08 TX3 MTS <yellow>       * 17 TFT:RS  <blue>   D/C
 *  09 RX2 .. (GPS)           * 16 TFT:RST <green>  RST
 *  10 TX2 .. (GPS)           * 15 SPI:CS  <yellow> TCS
 *  11 SPI:DOUT <orange> SI   * 14 SPI:SCK <grey?>  SCK
 *  12 SPI:DIN  <red>    SO   * 13 LED_BOARD
 *
 *  [[ underside  ]]
 *
 *  ** not used **
 */
//

// UART Serial communications

// Host comms for data exchange with computer

#define HWSERIAL      Serial2
#define HWSERIAL_BAUD 57600

// GPS updates

//#define GPSSERIAL      Serial2
//#define GPSSERIAL_BAUD ???

// Innovate MTS serial data
#define MTSSERIAL      Serial2
#define MTSSERIAL_BAUD 19200

void maintainMTS();
void buildPacket(byte *buffer, uint8_t len);
#define RECBUFF 256
byte mtsbuffer[RECBUFF];

#include <ADC.h>
#define RING_BUFFER_DEFAULT_BUFFER_SIZE 32
#include "RingBuffer.h"
// and IntervalTimer
#include <IntervalTimer.h>

#define ADC_RESOLUTION 12
#define ADC_POT A7 // (21) ADC0_SE7B

ADC *adc = new ADC();
IntervalTimer timer0;
RingBuffer *buffer0 = new RingBuffer;
int startTimerValue0 = 0;
const int period0 = 1000; // us

void setupADC();
void maintainADC();
void timer0_callback();

// LED Arrangement
#include <Led.h>

Led *Leds; // = {13, 18, 19};
const int LED_BOARD = 0; // Pin 13 (on board)
const int LED_GREEN = 1; // 18 <purple> GREEN
const int LED_RED = 2; // 19 <blue> RED
int ledCount;

void maintainLeds();
void setupLeds();


#define TFT_LITE   20 // PWM backlight/brightness control

Display display = Display();

#include <Encoder.h>

// Encoder
#define ENC_A      22
#define ENC_B      23
#define ENC_BUTTON 06

// Note the 2000millis delay during constructor!
Encoder rotary(ENC_A, ENC_B);

void maintainRotary();

#include <Bounce2.h>
#include <algorithm>

Bounce debouncer = Bounce();

void setupButton();
void maintainButton();

void setup() {
    setupLeds();
    setupButton();
    setupADC();
    display.setupVolts(buffer0, adc->getMaxValue(0));

    // Indicate startup with all Leds
    Leds[LED_BOARD].flash(9999);
    Leds[LED_RED].flash(9999);
    Leds[LED_GREEN].flash(10);

    Serial.begin(19200);  // USB, communication to PC or Mac

    display.setupBrightness(TFT_LITE);
    display.setupChannel(1, 0xFFF);
    display.setupChannel(2, 512);

    // Waits until USB is connected
    while(!Serial) {
        maintainLeds();
        delay(20);
    }

    display.splash();

    Serial.println("Connected USB.");

    MTSSERIAL.begin(MTSSERIAL_BAUD);
    while(!MTSSERIAL) {
        maintainLeds();
        delay(20);
    }

    Serial.println("Connected MTS.");

    Leds[LED_BOARD].off();
    Leds[LED_RED].off();
    // Let green led flash the whole ten count
}

/**
 * Configure the Leds[] collection
 */
void setupLeds() {
    int pins[3] = {13, 18, 19};
    Leds = Led::forPins(pins);
    ledCount = sizeof(Leds);
}

/**
 * Configure debuouncer.
 * - 10 millis of debounce
 */
void setupButton() {
    // Pullup requires the button connect the pin to ground
    debouncer.attach(ENC_BUTTON, INPUT_PULLUP);
    debouncer.interval(10);
}

/**
 * Configure analog input.
 * - Use a timer to do DMA reads to a buffer
 */
void setupADC() {
    pinMode(ADC_POT, INPUT);
    adc->setReference(ADC_REF_3V3, ADC_0);
    adc->setAveraging(16);
    adc->setResolution(ADC_RESOLUTION);
    adc->setConversionSpeed(ADC_MED_SPEED);
    adc->setSamplingSpeed(ADC_MED_SPEED);
    startTimerValue0 = timer0.begin(timer0_callback, period0);
    delayMicroseconds(250);
    adc->enableInterrupts(ADC_0);
    Serial.println("ADC Timers started");
}

// This function will be called with the desired frequency
// start the measurement
void timer0_callback(void) {
    adc->startSingleRead(ADC_POT, ADC_0); // also: startSingleDifferential, analogSynchronizedRead, analogSynchronizedReadDifferential
}

// when the measurement finishes, this will be called
// first: see which pin finished and then save the measurement into the correct buffer
void adc0_isr() {

    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel

    // add value to correct buffer
    if(pin==ADC_POT) {
//        digitalWriteFast(ledPin+3, HIGH);
        buffer0->write(adc->readSingle());
        display.voltsReady(true);
//        digitalWriteFast(ledPin+3, LOW);
    } else { // clear interrupt anyway
        adc->readSingle();
    }

    // restore ADC config if it was in use before being interrupted by the analog timer
    if (adc->adc0->adcWasInUse) {
        // restore ADC config, and restart conversion
        adc->adc0->loadConfig(&adc->adc0->adc_config);
        // avoid a conversion started by this isr to repeat itself
        adc->adc0->adcWasInUse = (uint8_t) false;
    }
    //digitalWriteFast(ledPin+2, !digitalReadFast(ledPin+2));
}


/******* LOOP *******/

uint32_t lastLoop = millis();
void loop() {
    uint32_t now = millis();
    if (now - lastLoop > 80) {
        maintainMTS();
        lastLoop = now;
    }
    maintainLeds();
    maintainRotary();
    maintainButton();
    display.maintain();
}

int32_t previousValue;

void maintainRotary() {
    int32_t value;
    value = rotary.read();
    if (value != previousValue) {
        int16_t delta = (int16_t) (previousValue - value);
        display.directionInput(delta);
//        Serial.printf("Rotary: previousValue=%ld value=%ld, delta=%d\n", previousValue, value, delta);
        previousValue = value;
    }
}

bool wasPressed = false;
void maintainButton() {

    // Update the Bounce instance :
    debouncer.update();

    // Get the updated value :
    bool pressed = !debouncer.read();
    if (wasPressed != pressed) {
        display.pressInput(1, pressed);
        wasPressed = pressed;
        if (pressed) {
            Leds[LED_GREEN].on();
        } else {
            Leds[LED_GREEN].off();
        }
    }
}

void maintainADC() {

    Leds[LED_BOARD].on();
    if(startTimerValue0 == 0) {
        Serial.println("Timer0 setup failed");
    }
    if(!buffer0->isEmpty()) { // read the values in the buffer
        digitalWriteFast(13, HIGH);
        Serial.print("Read pin 0: ");
        Serial.println(buffer0->read()*3.3/adc->getMaxValue());
        digitalWriteFast(13, LOW);
    }
    Leds[LED_BOARD].off();
}


// Each Led needs a `tick` to flash behaviour maintenance
void maintainLeds() {
    for (int i = 0; i < ledCount; i++) {
        Leds[i].tick();
    }
}


void maintainMTS() {
//    Leds[LED_BOARD].on();
    int incomingAvailable = MTSSERIAL.available();
    if (incomingAvailable > 0) {
//        Leds[LED_BOARD].on();
        uint8_t incomingRead = (uint8_t) MTSSERIAL.readBytes((char *) mtsbuffer, (size_t) incomingAvailable);
        if (incomingRead > 0) {
            // hand over bytes to packet decoder
            buildPacket(mtsbuffer, incomingRead);
        }
//        Leds[LED_BOARD].off();
    }
}

uint16_t headerword = 0x0000;
byte packetbuffer[64];
uint8_t packethead;
uint8_t packetWords = 0;
uint8_t packetBytes = 0;
uint16_t channel[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

void _debugPacket() {
    Leds[LED_RED].on();
    Serial.print("Packet: ");
    for (uint8_t i = 0; i < packetBytes; i++) {
        Serial.print(packetbuffer[i], HEX);
        i++;
        Serial.print(packetbuffer[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
    Leds[LED_RED].off();
}

/**
 * Return how many channels were read
 */
uint8_t _decodePacket(byte *packet) {
    uint8_t b = 0;
    // Header: recording, length
    b += 2;

    // Status: function, afr multiplier
    uint16_t status = (packet[b] << 8) | packet[b+1];
    if ((status & 0x4200) == 0x4200) {
        uint8_t function = (status & 0b0001110000000000) >> 10;
        Serial.print(function, DEC);
        uint8_t af =       (status & 0b0000000100000000) >>  8;
                af |=      (status & 0b0000000001111111);
        b += 2;
        // Lambda: 13 bit value
        uint16_t lambda = (packet[b] << 8) | packet[b+1];
        Serial.print(" Lambda = ");
        Serial.print(lambda, DEC);
        b += 2;
    }
    // Word by word each additional channel if present
    uint8_t ch = 0;
    for(; b<packetBytes; b+=2, ch++) {
        // Aux n: 13 bit value
        channel[ch] = (packet[b] << 8) | packet[b+1];
        Serial.print("; ch[");
        Serial.print(ch);
        Serial.print("] = ");
        Serial.print(channel[ch]);
    }
    Serial.println();
    return ch;
}

void buildPacket(byte *buffer, uint8_t len) {
//    Serial.print("<< ");
//    Serial.println(len);
    for (uint8_t i = 0; i < len; i++) {
        if (packetWords == 0) {
            // Not mid-packet, so keep looking for a valid header
            headerword = (headerword << 8);
            headerword |= buffer[i];
            if ((headerword & 0xA280) == 0xA280) {
                // Valid packet header word available
                // bit8, bit6, .. bit0
                packetWords = (headerword & 0b0000000100000000) >> 1;
                packetWords |= (headerword & 0b0000000001111111);
                // header contains "word" length.. double it for bytes
                packetBytes = (uint8_t) (packetWords * 2 + 2);
                packetbuffer[0] = (byte) ((headerword & 0xFF00) >> 8);
                packetbuffer[1] = (byte) (headerword & 0x00FF);
                packethead = 2;
            }
        } else {
            // Header is in place; bytes are added to packet buffer
            packetbuffer[packethead++] = buffer[i];

            // Until the packet head matches the bytes required
            if (packethead >= packetBytes) {
                packetbuffer[packethead] = 0x00;
                // Dispatch packet buffer for consumption
//                _debugPacket();
                uint8_t channelCount = _decodePacket(packetbuffer);
                if (channelCount >= 2) {
                    display.setChannel(1, channel[1]);
                    display.setChannel(2, channel[2]);
                }
                // Reset packet building
                headerword = 0;
                packetWords = 0;
            }
        }
    }
}