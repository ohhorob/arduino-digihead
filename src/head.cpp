#include <Arduino.h>
#include <Display.h>

#define PRINT false

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

//#define HWSERIAL      Serial2
//#define HWSERIAL_BAUD 57600

// GPS updates

//#define GPSSERIAL      Serial2
//#define GPSSERIAL_BAUD ???

// Innovate MTS serial data
#include <Drive.h>
#define MTSSERIAL      Serial2

#define RECBUFF 64
byte mtsbuffer[RECBUFF];

// Keep a small buffer for reading bytes from MTSSERIAL
void maintainMTS(); // Feed available serial data to Drive packet buffer

Drive drive;
void maintainDrive(); // Detect newly arrived packets

#include <ADC.h>
// https://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1
#include "RingBuffer.h"
// and IntervalTimer
#include <IntervalTimer.h>

#define ADC_RESOLUTION 12
#define ADC_POT A7 // (21) ADC0_SE7B

ADC *adc = new ADC();
IntervalTimer timer0;
RingBuffer *buffer0 = new RingBuffer;
int startTimerValue0 = 0;
const int period0 = 10000; // us
// 50,000 is good for infrequent signals

void setupADC();
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
//#include <algorithm>

Bounce debouncer = Bounce();

void setupButton();
void maintainButton();

#define ANSI_ESCAPE 0x1B
#define ANSI_LEFT_BRACKET 0x5B
const uint8_t ERASE_DISPLAY[] = {ANSI_ESCAPE, ANSI_LEFT_BRACKET, '2', 'J'};

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
    display.setupChannel(0, 8191); // Lambda
    display.setupChannel(1, MTS10BIT_MAX); // AFM; up to ~4V
    display.setupChannel(2, 480); // O2; up to ~1.2V
    display.setupChannel(3, MTS10BIT_MAX);
//    display.setupChannel(4, MTS10BIT_MAX);

    // Waits until USB is connected
#if PRINT
    while(!Serial) {
        maintainLeds();
        delay(20);
    }
#endif // PRINT

    display.splash();

#if PRINT
    Serial.write(ERASE_DISPLAY, 4);

    Serial.println("Connected USB.");
#endif // PRINT

    MTSSERIAL.begin(MTSSERIAL_BAUD);
    while(!MTSSERIAL) {
        maintainLeds();
        delay(20);
    }
#if PRINT
    Serial.println("Connected MTS.");
#endif
    Leds[LED_BOARD].off();
    Leds[LED_RED].off();
    Leds[LED_GREEN].off();
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
}

// This function will be called with the desired frequency
// start the measurement
void timer0_callback(void) {
    adc->startSingleRead(ADC_POT, ADC_0); // also: startSingleDifferential, analogSynchronizedRead, analogSynchronizedReadDifferential
}

// when the measurement finishes, this will be called
// first: see which pin finished and then save the measurement into the correct buffer
//uint8_t isrTick = 0;
void adc0_isr() {
//    if (isrTick++ % 2) {
//        Leds[LED_GREEN].on();
//    } else {
//        Leds[LED_GREEN].off();
//    }

    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel

    // add value to correct buffer
    if(pin==ADC_POT) {
        buffer0->write(adc->readSingle());
        display.voltsReady(true);
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
}


/******* LOOP *******/

uint32_t lastLoop = millis();
void loop() {
    uint32_t now = millis();
    if (now - lastLoop > 10) {
        maintainMTS();
        maintainDrive();
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


// Each Led needs a `tick` to flash behaviour maintenance
void maintainLeds() {
    for (int i = 0; i < ledCount; i++) {
        Leds[i].tick();
    }
}

// When a new packet has been found, push parts of it into the display
// TODO: feed values into drive statistics (avg/min/max/etc)
void maintainDrive() {
    while(Packet *p = drive.nextPacket()) {
        // Process the packet by sending it's values to display for processing
        display.setChannel(0, p->lambda);
        if (p->channelCount >= 2) {
            display.setChannel(1, p->channel[1]);
            display.setChannel(2, p->channel[2]);
            display.setChannel(3, p->channel[3]);
//            display.setChannel(4, p->channel[4]);
        }
    }
    display.setElapsed(drive.elapsedMillis());
}


void maintainMTS() {
//    Leds[LED_BOARD].on();
    // TODO: Move serial input to interrupt timer
    int incomingAvailable = MTSSERIAL.available();
    if (incomingAvailable > 0) {
        Leds[LED_BOARD].on();
        uint8_t incomingRead = (uint8_t) MTSSERIAL.readBytes((char *) mtsbuffer, (size_t) incomingAvailable);
        if (incomingRead > 0) {
            // hand over bytes to packet buffer
            drive.addBytes((char *) mtsbuffer, incomingRead);
        }
        Leds[LED_BOARD].off();
    }
}