#include <Arduino.h>
#include <Display.h>

#define PRINT true
//#define TESTMTS

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

// Innovate MTS serial data
#include <Drive.h>
#define MTSSERIAL      Serial2

#define RECBUFF 64
byte mtsbuffer[RECBUFF];

// Keep a small buffer for reading bytes from MTSSERIAL
void maintainMTS(); // Feed available serial data to Drive packet buffer
#ifdef TESTMTS
void testMTS();
#endif // TESTMTS

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

#include "TinyGPS.h"

// GPS updates

#define GPSSERIAL      Serial3
#define GPSSERIAL_BAUD 9600

TinyGPS gps;

void setupGps();
void maintainGps();

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
    // Aux1 => not connected
    // Aux2 => Injector Pulse Width
    display.setupChannel(1, MTS10BIT_MAX);
    // Aux3 => O2; up to ~1.2V
    display.setupChannel(2, 480);
    // Aux4 => AFM; up to ~4V
    display.setupChannel(3, MTS10BIT_MAX);

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

    setupGps();

    Leds[LED_BOARD].off();
    Leds[LED_RED].off();
    Leds[LED_GREEN].off();
#ifdef TESTMTS
    testMTS();
#endif // TESTMTS
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

void setupGps() {
    GPSSERIAL.begin(GPSSERIAL_BAUD);
}

/******* LOOP *******/

uint32_t lastLoop = millis();
void loop() {
    uint32_t now = millis();
    if (now - lastLoop > 10) {
        maintainMTS();
        maintainGps();
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


Packet *p;

// When a packets are available, push parts of it into the display
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

// Push all available MTS bytes to the drive
// if that results in packets being decoded, they will be buffered in the Drive packetbuffer!
void maintainMTS() {
    while (MTSSERIAL.available()) {
        uint8_t m = (uint8_t) (MTSSERIAL.read() & 0xFF);
        drive.encode(m);
    }
}

#ifdef TESTMTS
int incoming[] = {0x27, 0x39, 0x00, 0xB2, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x7A, 0xB2, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x7A};
//                  --    --    --    HH    HH    a0    a0    a1    a1    a2    a2    a3    a3    HH    HH    a0    a0    a1    a1    a2    a2    a3    a3
void testMTS() {
    size_t intSize = sizeof(int);
//    Serial.print("sizeof(int) == "); Serial.println(intSize, DEC);
//    Serial.print("sizeof(uint8_t) == "); Serial.println(sizeof(uint8_t), DEC);
//    Serial.print("sizeof(uint16_t) == "); Serial.println(sizeof(uint16_t), DEC);
    Serial.print("sizeof(Packet) == "); Serial.println(sizeof(Packet), DEC);

//    Serial.println("MTS::Function >");
//    for (uint8_t f = MTS::Funtion::NORMAL; f <= MTS::Funtion::RESERVED; f++) {
//        Serial.print(f < 0b100 ? f < 0b10 ? "0b00" : "0b0" : "0b"); Serial.println(f, BIN);
//    }
//    Serial.println();

    // Feed predefined byte sequences to verify packet decoding
    for(int i = 0; i < sizeof(incoming) / intSize; i++) {
        uint8_t in = (uint8_t) (incoming[i] & 0xFF);
        Serial.print(i < 10 ? "[0" : "["); Serial.print(i); Serial.print("] ");
        Serial.print(in > 0xF ? "0x" : "0x0");
        Serial.print(in, HEX); Serial.print(" > ");

        drive.encode(in);

        if (Packet *p = drive.nextPacket()) {
            Serial.print("FOUND PACKET >> ");
            Serial.print(p->channelCount); Serial.println(" channels");
        }
    }
    Serial.println("Finished testMTS");
}
#endif // TESTMTS

long lat, lon;
unsigned long fix_age, time, date, speed, course;
unsigned long chars;
unsigned short sentences, failed_checksum;
int year;
byte month, day, hour, minutes, second, hundredths;

void maintainGps() {
    while (GPSSERIAL.available()) {
        int g = GPSSERIAL.read();
        if (gps.encode(g)) {
            // Packet decoded.
            // statistics
            gps.stats(&chars, &sentences, &failed_checksum);
            // retrieves +/- lat/long in 100000ths of a degree
            gps.get_position(&lat, &lon, &fix_age);
            // time in hhmmsscc, date in ddmmyy
//                gps.get_datetime(&date, &time, &fix_age);
            // returns speed in 100ths of a knot
            speed = gps.speed();
            // course in 100ths of a degree
            course = gps.course();
            // split open date/time components
            gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &fix_age);

            display.setGPSData(lat, lon, course);
//            Serial.print(year); Serial.print("-"); Serial.print(month); Serial.print("-"); Serial.print(day);
//            Serial.print("T"); Serial.print(hour); Serial.print(":"); Serial.print(minutes); Serial.print(" $ ");
//            Serial.print(chars); Serial.print(":"); Serial.print(sentences); Serial.print(":"); Serial.print(failed_checksum);
//            Serial.print("> lat/long  = "); Serial.print(lat); Serial.print("/"); Serial.print(lon);
//            Serial.print(" course = "); Serial.print(course);
//            Serial.println();
        }
    }
}