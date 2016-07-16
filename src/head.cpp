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
#define MTSSERIAL      Serial3
#define MTSSERIAL_BAUD 19200

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

    // Waits until USB is connected
    while(!Serial) {
        maintainLeds();
        delay(20);
    }

    HWSERIAL.begin(HWSERIAL_BAUD);

    display.splash();

    Serial.println("Connected.");

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

uint32_t lastLoop = 0;
void loop() {
//    uint32_t now = millis();
//    if (now - lastLoop > 1000) {
//        maintainADC();
//        lastLoop = now;
//    }
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