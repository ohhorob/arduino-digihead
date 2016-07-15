#include <Arduino.h>
#include <Display.h>

// LED Arrangement
#include <Led.h>

Led *Leds; // = {13, 29, 28};
const int LED_BOARD = 0; // Pin 13 (on board)
const int LED_GREEN = 1; // Pin 18
const int LED_RED = 2; // Pin 19
int ledCount;

void maintainLeds();
void setupLeds();


#define TFT_LITE   20 // PWM backlight/brightness control

Display display = Display();

#include <Encoder.h>

// Encoder
#define ENC_A      22
#define ENC_B      23

// Note the 2000millis delay during constructor!
Encoder rotary(ENC_A, ENC_B);

void maintainRotary();

// Each Led needs a `tick` to flash behaviour maintenance
void maintainLeds() {
    for (int i = 0; i < ledCount; i++) {
        Leds[i].tick();
    }
}

void setupLeds() {
    int pins[3] = {13, 18, 19};
    Leds = Led::forPins(pins);
    ledCount = sizeof(Leds);
}

void setup() {
    setupLeds();

    // Indicate startup with all Leds
    Leds[LED_BOARD].flash(9999);
    Leds[LED_RED].flash(9999);
    Leds[LED_GREEN].flash();

    Serial.begin(19200);  // USB, communication to PC or Mac

    display.setupBrightness(TFT_LITE);

    // Waits until USB is connected
    while(!Serial) {
        maintainLeds();
        delay(20);
    }

    display.splash();

    Serial.println("Connected.");

    Leds[LED_BOARD].off();
    Leds[LED_RED].off();
}

void loop() {
    maintainLeds();
    maintainRotary();
    display.maintain();
}

int32_t previousValue;

void maintainRotary() {
    int32_t value;
    value = rotary.read();
    if (value != previousValue) {
        int16_t delta = (int16_t) (previousValue - value);
        display.changeBrightness(delta);
//        Serial.printf("Rotary: previousValue=%ld value=%ld, delta=%d\n", previousValue, value, delta);
        previousValue = value;
    }
}