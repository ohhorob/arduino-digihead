#include <Arduino.h>

#include "Led.h"

// LED Arrangement
Led *Leds; // = {13, 29, 28};
const int LED_BOARD = 0; // Pin 13 (on board)
const int LED_RED = 1; // Pin 28
const int LED_GREEN = 2; // Pin 29
int ledCount;

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

    // Waits until USB is connected
    while(!Serial)
        maintainLeds();

    Serial.println("Connected.");

    Leds[LED_GREEN].off();
}

void loop() {
    maintainLeds();
}