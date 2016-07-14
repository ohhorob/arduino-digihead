#include <Arduino.h>
#include <SPI.h>


// LED Arrangement
#include "Led.h"

Led *Leds; // = {13, 29, 28};
const int LED_BOARD = 0; // Pin 13 (on board)
const int LED_GREEN = 1; // Pin 18
const int LED_RED = 2; // Pin 19
int ledCount;

void maintainLeds();
void setupLeds();


// TFT Hardware; requires SPI
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define SPI_SCK    14
#define TFT_CS     15
#define TFT_RST    16
#define TFT_DC     17
#define TFT_LITE   20 // PWM backlight/brightness control

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Global Screen State
int brightness = 0xFE;

void setupTft();
void tftWelcome();
void maintainTft();

#include <Encoder.h>

// Encoder
#define ENC_A      22
#define ENC_B      23

Encoder rotary(ENC_A, ENC_B);

long previousValue = 0;

void maintainKnob();

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

    setupTft();

    // Waits until USB is connected
    while(!Serial)
        maintainLeds();

    tftWelcome();
    Serial.println("Connected.");

    Leds[LED_BOARD].off();
    Leds[LED_RED].off();
}

void loop() {
    maintainLeds();
    maintainKnob();
    maintainTft();
}



void setupTft() {
    SPI.setSCK(SPI_SCK);

    // Use this initializer (uncomment) if you're using a 1.44" TFT
    tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, green tab
    tft.fillScreen(ST7735_BLACK);

    pinMode(TFT_LITE, OUTPUT);
    analogWrite(TFT_LITE, brightness);
}

void tftWelcome() {
    tft.setTextWrap(false);
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(0, 30);
    tft.setTextColor(ST7735_YELLOW);
    tft.setTextSize(2);
    tft.println("DigiHead");
}

void maintainTft() {

}


void maintainKnob() {
    long value;
    value = rotary.read();
    if (value != previousValue) {
        brightness += (previousValue - value) * (1 + (int)(brightness * 0.075));
        if (brightness > 0xFE) {
            brightness = 0xFE;
            Leds[LED_BOARD].flash();
        } else if (brightness < 0) {
            brightness = 0;
            Leds[LED_RED].flash();
        }
        Serial.print("Knob: value=");
        Serial.print(value, DEC);
        Serial.print(" delta=");
        Serial.print(previousValue - value);
        Serial.print(" brightness=");
        Serial.println(brightness, DEC);
        previousValue = value;
        // TODO: change to setting TFT 'update-backlight' flag for next screen maintenance cycle
        analogWrite(TFT_LITE, brightness);
//        manager.updateBrightness(brightness);
    }
}