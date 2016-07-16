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
 *  m
 */
//

#define MTSSERIAL Serial3

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

    display.splash();

    Serial.println("Connected.");

    Leds[LED_BOARD].off();
    Leds[LED_RED].off();
}

void setupLeds() {
    int pins[3] = {13, 18, 19};
    Leds = Led::forPins(pins);
    ledCount = sizeof(Leds);
}

void setupButton() {
    // Pullup requires the button connect the pin to ground
    debouncer.attach(ENC_BUTTON, INPUT_PULLUP);
    debouncer.interval(10);
}


/******* LOOP *******/

void loop() {
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


// Each Led needs a `tick` to flash behaviour maintenance
void maintainLeds() {
    for (int i = 0; i < ledCount; i++) {
        Leds[i].tick();
    }
}