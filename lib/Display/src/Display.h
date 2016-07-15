#ifndef DIGIHEAD_DISPLAY_H
#define DIGIHEAD_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_GFX.h>
#include <SPI.h>

// Global Screen State
typedef struct {
    uint32_t lastUpdate;
    bool brightness;
    bool time;
} TFTUpdate;

// TFT Hardware; requires SPI
// https://www.adafruit.com/products/358

class Display {
public:
    Display(int8_t cs=15, int8_t rs=17, int8_t rst=16, int8_t spi_sck=14);
    void maintain();
    void setupBrightness(int8_t pwm=20);
    void changeBrightness(int16_t delta);
    void setBrightness(uint8_t b=128);

    void splash();

private:
    Adafruit_ST7735* _tft;

    TFTUpdate* _tftUpdate;

    void _setBrightness(uint8_t b);
    uint8_t   _brightnessPWM;
    uint8_t   _brightness;
};

// Update interval

#define DISPLAY_INTERVAL_UPDATE 200

#endif //DIGIHEAD_DISPLAY_H
