#ifndef DIGIHEAD_DISPLAY_H
#define DIGIHEAD_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_GFX.h>
#include <SPI.h>

// Global Screen State
typedef struct {
    uint32_t lastUpdate;
    bool needsUpdate;
    bool brightness;
    bool time;
    bool menu;
} TFTUpdate;

typedef struct {
    uint8_t mode; // Current mode to render
    uint8_t nextMode; // Change to this mode next
    uint32_t nextWhen; // After this millis has been reached
    bool changeModeOnDirection;
} TFTPager;

typedef struct {
    uint16_t x0;
    uint16_t y0;
    uint16_t height;
    uint16_t width;
    uint16_t maxWidth;
    uint16_t colour;
} TFTBar;

// TFT Hardware; requires SPI
// https://www.adafruit.com/products/358

class Display {
public:
    Display(int8_t cs=15, int8_t rs=17, int8_t rst=16, int8_t spi_sck=14);
    void maintain();
    void directionInput(int16_t delta);
    void pressInput(uint8_t button, boolean pressed);
    void setupBrightness(int8_t pwm=20);
    void changeBrightness(int16_t delta);
    void setBrightness(int16_t b=128);

    void splash();

private:
    Adafruit_ST7735* _tft;

    TFTUpdate* _tftUpdate;
    TFTPager* _pager;

    void _setBrightness(uint8_t b);
    uint8_t   _brightnessPWM;
    uint8_t   _brightness;
    TFTBar*   _brightnessBar;

    void _modeDrawBrightness();
    void _modeDrawMenu();
};

// Update interval

#define DISPLAY_INTERVAL_UPDATE 50

#define DISPLAY_BACKGROUND ST7735_BLACK

#endif //DIGIHEAD_DISPLAY_H
