#ifndef DIGIHEAD_DISPLAY_H
#define DIGIHEAD_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <RingBuffer.h>

#define INITR_144GREENTAB   0x1

// Update interval

#define DISPLAY_INTERVAL_UPDATE 50

#define DISPLAY_BACKGROUND ST7735_BLACK

#define DISPLAY_MAX_CHANNELS 8

#define DISPLAY_MAX_MODES 5

// Global Screen State
typedef struct {
    uint32_t lastUpdate;
    bool needsUpdate;
    bool needsBackground;
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
    uint32_t value;
    uint32_t maxValue;
} TFTBar;

// TFT Hardware; requires SPI
// https://www.adafruit.com/products/358

class Display {
public:
    Display(int8_t cs=15, int8_t rs=17, int8_t rst=16, int8_t spi_sck=14);
    void maintain();
    void directionInput(int16_t delta);
    void pressInput(uint8_t button, boolean pressed);
    void setupBrightness(uint8_t pwm = 20);
    void changeBrightness(int16_t delta);
    void setBrightness(int16_t b=128);
    void setupVolts(RingBuffer *buffer, uint32_t max);
    void voltsReady(bool ready);
    void setupChannel(uint8_t channel, uint32_t max);
    void setChannel(uint8_t channel, uint32_t value);
    void setElapsed(uint32_t millis);
    void setGPSData(long lat, long lon, unsigned long course);
    void setTime();

    void splash();

private:
    Adafruit_ST7735* _tft;

    TFTUpdate* _tftUpdate;
    TFTPager* _pager;

    void _setBrightness(uint8_t b);
    uint8_t   _brightnessPWM;
    uint8_t   _brightness;
    TFTBar*   _brightnessBar;

    RingBuffer *_voltBuffer;
    TFTBar*     _voltsBar;

    TFTBar *_channelBars[DISPLAY_MAX_CHANNELS];

    uint32_t _elapsed;


    long _lastLat;
    long _lastLon;
    unsigned long _course;
    byte _heading[3];
    byte _month, _day, _hour, _minute, _second, _hundredths;

    void _modeDrawBrightness();
    void _modeDrawMenu();
    void _modeDrawVolts();
    void _modeDrawChannels();
    void _modeDrawDrive();
    void _modeDrawStatus();

    void _renderBorder(uint8_t width, uint16_t colour);
    void _renderBar(TFTBar *bar, double fraction);
};

#endif //DIGIHEAD_DISPLAY_H
