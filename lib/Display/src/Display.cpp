#include "Display.h"

Display::Display(int8_t cs, int8_t rs, int8_t rst, int8_t spi_sck)
{
    // Ensure the SPI clock is on the correct pin
    SPI.setSCK(spi_sck);

    _tft = {new Adafruit_ST7735(cs, rs, rst)};

    // State initialisation
    _tftUpdate = {new TFTUpdate()};
    _tftUpdate->brightness = false;
    _tftUpdate->time = false;
    _tftUpdate->lastUpdate = 0;

    _brightnessPWM = 0;

    _tft->initR(INITR_144GREENTAB);   // initialize a ST7735S chip, green tab
    _tft->fillScreen(ST7735_BLACK);
}


/**
 * Enable brightness control via PWM output.
 */
void Display::setupBrightness(int8_t pwm) {
    _brightness = 128;
    _brightnessPWM = pwm;
    pinMode(_brightnessPWM, OUTPUT);
    setBrightness(_brightness);
}

void Display::changeBrightness(int16_t delta) {
    // Scale change in brightness based on current value
    // Small LED PWM changes are not noticeable close to 100% duty
    int16_t newBrightness = delta * 2 + ((int16_t )_brightness);
//    Serial.println(newBrightness, HEX);
    if (newBrightness <= 0 && _brightness >= 0) {
        _setBrightness(0);
    } else if (newBrightness >= 255) {
        _setBrightness(255);
    } else {
        _setBrightness(newBrightness & 0xFF);
    }
}

void Display::setBrightness(uint8_t b) {
    // Only when brightness was configured
    if (_brightnessPWM > 0) {
        // Clamp the value from min to max
        if (b < 0) {
            b = 0;
        } else if (b > 0xFF) {
            b = 0xFF;
        }
    }
}

void Display::_setBrightness(uint8_t b) {
    // Only take action if there was a change
    if (b != _brightness) {
        _brightness = b;
        _tftUpdate->brightness = true;
        analogWrite(_brightnessPWM, _brightness);
    }
}

/**
 * Update or redraw visible elements.
 */
void Display::maintain() {
    uint32_t now = millis();
    if (now - _tftUpdate->lastUpdate > DISPLAY_INTERVAL_UPDATE) {
        if (_tftUpdate->brightness) {

            // Draw brightness bar (bottom of screen)
            int16_t y = _tft->height() - 2;
            // Full width black first
            int16_t full_width = _tft->width() - 2;
            _tft->drawLine(1, y, full_width, y, ST7735_BLACK);

            // Then brightness width in blue
            double fraction = _brightness;
            fraction /= 255;
            fraction *= full_width;
//            Serial.println(fraction, 3);
            int16_t w = (int16_t)(fraction + 0.5);
            _tft->drawLine(1, y, w, y, ST7735_BLUE);

            // Dump to screen for debug
//            _tft->setCursor(60, 55);
//            _tft->setTextColor(ST7735_CYAN);
//            _tft->fillRect(60, 55, 50, 20, ST7735_BLACK);
//            _tft->print(_brightness);

            _tftUpdate->brightness = false;
        }
        _tftUpdate->lastUpdate = now;
    }
}

void Display::splash() {
    _tft->setTextWrap(false);
    _tft->fillScreen(ST7735_BLACK);
    _tft->setCursor(10, 30);
    _tft->setTextColor(ST7735_YELLOW);
    _tft->setTextSize(2);
    _tft->println("DigiHead");
}