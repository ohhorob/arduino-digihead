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
    _tftUpdate->needsUpdate = true;

    _pager = {new TFTPager()};
    _pager->mode = 0x00; // Splash/Welcome
    _pager->nextMode = 0x01;
    _pager->nextWhen = millis() + 5000; // Menu in 5 seconds
    _pager->changeModeOnDirection = false;

    _brightnessPWM = 0;

    _tft->initR(INITR_144GREENTAB);   // initialize a ST7735S chip, green tab
    _tft->fillScreen(DISPLAY_BACKGROUND);
}


/**
 * Enable brightness control via PWM output.
 *
 * Call when a PWM pin has been allocated to driving the TFT backlight.
 *
 * Omiting the call will disable brightness control.
 */
void Display::setupBrightness(int8_t pwm) {
    _brightness = 128;
    _brightnessPWM = pwm;
    pinMode(_brightnessPWM, OUTPUT);

    _brightnessBar = {new TFTBar()};
    _brightnessBar->colour = ST7735_BLUE;
    uint16_t inset = 2;
    _brightnessBar->x0 = inset;
    _brightnessBar->maxWidth = _tft->width() - (2 * inset);
    _brightnessBar->height = 14;
    _brightnessBar->y0 = _tft->height() - _brightnessBar->height - inset;

    setBrightness(_brightness);
}

/**
 * Brightness adjustment API
 *
 * Delta is scaled for covering full range quicker.
 */
void Display::changeBrightness(int16_t delta) {
    // TODO: Scale change in brightness based on current value
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

/**
 * Brightness setting API
 *
 * Value will be clamped to 0<= b <= 255
 */
void Display::setBrightness(int16_t b) {
    // Only when brightness was configured
    if (_brightnessPWM > 0) {
        // Clamp the value from min to max
        if (b < 0) {
            b = 0;
        } else if (b > 0xFF) {
            b = 0xFF;
        }
        _setBrightness(b);
    }
}

/**
 * Internal brightness
 */
void Display::_setBrightness(uint8_t b) {
    // Only take action if there was a change
    if (b != _brightness) {
        _brightness = b;
        _tftUpdate->brightness = true;
        analogWrite(_brightnessPWM, _brightness);
    }
}

/**
 * Accept input from directional control.
 */
int16_t accumulatedDirection = 0;
void Display::directionInput(int16_t delta) {
    if (_pager->changeModeOnDirection) {
        uint32_t now = millis();

        accumulatedDirection += delta;

        // Limit the rate of mode changing by forcing some time to elapse from the last change
        if (now - _pager->nextWhen > 100 && abs(accumulatedDirection) >= 5) {
            _tftUpdate->needsUpdate = true;
            _pager->nextWhen = now;
            _pager->nextMode = _pager->mode + (delta > 0 ? 1 : -1);
            if (_pager->nextMode > 5) {
                _pager->nextMode = 5;
            }
            Serial.print("Direction Input nexMode=");
            Serial.println(_pager->nextMode);
            accumulatedDirection = 0;
        } else if (abs(accumulatedDirection) < 5) {
            Serial.print("Still accumulating direction: ");
            Serial.println(accumulatedDirection, DEC);
        } else if (now - _pager->nextWhen > 100) {
            // Reset accumulated direction
            accumulatedDirection = 0;
        }
    } else {
        accumulatedDirection = 0;
        changeBrightness(delta);
    }
}

void Display::pressInput(uint8_t button, boolean pressed) {
    switch (button) {
        case 0x01:
            Serial.print("Encoder Button: ");
            Serial.println(pressed ? "DOWN" : "UP");
            _pager->changeModeOnDirection = pressed;
            break;
        default:
            Serial.print("Unknown button: ");
            Serial.println(button);
    }
}

/**
 * Update or redraw visible elements.
 */
void Display::maintain() {
    uint32_t now = millis();
    // Test mode change if one was scheduled
    if (_pager->nextWhen > 0 && now >= _pager->nextWhen) {
        // Advance to next mode
        _pager->mode = _pager->nextMode;
        _pager->nextWhen = 0;
        _tftUpdate->needsUpdate = true;
        Serial.print("maintain: changed mode to "); Serial.println(_pager->mode, HEX);
    }

    // Perform drawing tasks once per interval
    if ((now - _tftUpdate->lastUpdate) > DISPLAY_INTERVAL_UPDATE) {
        if (_tftUpdate->needsUpdate) {
            // Draw the current mode; let it determine if actual drawing needs to be done
            switch (_pager->mode) {
                case 0x00: // Splash/welcome/boot
                    splash();
                    break;
                case 0x01: // Menu
                    _modeDrawMenu();
                    break;
                default: // Hmmm.. not sure
                    _tft->fillScreen(DISPLAY_BACKGROUND);
                    _tft->setCursor(40, 60);
                    _tft->setTextColor(ST7735_MAGENTA);
                    _tft->setTextSize(2);
                    _tft->println(_pager->mode, DEC);
//                    _pager->nextMode = 0x00;
//                    _pager->nextWhen = millis() + 1000;
                    break;
            }
            _tftUpdate->lastUpdate = now;
            _tftUpdate->needsUpdate = false;
        }

        // Test overlay (global, partial screen elements)
        if (_tftUpdate->brightness) {
            _modeDrawBrightness();
            _tftUpdate->brightness = false;
            // Reset mode back to previous in 1 second
            _pager->nextMode = _pager->mode;
            _pager->nextWhen = now + 1000;
        }

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

void Display::_modeDrawBrightness() {
    // Render a bar for brightness
    TFTBar* bar = _brightnessBar;

    double fraction = _brightness;
    fraction /= 255;
    fraction *= bar->maxWidth;
//            Serial.println(fraction, 3);
    int16_t w = (int16_t)(fraction + 0.5);

    int16_t maxHeight = bar->y0 + bar->height;

    // Blackout
    for (int16_t y = bar->y0; y < maxHeight; y++) {
        _tft->drawFastHLine(bar->x0 + w, y, bar->x0 + bar->maxWidth, DISPLAY_BACKGROUND);
    }

    // Partial bar
    for (int16_t y = bar->y0; y < maxHeight; y++) {
        _tft->drawFastHLine(bar->x0, y, w, bar->colour);
    }
}

void Display::_modeDrawMenu() {
    _tft->fillScreen(DISPLAY_BACKGROUND);
    _tft->drawRect(2, 2, _tft->width()-4, _tft->height()-4, ST7735_CYAN);
}