#ifndef COSMETICS_H
#define COSMETICS_H

#include <Arduino.h>
#include <Wire.h>
#include <BuzzerSounds.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>

class Cosmetics {
public:
    // Pass a pointer to the I2C bus you want to use (defaults to the first Wire)
    Cosmetics(TwoWire* wireBus = &Wire);

    // Initialize with pins. 
    void begin(int buzzerPin, int sda, int scl, uint32_t frequency = 400000);

    void tick();
    void oledTick();
    void buzzerTick();

private:
    TwoWire* _wireBus; // Store which bus we are using
    Adafruit_SSD1306 display;
    RoboEyes<Adafruit_SSD1306> roboEyes;
    BuzzerSounds bs;

    unsigned long _lastTick;
    unsigned long _lastOledUpdate;
    int _iteration;
    bool _initialized;
};

#endif