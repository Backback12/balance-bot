#include "cosmetics.h"

// The display is initialized with the pointer provided in the constructor
Cosmetics::Cosmetics(TwoWire* wireBus) 
    : _wireBus(wireBus),
      display(128, 64, wireBus, -1), 
      roboEyes(display), 
      _lastTick(0), 
      _iteration(0), 
      _initialized(false) {}

void Cosmetics::begin(int buzzerPin, int sda, int scl, uint32_t frequency) {
    // Initialize the specific I2C bus provided (Wire or Wire1)
    _wireBus->begin(sda, scl, frequency);

    // bs.begin(buzzerPin);

    // 0x3C is the standard OLED address
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("OLED on chosen Wire bus failed"));
        return; 
    }

    roboEyes.begin(128, 64, 100); 
    roboEyes.setPosition(DEFAULT);
    roboEyes.close(); 
    roboEyes.setAutoblinker(true);
    roboEyes.setIdleMode(true, 5, 1);
    roboEyes.setWidth(26, 26);
    roboEyes.setHeight(32, 32);
    roboEyes.setSpacebetween(50);
    roboEyes.setBorderradius(13, 13);

    // bs.play(1); 
    _initialized = true;
}


void Cosmetics::tick() {
    oledTick(); 
    // buzzerTick();
}
void Cosmetics::buzzerTick() { bs.update(); }
void Cosmetics::oledTick() {
    if (!_initialized) return;
    unsigned long now = millis();

    if (now - _lastOledUpdate < 50) { // 1000/30fps ~= 33ms, 50ms=20fps
        return;
    }
    roboEyes.update();

    /*
    // Change Emotion
    if (now - _lastTick > 3000) {
        switch (_iteration) {
            case 0: bs.play(0, 100); roboEyes.setMood(HAPPY); break;
            case 1: bs.play(1); roboEyes.anim_laugh(); break;
            case 2: bs.play(2); roboEyes.setMood(TIRED); break;
            case 3: bs.play(3); roboEyes.anim_confused(); break;
            case 4: bs.play(4, 50); roboEyes.anim_laugh(); break;
        }
        _lastTick = millis();
        _iteration = (_iteration + 1) % 5;
    }
    */
}