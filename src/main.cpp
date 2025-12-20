/*
https://github.com/evertonramires/CuteBuzzerSounds/blob/master/src/Sounds.h
https://github.com/FluxGarage/RoboEyes/blob/main/examples/i2c_SSD1306_AnimationSequences/i2c_SSD1306_AnimationSequences.ino
*/


#include <Arduino.h>


// #include <CuteBuzzerSounds.h>
#include <BuzzerSounds.h>
BuzzerSounds bs;
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <FluxGarage_RoboEyes.h>
// create a RoboEyes instance using an Adafruit_SSD1306 display driver
RoboEyes<Adafruit_SSD1306> roboEyes(display); 




#define BUZZER_PIN 25



void setup() {
  Serial.begin(115200);
  bs.begin(BUZZER_PIN);

  // OLED Display
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C or 0x3D
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Startup robo eyes
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100); // screen-width, screen-height, max framerate - 60-100fps are good for smooth animations
  roboEyes.setPosition(DEFAULT); // eye position should be middle center
  roboEyes.close(); // start with closed eyes 
  roboEyes.setAutoblinker(true);
  

  bs.play(1);
}

float last_tick = 0;
int iteration = 0;

void loop() 
{
  roboEyes.update(); // update eyes drawings
  bs.update();

  if (millis() > last_tick + 3000)
  {
    switch (iteration)
    {
    case 0:
      Serial.println("Sound test");
      bs.play(0, 100); // SOUND_TEST
      // roboEyes.anim_laugh();
      roboEyes.setMood(HAPPY);
      break;
    case 1:
    Serial.println("Sound happy");
      bs.play(1); // SOUND_HAPPY
      roboEyes.anim_laugh();
      break;
    case 2:
    Serial.println("Sound sad");
      bs.play(2); // SOUND_SAD
      // roboEyes.anim_confused();
      roboEyes.setMood(TIRED);
      break;
    case 3:
      Serial.println("Sound confused");
      roboEyes.setMood(DEFAULT);
      bs.play(3); // SOUND_CONFUSED
      roboEyes.anim_confused();
      break;
    case 4:
      Serial.println("Sound cos");
      bs.play(4, 50); // SOUND_COS
      // roboEyes.setMood()
      roboEyes.anim_laugh();
      break;
    
    }
    last_tick = millis();
    iteration = (iteration + 1) % 5;
  }
}
