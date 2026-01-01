#include <FastLED.h>

#define DATA_PIN        6
#define NUM_LEDS        147
#define BRIGHTNESS_PIN  A0   
#define SWITCH_PIN 2


CRGB leds[NUM_LEDS];

void setup() {
  pinMode(SWITCH_PIN, INPUT);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(0);        // start dark
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();

  Serial.begin(9600);
}

void loop() {
  bool switchStatus = digitalRead(SWITCH_PIN);
  int raw = analogRead(BRIGHTNESS_PIN);
  uint8_t brightness = map(raw, 0, 1023, 0, 255);
  Serial.println(switchStatus);
  
  if (switchStatus == HIGH) {
    FastLED.setBrightness(brightness);
  }
  else if (switchStatus == LOW) {
    FastLED.setBrightness(0);
  }

  FastLED.show();
  delay(10);
}
