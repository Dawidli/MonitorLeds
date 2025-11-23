#include <FastLED.h>

#define NUM_LEDS 147
#define DATA_PIN 6
#define MAX_BRIGHTNESS 120   // optional: limit max brightness

CRGB leds[NUM_LEDS];

const uint8_t START_BYTE = 255;
const uint32_t FRAME_SIZE = NUM_LEDS * 3;  // R,G,B per LED

void setup() {
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  Serial.begin(2000000);   // must match Python BAUD
}

void loop() {
  // wait for start byte
  if (Serial.available()) {
    int b = Serial.read();
    if (b == START_BYTE) {
      if (readFrame()) {
        FastLED.show();
      }
    }
  }
}

bool readFrame() {
  static uint8_t buf[FRAME_SIZE];
  uint32_t got = 0;
  unsigned long start = millis();

  while (got < FRAME_SIZE && (millis() - start) < 50) {
    if (Serial.available()) {
      buf[got++] = (uint8_t)Serial.read();
    }
  }

  if (got < FRAME_SIZE) {
    // timeout / incomplete frame
    return false;
  }

  // unpack RGB into leds[]
  for (int i = 0; i < NUM_LEDS; ++i) {
    uint8_t r = buf[3 * i + 0];
    uint8_t g = buf[3 * i + 1];
    uint8_t b = buf[3 * i + 2];
    leds[i].setRGB(r, g, b);
  }

  return true;
}
