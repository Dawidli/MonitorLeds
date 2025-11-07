// LedBridge_Fast.ino
#include <FastLED.h>

#define LED_PIN       6
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB       // change to RGB if colors look off
#define NUM_LEDS      147
#define BRIGHTNESS    30
#define BAUD          2000000   // try 2000000 once stable

CRGB leds[NUM_LEDS];
static uint8_t frameBuf[NUM_LEDS * 3];

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(50);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setDither(0);
  FastLED.clear(true);

  Serial.begin(BAUD);
  // Optional banner then silence (don’t print during streaming)
  Serial.println("READY");
}

bool readExact(uint8_t* dst, size_t n, uint32_t timeout_ms) {
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < n) {
    int c = Serial.read();
    if (c >= 0) {
      dst[got++] = (uint8_t)c;
    } else {
      if ((millis() - t0) > timeout_ms) return false;
      // tiny wait lets USB buffer refill
      delayMicroseconds(100);
    }
  }
  return true;
}

void loop() {
  // Seek header 0xAA 0x55
  int a = Serial.read();
  if (a < 0) return;
  if ((uint8_t)a != 0xAA) return;
  int b = Serial.read();
  if (b < 0 || (uint8_t)b != 0x55) return;

  // Read payload
  const size_t need = NUM_LEDS * 3;
  if (!readExact(frameBuf, need, 20)) {
    // flush junk and resync fast
    while (Serial.read() >= 0) {}
    return;
  }

  // Copy GRB bytes into FastLED buffer
  for (int i = 0, j = 0; i < NUM_LEDS; ++i, j += 3) {
    leds[i].g = frameBuf[j + 0];
    leds[i].r = frameBuf[j + 1];
    leds[i].b = frameBuf[j + 2];
  }
  FastLED.show();
}
