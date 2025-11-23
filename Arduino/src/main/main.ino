#include <FastLED.h>

#define NUM_LEDS 147
#define DATA_PIN 6
#define MAX_BRIGHTNESS 100   // or whatever you like

CRGB leds[NUM_LEDS];

const uint8_t START_BYTE = 255;
const uint32_t FRAME_SIZE = NUM_LEDS * 3;  // R,G,B per LED

// Idle / fade-out behaviour
const unsigned long IDLE_START_MS   = 1000;  // after this with no frame, start fading
const unsigned long FADE_INTERVAL_MS = 40;  // ms between fade steps
const uint8_t FADE_AMOUNT           = 20;   // how much to fade per step (0-255)

unsigned long lastFrameTime = 0;
unsigned long lastFadeTime  = 0;
bool ledsAreOff = true;
bool fading     = false;

void setup() {
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear(true);           // start fully off
  ledsAreOff    = true;
  lastFrameTime = millis();

  Serial.begin(2000000);         // must match Python BAUD
}

void loop() {
  unsigned long now = millis();

  // --- 1) Try to receive a new frame ---
  if (Serial.available()) {
    int b = Serial.read();
    if (b == START_BYTE) {
      if (readFrame()) {
        // We got a complete, valid frame → show it
        FastLED.show();
        lastFrameTime = now;
        ledsAreOff = false;
        fading = false;       // cancel any ongoing fade
      }
    }
  }

  // --- 2) Start fade when we've been idle long enough ---
  if (!ledsAreOff && !fading && (now - lastFrameTime > IDLE_START_MS)) {
    fading = true;
    lastFadeTime = now;
  }

  // --- 3) Perform fade steps while idle ---
  if (fading && (now - lastFadeTime > FADE_INTERVAL_MS)) {
    lastFadeTime = now;

    // Fade towards black
    fadeToBlackBy(leds, NUM_LEDS, FADE_AMOUNT);
    FastLED.show();

    // Check if all LEDs are now off
    bool anyOn = false;
    for (int i = 0; i < NUM_LEDS; ++i) {
      if (leds[i].r || leds[i].g || leds[i].b) {
        anyOn = true;
        break;
      }
    }
    if (!anyOn) {
      ledsAreOff = true;
      fading = false;
    }
  }
}

bool readFrame() {
  static uint8_t buf[FRAME_SIZE];
  uint32_t got = 0;
  unsigned long start = millis();

  // Read exactly FRAME_SIZE bytes, with timeout
  while (got < FRAME_SIZE && (millis() - start) < 50) {
    if (Serial.available()) {
      buf[got++] = (uint8_t)Serial.read();
    }
  }

  if (got < FRAME_SIZE) {
    // Incomplete frame: ignore it (do NOT update LEDs)
    return false;
  }

  // Unpack RGB into leds[]
  for (int i = 0; i < NUM_LEDS; ++i) {
    uint8_t r = buf[3 * i + 0];
    uint8_t g = buf[3 * i + 1];
    uint8_t b = buf[3 * i + 2];
    leds[i].setRGB(r, g, b);
  }

  return true;
}
