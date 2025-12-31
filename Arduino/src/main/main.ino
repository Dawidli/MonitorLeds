#include <FastLED.h>

#define NUM_LEDS 147
#define DATA_PIN 6
#define MAX_BRIGHTNESS 100

CRGB leds[NUM_LEDS];

// --- Serial protocol ---
static const uint32_t BAUD = 2000000;
static const uint8_t HEADER[4] = {0xAA, 0x55, 0xAA, 0x55};  // magic sync
static const uint16_t FRAME_SIZE = NUM_LEDS * 3;            // RGB bytes

// Optional: disable idle fade while debugging (recommended)
#define ENABLE_IDLE_FADE 0

#if ENABLE_IDLE_FADE
const unsigned long IDLE_START_MS    = 1000;
const unsigned long FADE_INTERVAL_MS = 40;
const uint8_t       FADE_AMOUNT      = 20;

unsigned long lastFrameTime = 0;
unsigned long lastFadeTime  = 0;
bool ledsAreOff = true;
bool fading     = false;
#endif

// Reads exactly n bytes into buf, returns true if complete within timeout_ms
bool readExact(uint8_t* buf, uint16_t n, uint16_t timeout_ms) {
  uint16_t got = 0;
  unsigned long start = millis();
  while (got < n && (millis() - start) < timeout_ms) {
    while (Serial.available() && got < n) {
      buf[got++] = (uint8_t)Serial.read();
    }
  }
  return (got == n);
}

// Scan stream until we see HEADER sequence
bool waitForHeader() {
  static uint8_t state = 0;

  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    if (state == 0) {
      state = (b == HEADER[0]) ? 1 : 0;
    } else if (state == 1) {
      state = (b == HEADER[1]) ? 2 : (b == HEADER[0] ? 1 : 0);
    } else if (state == 2) {
      state = (b == HEADER[2]) ? 3 : (b == HEADER[0] ? 1 : 0);
    } else { // state == 3
      if (b == HEADER[3]) {
        state = 0;
        return true;
      } else {
        state = (b == HEADER[0]) ? 1 : 0;
      }
    }
  }
  return false;
}

void setup() {
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear(true);

  Serial.begin(BAUD);

#if ENABLE_IDLE_FADE
  lastFrameTime = millis();
#endif
}

void loop() {
  // 1) Wait for a full header (robust sync)
  if (waitForHeader()) {
    static uint8_t buf[FRAME_SIZE];

    // 2) Read full payload
    if (readExact(buf, FRAME_SIZE, 50)) {
      // 3) Unpack RGB
      for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t r = buf[3 * i + 0];
        uint8_t g = buf[3 * i + 1];
        uint8_t b = buf[3 * i + 2];
        leds[i].setRGB(r, g, b);
      }
      FastLED.show();

#if ENABLE_IDLE_FADE
      unsigned long now = millis();
      lastFrameTime = now;
      ledsAreOff = false;
      fading = false;
#endif
    }
  }

#if ENABLE_IDLE_FADE
  // --- Idle fade (optional) ---
  unsigned long now = millis();

  if (!ledsAreOff && !fading && (now - lastFrameTime > IDLE_START_MS)) {
    fading = true;
    lastFadeTime = now;
  }

  if (fading && (now - lastFadeTime > FADE_INTERVAL_MS)) {
    lastFadeTime = now;
    fadeToBlackBy(leds, NUM_LEDS, FADE_AMOUNT);
    FastLED.show();

    bool anyOn = false;
    for (int i = 0; i < NUM_LEDS; ++i) {
      if (leds[i].r || leds[i].g || leds[i].b) { anyOn = true; break; }
    }
    if (!anyOn) {
      ledsAreOff = true;
      fading = false;
    }
  }
#endif
}
