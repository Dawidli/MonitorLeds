#include <FastLED.h>

#define NUM_LEDS        147
#define DATA_PIN        6

#define MAX_BRIGHTNESS  255      // hard safety cap (0–255)
#define SWITCH_PIN      2
#define BRIGHTNESS_PIN  A0       // <-- analog pin (A0/A1/...)

CRGB leds[NUM_LEDS];

// --- Serial protocol ---
static const uint32_t BAUD = 2000000;
static const uint8_t  HEADER[4]   = {0xAA, 0x55, 0xAA, 0x55};
static const uint16_t FRAME_SIZE  = NUM_LEDS * 3;

// ----- helpers -----
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
  pinMode(SWITCH_PIN, INPUT);     // If you want bulletproof: INPUT_PULLUP + wire switch to GND
  pinMode(BRIGHTNESS_PIN, INPUT);

  Serial.begin(BAUD);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear(true);
}

void loop() {
  // ---- Read switch ----
  bool switchOn = (digitalRead(SWITCH_PIN) == HIGH);

  // ---- Read potentiometer and map to 0..MAX_BRIGHTNESS ----
  int raw = analogRead(BRIGHTNESS_PIN);                 // 0..1023
  uint8_t potBrightness = map(raw, 0, 1023, 0, MAX_BRIGHTNESS);

  // Optional smoothing (removes jitter). Comment out if you don't want it.
  static uint16_t smooth = 0;
  smooth = (smooth * 9 + potBrightness * 1) / 10;       // simple IIR
  uint8_t brightness = (uint8_t)smooth;

  if (!switchOn) {
    // OFF: force brightness to 0 and show once
    FastLED.setBrightness(0);
    FastLED.show();   // no need to clear; brightness 0 makes it dark
    // If you prefer true black data on strip, uncomment:
    // FastLED.clear(true);
    return;           // skip serial parsing while off
  }

  // ON: set brightness from potentiometer
  FastLED.setBrightness(brightness);

  // Receive and display frames when available
  if (waitForHeader()) {
    static uint8_t buf[FRAME_SIZE];

    if (readExact(buf, FRAME_SIZE, 50)) {
      for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t r = buf[3 * i + 0];
        uint8_t g = buf[3 * i + 1];
        uint8_t b = buf[3 * i + 2];
        leds[i].setRGB(r, g, b);
      }
      FastLED.show();
    }
    // If readExact fails, we ignore the partial frame (no show)
  }

  // (Optional) small delay to reduce CPU usage (not required)
  // delay(1);
}
