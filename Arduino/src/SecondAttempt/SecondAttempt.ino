#include <FastLED.h>

// ====== Hardware config ======
#define NUM_LEDS 147            // total LEDs on the strip
#define DATA_PIN 6              // Arduino pin -> 330Ω -> DIN of strip
CRGB leds[NUM_LEDS];

// NOTE: We don't do brightness limiting or "min visible" filtering here.
// The PC (Python) will handle that. Arduino just applies what it receives.

void setup() {
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);   // full range; Python will pre-scale
  Serial.begin(115200);         // line protocol: "R,G,B\n"
}

void loop() {
  static char buf[32];          // holds one incoming line, e.g. "123,45,6"
  static uint8_t idx = 0;

  // Read available characters and collect until newline
  while (Serial.available()) {
    char c = Serial.read();

    // End-of-line? Parse and apply.
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        buf[idx] = '\0';
        idx = 0;

        // Parse "R,G,B" (integers 0..255). Whitespace tolerated.
        int r, g, b;
        if (sscanf(buf, "%d,%d,%d", &r, &g, &b) == 3) {
          // Constrain to valid range (safety only; PC should already do this)
          r = constrain(r, 0, 255);
          g = constrain(g, 0, 255);
          b = constrain(b, 0, 255);

          fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
          FastLED.show();
          // Optional debug:
          // Serial.printf("OK %d,%d,%d\n", r, g, b);
        }
      }
    }
    // Keep accumulating as long as there is room
    else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
    // Overflow guard: reset buffer if line too long
    else {
      idx = 0;
    }
  }
}
