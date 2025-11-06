#include <FastLED.h>

#define LED_PIN     6
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB        // if colors look swapped, try RGB
#define NUM_LEDS    147
#define BRIGHTNESS  50

CRGB leds[NUM_LEDS];

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(50);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);

  Serial.begin(1000000);
  while (!Serial) {}
  Serial.println("READY");
}

void showPixel(int index, uint8_t r, uint8_t g, uint8_t b) {
  FastLED.clear();
  if (index >= 0 && index < NUM_LEDS) {
    leds[index] = CRGB(r, g, b);
  }
  FastLED.show();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.startsWith("BLUE")) {
    int n = line.substring(4).toInt();
    n = constrain(n, 0, NUM_LEDS);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    for (int i = 0; i < n; ++i) leds[i] = CRGB::Blue;
    FastLED.show();
    Serial.println("OK");
  } else if (line.startsWith("PIX")) {
    // format: PIX <index> <r> <g> <b>
    int i, r, g, b;
    if (sscanf(line.c_str(), "PIX %d %d %d %d", &i, &r, &g, &b) == 4) {
      i = constrain(i, 0, NUM_LEDS - 1);
      r = constrain(r, 0, 255);
      g = constrain(g, 0, 255);
      b = constrain(b, 0, 255);
      showPixel(i, (uint8_t)r, (uint8_t)g, (uint8_t)b);
      Serial.println("OK");
    } else {
      Serial.println("ERR");
    }
  }
}
