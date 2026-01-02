#include <FastLED.h>
#include <math.h>
#include <EEPROM.h>

#define NUM_LEDS 147
#define DATA_PIN 6

// ---- Pins ----
const int BTN_CASE = 2;  // CASE
const int BTN_UP   = 4;  // UP
const int BTN_DOWN = 3;  // DOWN

const int POT_PIN    = A0; // brightness pot
const int SWITCH_PIN = 7;  // ON/OFF switch (ON = HIGH)

// Indicator LEDs
const int UI_LED1 = 9;   // saturation indicator
const int UI_LED2 = 10;  // gamma indicator
const int UI_LED3 = 11;  // warm/cool indicator
const int UI_LED4 = 5;   // threshold indicator

#define MAX_BRIGHTNESS 255

CRGB leds[NUM_LEDS];

// --- Serial protocol (frames from Python) ---
static const uint32_t BAUD = 2000000;
static const uint8_t  HEADER[4]  = {0xAA, 0x55, 0xAA, 0x55};
static const uint16_t FRAME_SIZE = NUM_LEDS * 3;

// ---------- Threshold parameter ----------
int enable_thresh_sum = 6;
const int THRESH_MIN = 0;
const int THRESH_MAX = 60;

// ---------- Button timing ----------
unsigned long last_case_press_ms = 0;
const unsigned long CASE_DEBOUNCE_MS = 500; // prevents case skipping

const unsigned long HOLD_MS   = 1000; // your chosen values
const unsigned long REPEAT_MS = 100;

unsigned long up_press_start_ms   = 0;
unsigned long up_last_repeat_ms   = 0;
unsigned long down_press_start_ms = 0;
unsigned long down_last_repeat_ms = 0;

// ----- Factory reset via CASE hold -----
const unsigned long CASE_RESET_MS = 5000; // 5 seconds
unsigned long case_press_start_ms = 0;
bool case_reset_done = false;

// ---------- Persistent config ----------
struct Config {
  uint32_t magic;
  int16_t  sat_x100;
  int16_t  gamma_x100;
  int16_t  warm_x100;
  int16_t  thresh_sum;
};

const uint32_t CFG_MAGIC = 0x4C454431; // "LED1"
Config cfg;

bool cfg_dirty = false;
unsigned long cfg_dirty_since_ms = 0;
const unsigned long CFG_SAVE_DELAY_MS = 1000; // save 1s after last change

// ---------- Robust frame receiver ----------
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
    if (state == 0)       state = (b == HEADER[0]) ? 1 : 0;
    else if (state == 1)  state = (b == HEADER[1]) ? 2 : (b == HEADER[0] ? 1 : 0);
    else if (state == 2)  state = (b == HEADER[2]) ? 3 : (b == HEADER[0] ? 1 : 0);
    else {
      if (b == HEADER[3]) { state = 0; return true; }
      state = (b == HEADER[0]) ? 1 : 0;
    }
  }
  return false;
}

void discardIncomingFrames() {
  static uint8_t dump[FRAME_SIZE];
  while (waitForHeader()) {
    if (!readExact(dump, FRAME_SIZE, 5)) break;
  }
  while (Serial.available()) Serial.read();
}

// ---------- Buttons ----------
bool pressedEdge(bool current, bool &last) {
  bool e = (current == LOW && last == HIGH); // INPUT_PULLUP: press is HIGH->LOW
  last = current;
  return e;
}

// ---------- UI parameters ----------
int case_selected = 0;

// Saturation factor (x100): 0..200 => 0.00..2.00
int sat_x100   = 90;
// Gamma (x100): 50..300 => 0.50..3.00
int gamma_x100 = 120;
// Warm/Cool (x100): -50..+50 => -0.50..+0.50
int warm_x100  = 0;

bool last_case = HIGH;
bool last_up   = HIGH;
bool last_down = HIGH;

// Gamma LUT
uint8_t gammaLUT[256];
int last_gamma_x100 = -1;

void rebuildGammaLUT(int gx100) {
  if (gx100 < 10) gx100 = 10;
  if (gx100 > 600) gx100 = 600;

  float g = gx100 / 100.0f;
  for (int i = 0; i < 256; i++) {
    float x = i / 255.0f;
    float y = powf(x, g);
    int v = (int)(y * 255.0f + 0.5f);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    gammaLUT[i] = (uint8_t)v;
  }
}

static inline uint8_t clamp8(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

CRGB applyCorrectionsFast(uint8_t r, uint8_t g, uint8_t b) {
  int gray = (r + g + b) / 3;
  int sr = gray + ((int)(r - gray) * sat_x100) / 100;
  int sg = gray + ((int)(g - gray) * sat_x100) / 100;
  int sb = gray + ((int)(b - gray) * sat_x100) / 100;
  r = clamp8(sr);
  g = clamp8(sg);
  b = clamp8(sb);

  int offset = (warm_x100 * 20) / 50;
  r = clamp8((int)r + offset);
  b = clamp8((int)b - offset);

  r = gammaLUT[r];
  g = gammaLUT[g];
  b = gammaLUT[b];

  return CRGB(r, g, b);
}

// ---------- UI LED mapping ----------
int uiLevelForCase(int c) {
  if (c == 0) return map(sat_x100, 0, 200, 0, 200);
  if (c == 1) {
    int mag = abs(gamma_x100 - 100);
    return map(mag, 0, 200, 0, 200);
  }
  if (c == 2) {
    int mag = abs(warm_x100);
    return map(mag, 0, 50, 0, 200);
  }
  return map(enable_thresh_sum, THRESH_MIN, THRESH_MAX, 0, 200);
}

void updateUiLeds() {
  int lvl = uiLevelForCase(case_selected);
  analogWrite(UI_LED1, (case_selected == 0) ? lvl : 0);
  analogWrite(UI_LED2, (case_selected == 1) ? lvl : 0);
  analogWrite(UI_LED3, (case_selected == 2) ? lvl : 0);
  analogWrite(UI_LED4, (case_selected == 3) ? lvl : 0);
}

// ---------- Step helpers ----------
void stepUp() {
  if (case_selected == 0) sat_x100 += 5;
  if (case_selected == 1) gamma_x100 += 5;
  if (case_selected == 2) warm_x100 += 5;
  if (case_selected == 3) enable_thresh_sum += 2;
}
void stepDown() {
  if (case_selected == 0) sat_x100 -= 5;
  if (case_selected == 1) gamma_x100 -= 5;
  if (case_selected == 2) warm_x100 -= 5;
  if (case_selected == 3) enable_thresh_sum -= 2;
}

void clampParams() {
  if (sat_x100 < 0) sat_x100 = 0;
  if (sat_x100 > 200) sat_x100 = 200;

  if (gamma_x100 < 50) gamma_x100 = 50;
  if (gamma_x100 > 300) gamma_x100 = 300;

  if (warm_x100 < -50) warm_x100 = -50;
  if (warm_x100 > 50) warm_x100 = 50;

  if (enable_thresh_sum < THRESH_MIN) enable_thresh_sum = THRESH_MIN;
  if (enable_thresh_sum > THRESH_MAX) enable_thresh_sum = THRESH_MAX;
}

// ---------- Persistent load/save ----------
void loadConfig() {
  EEPROM.get(0, cfg);
  if (cfg.magic != CFG_MAGIC) return;

  sat_x100          = cfg.sat_x100;
  gamma_x100        = cfg.gamma_x100;
  warm_x100         = cfg.warm_x100;
  enable_thresh_sum = cfg.thresh_sum;

  clampParams();
}

void markConfigDirty() {
  cfg_dirty = true;
  cfg_dirty_since_ms = millis();
}

void saveConfigIfDue() {
  if (!cfg_dirty) return;
  unsigned long now = millis();
  if (now - cfg_dirty_since_ms < CFG_SAVE_DELAY_MS) return;

  cfg.magic      = CFG_MAGIC;
  cfg.sat_x100   = (int16_t)sat_x100;
  cfg.gamma_x100 = (int16_t)gamma_x100;
  cfg.warm_x100  = (int16_t)warm_x100;
  cfg.thresh_sum = (int16_t)enable_thresh_sum;

  EEPROM.put(0, cfg);
  cfg_dirty = false;
}

// ---------- Factory reset ----------
void factoryReset() {
  sat_x100          = 90;
  gamma_x100        = 120;
  warm_x100         = 0;
  enable_thresh_sum = 6;

  clampParams();

  rebuildGammaLUT(gamma_x100);
  last_gamma_x100 = gamma_x100;

  // Save immediately
  cfg.magic      = CFG_MAGIC;
  cfg.sat_x100   = (int16_t)sat_x100;
  cfg.gamma_x100 = (int16_t)gamma_x100;
  cfg.warm_x100  = (int16_t)warm_x100;
  cfg.thresh_sum = (int16_t)enable_thresh_sum;
  EEPROM.put(0, cfg);

  // Quick confirmation blink
  analogWrite(UI_LED1, 200);
  analogWrite(UI_LED2, 200);
  analogWrite(UI_LED3, 200);
  analogWrite(UI_LED4, 200);
  delay(150);

  updateUiLeds();
}

void setup() {
  pinMode(BTN_CASE, INPUT_PULLUP);
  pinMode(BTN_UP,   INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  pinMode(POT_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT);

  pinMode(UI_LED1, OUTPUT);
  pinMode(UI_LED2, OUTPUT);
  pinMode(UI_LED3, OUTPUT);
  pinMode(UI_LED4, OUTPUT);

  Serial.begin(BAUD);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.clear(true);

  loadConfig();

  rebuildGammaLUT(gamma_x100);
  last_gamma_x100 = gamma_x100;

  updateUiLeds();
}

void loop() {
  unsigned long now = millis();

  bool b_case = digitalRead(BTN_CASE);
  bool b_up   = digitalRead(BTN_UP);
  bool b_down = digitalRead(BTN_DOWN);

  bool changed = false;

  // ---- CASE hold detection for factory reset (5s) ----
  if (b_case == LOW) {
    if (case_press_start_ms == 0) {
      case_press_start_ms = now;
      case_reset_done = false;
    }
    if (!case_reset_done && (now - case_press_start_ms >= CASE_RESET_MS)) {
      factoryReset();
      case_reset_done = true;
      changed = true;
    }
  } else {
    case_press_start_ms = 0;
    case_reset_done = false;
  }

  // ---- CASE short press (debounced) ----
  if (pressedEdge(b_case, last_case)) {
    if (now - last_case_press_ms > CASE_DEBOUNCE_MS) {
      case_selected = (case_selected + 1) % 4;
      last_case_press_ms = now;
      changed = true;
    }
  }

  // ---- Tap UP/DOWN ----
  if (pressedEdge(b_up, last_up)) {
    stepUp();
    up_press_start_ms = now;
    up_last_repeat_ms = now;
    changed = true;
    markConfigDirty();
  }

  if (pressedEdge(b_down, last_down)) {
    stepDown();
    down_press_start_ms = now;
    down_last_repeat_ms = now;
    changed = true;
    markConfigDirty();
  }

  // ---- Hold-to-repeat UP ----
  if (b_up == LOW) {
    if (up_press_start_ms != 0 && (now - up_press_start_ms) >= HOLD_MS) {
      if ((now - up_last_repeat_ms) >= REPEAT_MS) {
        stepUp();
        up_last_repeat_ms = now;
        changed = true;
        markConfigDirty();
      }
    }
  } else {
    up_press_start_ms = 0;
  }

  // ---- Hold-to-repeat DOWN ----
  if (b_down == LOW) {
    if (down_press_start_ms != 0 && (now - down_press_start_ms) >= HOLD_MS) {
      if ((now - down_last_repeat_ms) >= REPEAT_MS) {
        stepDown();
        down_last_repeat_ms = now;
        changed = true;
        markConfigDirty();
      }
    }
  } else {
    down_press_start_ms = 0;
  }

  clampParams();

  if (gamma_x100 != last_gamma_x100) {
    rebuildGammaLUT(gamma_x100);
    last_gamma_x100 = gamma_x100;
    changed = true;
    markConfigDirty();
  }

  if (changed) updateUiLeds();

  saveConfigIfDue();

  // ---- Switch + pot brightness ----
  bool switchOn = (digitalRead(SWITCH_PIN) == HIGH);

  int raw = analogRead(POT_PIN);
  uint8_t potBrightness = map(raw, 0, 1023, 0, MAX_BRIGHTNESS);

  static uint16_t smooth = 0;
  smooth = (smooth * 9 + potBrightness) / 10;
  uint8_t brightness = (uint8_t)smooth;

  if (!switchOn) {
    FastLED.setBrightness(0);
    FastLED.show();
    discardIncomingFrames();
    return;
  }

  FastLED.setBrightness(brightness);

  // ---- Receive and display ----
  if (waitForHeader()) {
    static uint8_t buf[FRAME_SIZE];

    if (readExact(buf, FRAME_SIZE, 50)) {
      for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t r = buf[3*i + 0];
        uint8_t g = buf[3*i + 1];
        uint8_t b = buf[3*i + 2];

        uint16_t sum = (uint16_t)r + (uint16_t)g + (uint16_t)b;
        if (sum <= (uint16_t)enable_thresh_sum) {
          leds[i] = CRGB::Black;
        } else {
          leds[i] = applyCorrectionsFast(r, g, b);
        }
      }
      FastLED.show();
    }
  }
}
