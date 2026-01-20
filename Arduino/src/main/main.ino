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

// --- Serial protocol ---
static const uint32_t BAUD = 2000000;
static const uint8_t  HEADER[4]  = {0xAA, 0x55, 0xAA, 0x55};
static const uint16_t FRAME_SIZE = NUM_LEDS * 3; // expected payload length

// ---------- Threshold parameter ----------
int enable_thresh_sum = 6;
const int THRESH_MIN = 0;
const int THRESH_MAX = 60;

// ---------- Button timing ----------
unsigned long last_case_press_ms = 0;
const unsigned long CASE_DEBOUNCE_MS = 500;

const unsigned long HOLD_MS   = 1000;
const unsigned long REPEAT_MS = 100;

unsigned long up_press_start_ms   = 0;
unsigned long up_last_repeat_ms   = 0;
unsigned long down_press_start_ms = 0;
unsigned long down_last_repeat_ms = 0;

// ----- Factory reset via CASE hold -----
const unsigned long CASE_RESET_MS = 5000;
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
const unsigned long CFG_SAVE_DELAY_MS = 1000;

// ---------- Buttons ----------
bool pressedEdge(bool current, bool &last) {
  bool e = (current == LOW && last == HIGH); // INPUT_PULLUP: press is HIGH->LOW
  last = current;
  return e;
}

// ---------- UI parameters ----------
int case_selected = 0;

// Saturation factor (x100): 0..200
int sat_x100   = 90;
// Gamma (x100): 50..300
int gamma_x100 = 120;
// Warm/Cool (x100): -50..+50
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

void showSolidOrange() {
  fill_solid(leds, NUM_LEDS, CRGB(255, 80, 0));
  FastLED.show();
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

  cfg.magic      = CFG_MAGIC;
  cfg.sat_x100   = (int16_t)sat_x100;
  cfg.gamma_x100 = (int16_t)gamma_x100;
  cfg.warm_x100  = (int16_t)warm_x100;
  cfg.thresh_sum = (int16_t)enable_thresh_sum;
  EEPROM.put(0, cfg);

  analogWrite(UI_LED1, 200);
  analogWrite(UI_LED2, 200);
  analogWrite(UI_LED3, 200);
  analogWrite(UI_LED4, 200);
  delay(150);

  updateUiLeds();
}

/* ============================================================
   Robust framed receiver
   Packet = HEADER(4) + SEQ(1) + LEN(2 LE) + PAYLOAD(LEN) + CRC16(2 LE)
   CRC16-CCITT computed over: SEQ + LEN + PAYLOAD
   ============================================================ */

static inline uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else             crc = (crc << 1);
  }
  return crc;
}

bool readExact(uint8_t* buf, uint16_t n, uint16_t timeout_ms) {
  uint16_t got = 0;
  unsigned long start = millis();
  while (got < n && (millis() - start) < timeout_ms) {
    int avail = Serial.available();
    while (avail-- > 0 && got < n) {
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
  // Quick drain; used for ambient mode and “bad streak” recovery.
  unsigned long start = millis();
  while (Serial.available() && (millis() - start) < 10) Serial.read();
}

// Returns true only if a full valid packet is read (len matches + CRC ok).
bool readPacket(uint8_t* payloadBuf) {
  static uint8_t badStreak = 0;

  if (!waitForHeader()) return false;

  uint8_t meta[3];
  if (!readExact(meta, 3, 40)) { // meta is tiny, but allow some slack
    badStreak++;
    if (badStreak >= 8) { discardIncomingFrames(); badStreak = 0; }
    return false;
  }

  uint8_t seq = meta[0];
  uint16_t len = (uint16_t)meta[1] | ((uint16_t)meta[2] << 8);

  if (len != FRAME_SIZE) {
    badStreak++;
    if (badStreak >= 8) { discardIncomingFrames(); badStreak = 0; }
    return false;
  }

  // Payload read: give this plenty of time so we don't time out mid-packet
  if (!readExact(payloadBuf, len, 120)) {
    badStreak++;
    if (badStreak >= 8) { discardIncomingFrames(); badStreak = 0; }
    return false;
  }

  uint8_t crcBytes[2];
  if (!readExact(crcBytes, 2, 40)) {
    badStreak++;
    if (badStreak >= 8) { discardIncomingFrames(); badStreak = 0; }
    return false;
  }

  uint16_t crc_rx = (uint16_t)crcBytes[0] | ((uint16_t)crcBytes[1] << 8);

  uint16_t crc = 0xFFFF;
  crc = crc16_ccitt_update(crc, seq);
  crc = crc16_ccitt_update(crc, (uint8_t)(len & 0xFF));
  crc = crc16_ccitt_update(crc, (uint8_t)(len >> 8));
  for (uint16_t i = 0; i < len; i++) crc = crc16_ccitt_update(crc, payloadBuf[i]);

  if (crc != crc_rx) {
    badStreak++;
    if (badStreak >= 8) { discardIncomingFrames(); badStreak = 0; }
    return false;
  }

  badStreak = 0;
  return true;
}

/* ============================================================ */

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
  FastLED.setDither(0);
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

  // ---- Pot brightness with deadband ----
  static uint8_t brightness = MAX_BRIGHTNESS;
  static uint8_t smooth = MAX_BRIGHTNESS;

  int raw = analogRead(POT_PIN);
  uint8_t target = map(raw, 0, 1023, 0, MAX_BRIGHTNESS);

  const uint8_t DEADBAND = 2;
  if (abs((int)target - (int)smooth) >= DEADBAND) {
    smooth = (uint8_t)((smooth * 9 + target) / 10);
  }
  brightness = smooth;

  if (!switchOn) {
    FastLED.setBrightness(brightness);
    discardIncomingFrames();
    showSolidOrange();
    return;
  }

  FastLED.setBrightness(brightness);

  // ---- Receive and display (LATEST ONLY) ----
  static uint8_t buf[FRAME_SIZE];
  static uint8_t latest[FRAME_SIZE];
  bool got = false;

  // Drain backlog: keep newest valid packet only.
  // Limit iterations so UI still runs.
  for (uint8_t k = 0; k < 6; k++) {
    if (readPacket(buf)) {
      memcpy(latest, buf, FRAME_SIZE);
      got = true;
    } else {
      break;
    }
  }

  if (got) {
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t r = latest[3*i + 0];
      uint8_t g = latest[3*i + 1];
      uint8_t b = latest[3*i + 2];

      uint16_t sum = (uint16_t)r + (uint16_t)g + (uint16_t)b;
      if (sum <= (uint16_t)enable_thresh_sum) leds[i] = CRGB::Black;
      else leds[i] = applyCorrectionsFast(r, g, b);
    }
    FastLED.show();
  }
}
