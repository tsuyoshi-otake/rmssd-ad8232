#include <M5Stack.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include <math.h>

// ---------- Pins ----------
constexpr int      ECG_PIN       = 34;  // AD8232 OUTPUT
constexpr int      LO_PLUS_PIN   = 13;  // AD8232 LO+
constexpr int      LO_MINUS_PIN  = 22;  // AD8232 LO-  (off GPIO12 flash-voltage strap)
constexpr adc1_channel_t ECG_ADC = ADC1_CHANNEL_6;  // GPIO34

// ---------- Timing ----------
constexpr uint32_t SAMPLE_US        = 4000;  // 250 Hz
constexpr uint32_t STARTUP_LOCKOUT_MS = 2500;  // AD8232 BPF settle
constexpr uint32_t REFRACTORY_MS    = 300;   // R-wave refractory (HR <= 200 bpm)
constexpr uint32_t PEAK_TIMEOUT_MS  = 120;
constexpr int      RR_BUF_SIZE      = 128;

// ---------- Calibration ----------
constexpr uint32_t CALIB_MIN_MS    = 5UL * 60UL * 1000UL;  // 5 minutes minimum
constexpr uint32_t CALIB_SAMPLE_MS = 5UL * 1000UL;         // pull one RMSSD every 5 s
constexpr int      CALIB_BUF_SIZE  = 60;                   // 5 min / 5 s
constexpr int      CALIB_MIN_FILL  = 50;                   // need >= 80% full to accept

// ---------- Layout ----------
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;
constexpr int STATS_TOP = 0;
constexpr int STATS_H   = 48;
constexpr int ECG_TOP   = STATS_H + 2;            // 50
constexpr int ECG_H     = 160;                    // 50..210
constexpr int ECG_MID   = ECG_TOP + ECG_H / 2;
constexpr int BAR_TOP   = ECG_TOP + ECG_H + 4;    // 214
constexpr int BAR_H     = 22;                     // 214..236

// ---------- ISR ring buffer ----------
struct RawSample {
  uint16_t raw;
  uint8_t  leadsOff;  // 1 = off, 0 = on
};
constexpr int RING_SIZE = 64;  // 256 ms of slack
static volatile RawSample ring[RING_SIZE];
static volatile uint16_t  ringHead = 0;  // written by ISR
static volatile uint16_t  ringTail = 0;  // written by loop
static volatile uint32_t  ringDrops = 0;
static hw_timer_t*        sampleTimer = nullptr;

// ---------- HRV state ----------
static float    rrBuf[RR_BUF_SIZE];
static int      rrHead = 0;
static int      rrCount = 0;

static float    ecgBase   = 2048.0f;
static float    ampEma    = 600.0f;
static float    threshold = 80.0f;

static bool     inPeak     = false;
static float    peakVal    = 0;
static uint32_t peakMs     = 0;
static uint32_t lastBeatMs = 0;

static float    bpm        = 0;
static float    rmssd      = 0;
static float    rmssdBase  = NAN;

// ---------- Calibration state (rolling 5-min median) ----------
static float    calibBuf[CALIB_BUF_SIZE];
static int      calibHead = 0;
static int      calibCount = 0;
static uint32_t calibStartMs    = 0;
static uint32_t calibNextSampleMs = 0;
static bool     baselineFrozen = false;

// ---------- Drawing state ----------
static int      sweepX    = 0;
static int      prevY     = ECG_MID;
static bool     prevValid = false;
static bool     beatTick  = false;

static uint32_t bootMs    = 0;
static uint32_t nextDisplayMs = 0;

// ---------- ISR ----------
void IRAM_ATTR onSampleTimer() {
  uint16_t next = (ringHead + 1) % RING_SIZE;
  if (next == ringTail) {
    ringDrops++;
    return;
  }
  int raw = adc1_get_raw(ECG_ADC);
  uint8_t lo = (gpio_get_level((gpio_num_t)LO_PLUS_PIN) ||
                gpio_get_level((gpio_num_t)LO_MINUS_PIN)) ? 1 : 0;
  ring[ringHead].raw      = (uint16_t)raw;
  ring[ringHead].leadsOff = lo;
  ringHead = next;
}

// ---------- HRV ----------
void addRR(float rrMs) {
  rrBuf[rrHead] = rrMs;
  rrHead = (rrHead + 1) % RR_BUF_SIZE;
  if (rrCount < RR_BUF_SIZE) rrCount++;
}

// RMSSD with two-stage outlier filter:
//   1) absolute cap: |dRR| < 300 ms
//   2) Malik 20%:    |dRR| < 0.2 * RR_prev
float calcRMSSD() {
  if (rrCount < 3) return 0;
  int start = (rrHead - rrCount + RR_BUF_SIZE) % RR_BUF_SIZE;
  float prev = rrBuf[start];
  float sumsq = 0;
  int n = 0;
  for (int i = 1; i < rrCount; i++) {
    int idx = (start + i) % RR_BUF_SIZE;
    float cur = rrBuf[idx];
    float d   = cur - prev;
    float ad  = fabsf(d);
    if (ad < 300.0f && ad < 0.2f * prev) {
      sumsq += d * d;
      n++;
    }
    prev = cur;
  }
  if (n == 0) return 0;
  return sqrtf(sumsq / (float)n);
}

// ---------- Calibration helpers ----------
void resetCalibration(uint32_t nowMs) {
  calibHead         = 0;
  calibCount        = 0;
  calibStartMs      = nowMs;
  calibNextSampleMs = nowMs + CALIB_SAMPLE_MS;
  baselineFrozen    = false;
  rmssdBase         = NAN;
}

void calibPush(float v) {
  calibBuf[calibHead] = v;
  calibHead = (calibHead + 1) % CALIB_BUF_SIZE;
  if (calibCount < CALIB_BUF_SIZE) calibCount++;
}

float calibMedian() {
  if (calibCount < 1) return NAN;
  static float tmp[CALIB_BUF_SIZE];
  for (int i = 0; i < calibCount; i++) tmp[i] = calibBuf[i];
  for (int i = 1; i < calibCount; i++) {
    float key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  if (calibCount & 1) return tmp[calibCount / 2];
  return 0.5f * (tmp[calibCount / 2 - 1] + tmp[calibCount / 2]);
}

void updateCalibration(uint32_t nowMs, bool leadsOff, bool lockout) {
  if (baselineFrozen)                 return;
  if (leadsOff || lockout)            return;
  if (rrCount < 20 || rmssd <= 0)     return;
  if (nowMs < calibNextSampleMs)      return;

  calibPush(rmssd);
  calibNextSampleMs = nowMs + CALIB_SAMPLE_MS;

  uint32_t elapsed   = nowMs - calibStartMs;
  bool     enoughT   = (elapsed >= CALIB_MIN_MS);
  bool     enoughN   = (calibCount >= CALIB_MIN_FILL);
  if (enoughT && enoughN) {
    rmssdBase = calibMedian();
  }
}

void registerBeat(uint32_t tMs, float peak) {
  if (lastBeatMs != 0) {
    float rr = (float)(tMs - lastBeatMs);
    if (rr > 300 && rr < 2000) {
      addRR(rr);
      bpm = 60000.0f / rr;
      rmssd = calcRMSSD();
    }
  }
  lastBeatMs = tMs;
  threshold = 0.75f * threshold + 0.25f * peak * 0.6f;
  if (threshold < 35)  threshold = 35;
  if (threshold > 800) threshold = 800;
}

// ---------- Drawing ----------
int ecgToY(float sig) {
  float halfH = ECG_H / 2 - 4;
  float norm  = sig / (ampEma * 2.5f);
  if (norm > 1)  norm = 1;
  if (norm < -1) norm = -1;
  return ECG_MID - (int)(norm * halfH);
}

void drawSweepPoint(int x, int y, bool leadsOff, bool tick, bool clipped) {
  M5.Lcd.startWrite();
  M5.Lcd.drawFastVLine(x, ECG_TOP, ECG_H, BLACK);
  M5.Lcd.drawPixel(x, ECG_MID, DARKGREY);

  if (tick) M5.Lcd.drawFastVLine(x, ECG_TOP, 8, RED);

  if (!leadsOff) {
    if (prevValid) {
      M5.Lcd.drawLine(x - 1, prevY, x, y, GREEN);
    } else {
      M5.Lcd.drawPixel(x, y, GREEN);
    }
    if (clipped) {
      // Clip marker at strip top/bottom edge
      M5.Lcd.drawPixel(x, ECG_TOP + 1,         YELLOW);
      M5.Lcd.drawPixel(x, ECG_TOP + ECG_H - 2, YELLOW);
    }
  }
  M5.Lcd.endWrite();
}

void drawSweepCursor() {
  int cx = (sweepX + 1) % SCREEN_W;
  M5.Lcd.startWrite();
  M5.Lcd.drawFastVLine(cx, ECG_TOP, ECG_H, BLACK);
  M5.Lcd.drawPixel(cx, ECG_MID, DARKGREY);
  M5.Lcd.endWrite();
}

void drawGridBaseline() {
  M5.Lcd.startWrite();
  for (int x = 0; x < SCREEN_W; x += 4) {
    M5.Lcd.drawPixel(x, ECG_MID, DARKGREY);
  }
  M5.Lcd.drawRect(0, ECG_TOP - 1, SCREEN_W, ECG_H + 2, DARKGREY);
  M5.Lcd.endWrite();
}

void drawStats(bool leadsOff, bool lockout) {
  M5.Lcd.fillRect(0, STATS_TOP, SCREEN_W, STATS_H, BLACK);
  M5.Lcd.setTextSize(2);

  if (leadsOff) {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(4, STATS_TOP + 4);
    M5.Lcd.print("LEADS OFF");
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(4, STATS_TOP + 26);
    M5.Lcd.print("check electrodes");
    return;
  }

  if (lockout) {
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.setCursor(4, STATS_TOP + 4);
    M5.Lcd.print("settling...");
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(4, STATS_TOP + 26);
    M5.Lcd.print("AD8232 BPF warmup");
    return;
  }

  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(4, STATS_TOP + 4);
  M5.Lcd.printf("HR %3.0f  RMSSD %3.0f ms", bpm, rmssd);

  M5.Lcd.setCursor(4, STATS_TOP + 26);
  uint32_t nowMs = millis();
  if (rrCount < 20 || rmssd <= 0) {
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.printf("collecting RR  n=%d", rrCount);
  } else if (isnan(rmssdBase)) {
    uint32_t elapsed = nowMs - calibStartMs;
    uint32_t remainS = (elapsed < CALIB_MIN_MS) ? (CALIB_MIN_MS - elapsed) / 1000 : 0;
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.printf("calib %lu:%02lu  n=%d",
                  (unsigned long)(remainS / 60),
                  (unsigned long)(remainS % 60),
                  calibCount);
  } else if (baselineFrozen) {
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("base %.0f ms (frozen)", rmssdBase);
  } else {
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("base %.0f ms (rolling)", rmssdBase);
  }
}

void drawBar() {
  M5.Lcd.startWrite();
  M5.Lcd.fillRect(0, BAR_TOP, SCREEN_W, BAR_H, BLACK);
  M5.Lcd.drawRect(0, BAR_TOP, SCREEN_W, BAR_H, DARKGREY);

  int cx = SCREEN_W / 2;
  M5.Lcd.drawFastVLine(cx, BAR_TOP, BAR_H, DARKGREY);
  M5.Lcd.drawFastVLine(0,            BAR_TOP, BAR_H, DARKGREY);
  M5.Lcd.drawFastVLine(SCREEN_W - 1, BAR_TOP, BAR_H, DARKGREY);
  M5.Lcd.endWrite();

  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(DARKGREY, BLACK);
  M5.Lcd.setCursor(2,             BAR_TOP + BAR_H / 2 - 3); M5.Lcd.print("0.5x");
  M5.Lcd.setCursor(cx - 8,        BAR_TOP + BAR_H / 2 - 3); M5.Lcd.print("1.0x");
  M5.Lcd.setCursor(SCREEN_W - 26, BAR_TOP + BAR_H / 2 - 3); M5.Lcd.print("2.0x");

  if (isnan(rmssdBase) || rmssd <= 0 || rrCount < 20) return;

  float ratio = rmssd / rmssdBase;
  float t = (log2f(ratio) + 1.0f) * 0.5f;  // 0.5x -> 0, 1x -> 0.5, 2x -> 1
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  int px = (int)(t * (SCREEN_W - 1));

  uint16_t color = CYAN;
  if (ratio > 1.15f)      color = GREEN;
  else if (ratio < 0.85f) color = ORANGE;

  int x1 = (cx < px) ? cx : px;
  int x2 = (cx > px) ? cx : px;
  M5.Lcd.fillRect(x1, BAR_TOP + 2, (x2 - x1) > 0 ? (x2 - x1) : 1, BAR_H - 4, color);

  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(SCREEN_W / 2 - 18, BAR_TOP + BAR_H / 2 - 3);
  M5.Lcd.printf("%.2fx", ratio);
}

// ---------- Sample processing (loop side) ----------
static int medBuf[3] = {2048, 2048, 2048};
static int medPos    = 0;

int median3(int v) {
  medBuf[medPos] = v;
  medPos = (medPos + 1) % 3;
  int a = medBuf[0], b = medBuf[1], c = medBuf[2];
  int hi = (a > b) ? a : b; if (c > hi) hi = c;
  int lo = (a < b) ? a : b; if (c < lo) lo = c;
  return a + b + c - hi - lo;
}

void processSample(int rawIn, bool leadsOff) {
  int raw = median3(rawIn);

  ecgBase += 0.002f * ((float)raw - ecgBase);
  float sig    = (float)raw - ecgBase;
  float absSig = fabsf(sig);

  ampEma += 0.005f * (absSig - ampEma);
  if (ampEma < 100) ampEma = 100;

  uint32_t nowMs = millis();
  bool lockout = (nowMs - bootMs) < STARTUP_LOCKOUT_MS;

  if (!leadsOff && !lockout) {
    threshold *= 0.9995f;
    if (threshold < 35) threshold = 35;

    if (!inPeak && absSig > threshold && (nowMs - lastBeatMs) > REFRACTORY_MS) {
      inPeak  = true;
      peakVal = absSig;
      peakMs  = nowMs;
    }
    if (inPeak) {
      if (absSig > peakVal) { peakVal = absSig; peakMs = nowMs; }
      if (absSig < threshold * 0.45f || (nowMs - peakMs) > PEAK_TIMEOUT_MS) {
        registerBeat(peakMs, peakVal);
        inPeak   = false;
        beatTick = true;
      }
    }
  } else {
    inPeak = false;
  }

  // Determine clip and Y
  float halfH = ECG_H / 2 - 4;
  float norm  = sig / (ampEma * 2.5f);
  bool  clipped = (norm > 1.0f) || (norm < -1.0f);
  int   y = leadsOff ? ECG_MID : ecgToY(sig);

  drawSweepPoint(sweepX, y, leadsOff, beatTick, clipped);
  beatTick = false;

  prevY     = y;
  prevValid = !leadsOff;

  sweepX = (sweepX + 1) % SCREEN_W;
  drawSweepCursor();
  if (sweepX == 0) prevValid = false;
}

// ---------- Arduino ----------
void setup() {
  M5.begin();  // also initializes Serial @ 115200

  pinMode(LO_PLUS_PIN,  INPUT_PULLDOWN);
  pinMode(LO_MINUS_PIN, INPUT_PULLDOWN);

  // ADC1 setup (IDF API direct; analogRead*() helpers also work but slower)
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ECG_ADC, ADC_ATTEN_DB_11);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(4, 4);
  M5.Lcd.println("AD8232 HRV");
  delay(400);
  M5.Lcd.fillScreen(BLACK);
  drawGridBaseline();
  drawBar();

  bootMs = millis();
  resetCalibration(bootMs);

  // 80 MHz APB / prescaler 80 -> 1 MHz timer; alarm at SAMPLE_US ticks
  sampleTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
  timerAlarmWrite(sampleTimer, SAMPLE_US, true);
  timerAlarmEnable(sampleTimer);
}

void loop() {
  M5.update();

  // Drain ring buffer — guaranteed to keep up with 250 Hz
  while (ringTail != ringHead) {
    uint16_t t = ringTail;
    RawSample s = { ring[t].raw, ring[t].leadsOff };
    ringTail = (t + 1) % RING_SIZE;
    processSample((int)s.raw, s.leadsOff != 0);
  }

  uint32_t nowMs = millis();
  bool     leadsOff = (digitalRead(LO_PLUS_PIN) || digitalRead(LO_MINUS_PIN));
  bool     lockout  = (nowMs - bootMs) < STARTUP_LOCKOUT_MS;

  // BtnA: restart calibration. BtnB: freeze / unfreeze the rolling baseline.
  if (M5.BtnA.wasPressed()) {
    resetCalibration(nowMs);
  }
  if (M5.BtnB.wasPressed() && !isnan(rmssdBase)) {
    baselineFrozen = !baselineFrozen;
  }

  updateCalibration(nowMs, leadsOff, lockout);

  if (nowMs > nextDisplayMs) {
    nextDisplayMs = nowMs + 500;
    drawStats(leadsOff, lockout);
    drawBar();

    Serial.print("BPM:");     Serial.print(bpm);
    Serial.print(",RMSSD:");  Serial.print(rmssd);
    Serial.print(",BASE:");   Serial.print(isnan(rmssdBase) ? 0 : rmssdBase);
    Serial.print(",CALIBN:"); Serial.print(calibCount);
    Serial.print(",FROZEN:"); Serial.print(baselineFrozen ? 1 : 0);
    Serial.print(",DROPS:");  Serial.println(ringDrops);
  }
}
