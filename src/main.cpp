#include <M5Stack.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

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

// ---------- SD logging ----------
constexpr int      SD_CS_PIN          = 4;       // M5Stack Basic on-board MicroSD CS
constexpr uint32_t FLUSH_INTERVAL_MS  = 5000;    // flush both log files every 5 s
constexpr int      RR_LOG_QUEUE_SIZE  = 32;      // RR events buffered between loop iterations

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

// ---------- SD logging state ----------
static bool      sdOk             = false;
static bool      loggingEnabled   = false;
static File      rrLogFile;
static File      sumLogFile;
static char      rrLogPath[40]    = "";
static char      sumLogPath[40]   = "";
static uint32_t  lastFlushMs      = 0;
static uint32_t  sessionStartMs   = 0;
static uint64_t  timeBaseUnixMs   = 0;   // 0 = not yet synced
static uint32_t  timeBaseMillis   = 0;

struct RREvent {
  uint32_t session_ms;
  uint64_t unix_time_ms;
  float    rr_ms;
  float    bpm;
  float    rmssd_ms;
  float    baseline_ms;
  float    ratio;
  int      rr_count;
  bool     leads_off;
};
static RREvent rrLogQueue[RR_LOG_QUEUE_SIZE];
static int     rrLogQHead = 0;   // producer (registerBeat)
static int     rrLogQTail = 0;   // consumer (loop)

// Serial line buffer for TIME command
static char    serBuf[64];
static int     serPos = 0;

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

// ---------- Time helpers ----------
static uint64_t nowUnixMs() {
  if (timeBaseUnixMs == 0) return 0;
  return timeBaseUnixMs + (uint64_t)(millis() - timeBaseMillis);
}

// Fills dst with ISO8601 UTC string "YYYY-MM-DDTHH:MM:SS.mmmZ" (24 chars + NUL).
// dst[0] = '\0' when unixMs is 0.
static void isoFromUnixMs(uint64_t unixMs, char* dst, size_t cap) {
  if (cap == 0) return;
  if (unixMs == 0) { dst[0] = '\0'; return; }
  time_t s = (time_t)(unixMs / 1000ULL);
  struct tm tm;
  gmtime_r(&s, &tm);
  snprintf(dst, cap, "%04d-%02d-%02dT%02d:%02d:%02d.%03luZ",
           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
           tm.tm_hour, tm.tm_min, tm.tm_sec,
           (unsigned long)(unixMs % 1000ULL));
}

// Format a uint64 as decimal (Arduino Print can't handle uint64 portably).
static void u64ToStr(uint64_t v, char* dst, size_t cap) {
  if (v == 0) { snprintf(dst, cap, "0"); return; }
  char buf[24];
  int p = 0;
  while (v > 0 && p < (int)sizeof(buf)) {
    buf[p++] = '0' + (int)(v % 10);
    v /= 10;
  }
  size_t i = 0;
  while (p > 0 && i + 1 < cap) {
    dst[i++] = buf[--p];
  }
  dst[i] = '\0';
}

// ---------- Serial emitters (unified I/S/R/E format) ----------
// Forward state used by emitSummary / emitRR / emitEcgSample
static bool ecgStreamEnabled = true;  // default ON; toggle with "ECG ON|OFF"

static void emitCommonPrefix(char kind) {
  Serial.print(kind);
  Serial.print(',');
  Serial.print((unsigned long)(millis() - sessionStartMs));
  Serial.print(',');
  char unixStr[24]; u64ToStr(nowUnixMs(), unixStr, sizeof(unixStr));
  Serial.print(unixStr);
}

static void emitEvent(const char* code, const char* param = nullptr) {
  emitCommonPrefix('I');
  Serial.print(',');
  Serial.print(code);
  if (param && param[0]) {
    Serial.print(',');
    Serial.print(param);
  }
  Serial.println();
}

static void emitSummary(bool leadsOff) {
  emitCommonPrefix('S');
  float baseline = isnan(rmssdBase) ? 0.0f : rmssdBase;
  Serial.printf(",%.2f,%.2f,%.2f,%d,%d,%lu,%d\n",
                bpm, rmssd, baseline,
                calibCount, baselineFrozen ? 1 : 0,
                (unsigned long)ringDrops, leadsOff ? 1 : 0);
}

static void emitRR(float rr) {
  emitCommonPrefix('R');
  float baseline = isnan(rmssdBase) ? 0.0f : rmssdBase;
  float ratio    = (isnan(rmssdBase) || rmssdBase <= 0) ? 0.0f : (rmssd / rmssdBase);
  Serial.printf(",%.1f,%.2f,%.2f,%.2f,%.4f,%d,%d\n",
                rr, (rr > 0 ? 60000.0f / rr : 0.0f),
                rmssd, baseline, ratio, rrCount, 0);
}

static void emitEcgSample(int raw) {
  if (!ecgStreamEnabled) return;
  Serial.print('E');
  Serial.print(',');
  Serial.print((unsigned long)(millis() - sessionStartMs));
  Serial.print(',');
  Serial.println(raw);
}

// ---------- SD helpers ----------
static bool findNextLogPath(const char* prefix, char* dst, size_t cap) {
  for (int i = 1; i < 1000000; i++) {
    snprintf(dst, cap, "/%s_%06d.csv", prefix, i);
    if (!SD.exists(dst)) return true;
  }
  dst[0] = '\0';
  return false;
}

static void initSdLogging() {
  sdOk = SD.begin(SD_CS_PIN);
  if (!sdOk) {
    emitEvent("SD_FAIL");
    loggingEnabled = false;
    return;
  }
  emitEvent("SD_OK");

  if (!findNextLogPath("rr", rrLogPath, sizeof(rrLogPath)) ||
      !findNextLogPath("summary", sumLogPath, sizeof(sumLogPath))) {
    emitEvent("SD_FAIL", "no_free_slot");
    sdOk = false;
    return;
  }

  rrLogFile  = SD.open(rrLogPath,  FILE_WRITE);
  sumLogFile = SD.open(sumLogPath, FILE_WRITE);
  if (!rrLogFile || !sumLogFile) {
    emitEvent("SD_FAIL", "open_failed");
    if (rrLogFile)  rrLogFile.close();
    if (sumLogFile) sumLogFile.close();
    sdOk = false;
    return;
  }

  rrLogFile.println(
    "session_ms,unix_time_ms,iso_time,rr_ms,bpm,rmssd_ms,baseline_ms,"
    "ratio,rr_count,quality,leads_off");
  sumLogFile.println(
    "session_ms,unix_time_ms,iso_time,bpm,rmssd_ms,baseline_ms,ratio,"
    "rr_count,leads_off,noise_count");
  rrLogFile.flush();
  sumLogFile.flush();

  loggingEnabled = true;
  emitEvent("RR_LOG",  rrLogPath);
  emitEvent("SUM_LOG", sumLogPath);
}

static void enqueueRREvent(float rr, bool leadsOff) {
  int next = (rrLogQHead + 1) % RR_LOG_QUEUE_SIZE;
  if (next == rrLogQTail) return;  // queue full -> drop oldest by skipping new
  RREvent& e = rrLogQueue[rrLogQHead];
  e.session_ms   = millis() - sessionStartMs;
  e.unix_time_ms = nowUnixMs();
  e.rr_ms        = rr;
  e.bpm          = (rr > 0) ? (60000.0f / rr) : 0;
  e.rmssd_ms     = rmssd;
  e.baseline_ms  = isnan(rmssdBase) ? 0.0f : rmssdBase;
  e.ratio        = (isnan(rmssdBase) || rmssdBase <= 0)
                     ? 0.0f
                     : (rmssd / rmssdBase);
  e.rr_count     = rrCount;
  e.leads_off    = leadsOff;
  rrLogQHead = next;
}

static void drainRRLog() {
  if (!loggingEnabled || !rrLogFile) {
    rrLogQTail = rrLogQHead;  // drop queued events when logging is off
    return;
  }
  while (rrLogQTail != rrLogQHead) {
    const RREvent& e = rrLogQueue[rrLogQTail];
    rrLogQTail = (rrLogQTail + 1) % RR_LOG_QUEUE_SIZE;

    char iso[28];   isoFromUnixMs(e.unix_time_ms, iso, sizeof(iso));
    char unixStr[24]; u64ToStr(e.unix_time_ms, unixStr, sizeof(unixStr));
    char line[200];
    snprintf(line, sizeof(line),
             "%lu,%s,%s,%.1f,%.2f,%.2f,%.2f,%.4f,%d,%.2f,%d",
             (unsigned long)e.session_ms,
             unixStr, iso,
             e.rr_ms, e.bpm, e.rmssd_ms, e.baseline_ms,
             e.ratio, e.rr_count, 1.0f, e.leads_off ? 1 : 0);
    rrLogFile.println(line);
  }
}

static void writeSummaryRow(bool leadsOff) {
  if (!loggingEnabled || !sumLogFile) return;
  uint64_t ums = nowUnixMs();
  char iso[28];   isoFromUnixMs(ums, iso, sizeof(iso));
  char unixStr[24]; u64ToStr(ums, unixStr, sizeof(unixStr));
  float baseline = isnan(rmssdBase) ? 0.0f : rmssdBase;
  float ratio    = (isnan(rmssdBase) || rmssdBase <= 0)
                     ? 0.0f
                     : (rmssd / rmssdBase);
  char line[200];
  snprintf(line, sizeof(line),
           "%lu,%s,%s,%.2f,%.2f,%.2f,%.4f,%d,%d,%d",
           (unsigned long)(millis() - sessionStartMs),
           unixStr, iso,
           bpm, rmssd, baseline, ratio,
           rrCount, leadsOff ? 1 : 0, 0 /* noise_count */);
  sumLogFile.println(line);
}

static void maybeFlushLogs() {
  if (!loggingEnabled) return;
  uint32_t now = millis();
  if ((now - lastFlushMs) < FLUSH_INTERVAL_MS) return;
  lastFlushMs = now;
  if (rrLogFile)  rrLogFile.flush();
  if (sumLogFile) sumLogFile.flush();
}

// ---------- Serial command (TIME <unix_sec>, ECG ON|OFF) ----------
static void handleSerialLine(const char* line) {
  if (strncmp(line, "TIME ", 5) == 0) {
    uint32_t t = (uint32_t)strtoul(line + 5, nullptr, 10);
    if (t == 0) return;
    timeBaseUnixMs = (uint64_t)t * 1000ULL;
    timeBaseMillis = millis();
    char buf[16]; snprintf(buf, sizeof(buf), "%lu", (unsigned long)t);
    emitEvent("TIME_SET", buf);
    return;
  }
  if (strcmp(line, "ECG ON") == 0) {
    ecgStreamEnabled = true;
    emitEvent("ECG_ON");
    return;
  }
  if (strcmp(line, "ECG OFF") == 0) {
    ecgStreamEnabled = false;
    emitEvent("ECG_OFF");
    return;
  }
  if (strcmp(line, "LOG ON") == 0) {
    if (sdOk) {
      loggingEnabled = true;
      emitEvent("LOG_ON");
    } else {
      emitEvent("LOG_FAIL", "sd_not_ok");
    }
    return;
  }
  if (strcmp(line, "LOG OFF") == 0) {
    loggingEnabled = false;
    emitEvent("LOG_OFF");
    return;
  }
}

static void pollSerial() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serPos > 0) {
        serBuf[serPos] = '\0';
        handleSerialLine(serBuf);
        serPos = 0;
      }
    } else if (serPos < (int)sizeof(serBuf) - 1) {
      serBuf[serPos++] = (char)c;
    } else {
      serPos = 0;  // overflow: discard line
    }
  }
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
      enqueueRREvent(rr, /*leadsOff=*/false);  // detection only fires when leads on
      emitRR(rr);
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

void drawLogIndicator() {
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(SCREEN_W - 50, STATS_TOP + 4);
  if (!sdOk) {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("SD FAIL");
  } else if (loggingEnabled) {
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print("LOG ON ");
  } else {
    M5.Lcd.setTextColor(DARKGREY, BLACK);
    M5.Lcd.print("LOG OFF");
  }
}

void drawStats(bool leadsOff, bool lockout) {
  M5.Lcd.fillRect(0, STATS_TOP, SCREEN_W, STATS_H, BLACK);
  drawLogIndicator();
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
  emitEcgSample(raw);

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
constexpr uint8_t LCD_BRIGHTNESS_DIM = 0;     // BtnC "dim" state (essentially off)
constexpr uint8_t LCD_BRIGHTNESS_ON  = 32;    // BtnC "on" / power-on default
static uint8_t    lcdBrightness      = LCD_BRIGHTNESS_ON;

void setup() {
  M5.begin();              // also initializes Serial; we re-init at 460800 below
  Serial.begin(460800);    // higher baud for ECG raw streaming (~5 KB/s)

  M5.Lcd.setBrightness(lcdBrightness);

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

  // Try SD init before the main UI starts.  Failures are non-fatal: measurement
  // and LCD/Serial output keep working, only logging is disabled.
  bootMs         = millis();
  sessionStartMs = bootMs;
  emitEvent("BOOT");
  initSdLogging();
  lastFlushMs    = millis();

  delay(400);
  M5.Lcd.fillScreen(BLACK);
  drawGridBaseline();
  drawBar();

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

  // BtnA: restart calibration. BtnB: freeze/unfreeze rolling baseline.
  // BtnC: toggle SD logging (no-op if SD is not OK).
  if (M5.BtnA.wasPressed()) {
    resetCalibration(nowMs);
  }
  if (M5.BtnB.wasPressed() && !isnan(rmssdBase)) {
    baselineFrozen = !baselineFrozen;
  }
  // BtnC: toggle LCD backlight between dim (≈off) and visible.
  if (M5.BtnC.wasPressed()) {
    lcdBrightness = (lcdBrightness == LCD_BRIGHTNESS_DIM)
                      ? LCD_BRIGHTNESS_ON
                      : LCD_BRIGHTNESS_DIM;
    M5.Lcd.setBrightness(lcdBrightness);
    emitEvent("LCD",
              lcdBrightness == LCD_BRIGHTNESS_DIM ? "DIM" : "ON");
  }

  updateCalibration(nowMs, leadsOff, lockout);

  // Drain RR log queue and run any time-sync command from PC.
  pollSerial();
  drainRRLog();
  maybeFlushLogs();

  if (nowMs > nextDisplayMs) {
    nextDisplayMs = nowMs + 500;
    drawStats(leadsOff, lockout);
    drawBar();
    writeSummaryRow(leadsOff);
    emitSummary(leadsOff);
  }
}
