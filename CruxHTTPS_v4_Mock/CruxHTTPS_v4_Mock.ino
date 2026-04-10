// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS_v4_mock.ino — Crux IoT · v4 MOCK (simulated sensor data)
// ─────────────────────────────────────────────────────────────────────
//  Board     : VVM601 (ESP32-S3 + Quectel EC200U 4G LTE)
//  Libraries : TinyGSM, ArduinoJson v7+
//
//  MOCK MODE:
//    • No physical sensors connected (no DHT, no INA219)
//    • All sensor values are realistically randomized each cycle
//    • All HTTP / GPRS / modem / registration logic is IDENTICAL to v4
//    • Safe to run on a bare board with only the modem wired up
// ═══════════════════════════════════════════════════════════════════════

#include "config.h"
#include <esp_task_wdt.h>

// ── TinyGSM must be configured BEFORE the include ────────────────────
#define TINY_GSM_MODEM_BG96
#define TINY_GSM_USE_GPRS true
#define SerialAT Serial1
#define SerialMon Serial

#include <TinyGsmClient.h>
#include <ArduinoJson.h>

#if !defined(ARDUINOJSON_VERSION_MAJOR) || (ARDUINOJSON_VERSION_MAJOR < 7)
#define CRUX_USE_JSONDOC_V6 1
#else
#define CRUX_USE_JSONDOC_V6 0
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#if USE_HTTPS
TinyGsmClientSecure netClient(modem);
#else
TinyGsmClient netClient(modem);
#endif

// ═════════════════════════════════════════════════════════════════════
//  GLOBALS
// ═════════════════════════════════════════════════════════════════════
bool registered = false;
uint32_t lastSendMs = 0;
uint32_t bootTime = 0;
uint32_t successCount = 0;
uint32_t failCount = 0;

float sensorTemp = 0.0;
float sensorHumid = 0.0;
float sensorCurrent = 0.0;
float sensorVoltage = 0.0;
float sensorPower = 0.0;
float sensorSOC = 0.0;

// Mock state — slow drift variables so data feels realistic over time
float mockVoltage = 12.50;   // starts mid-range
float mockSOC = 55.0;        // starts mid-range
float mockTemp = 28.0;

// Forward decl.
static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret);

// ═════════════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ═════════════════════════════════════════════════════════════════════

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(200);

  // Seed the RNG from an unconnected ADC pin for better randomness
  randomSeed(analogRead(0) ^ (analogRead(1) << 8) ^ micros());

  LOG("═══════════════════════════════════════════════");
  LOG("  Crux IoT · Firmware v" FIRMWARE_VER " [MOCK MODE]");
  LOG("  Device : " DEVICE_NAME);
  LOG("  ID     : " DEVICE_ID);
  LOG("  Server : " SERVER_HOST);
  LOGF("  Mode   : %s (port %d)\n", USE_HTTPS ? "HTTPS" : "HTTP", SERVER_PORT);
  LOG("  Build  : CruxHTTPS_v4_mock — NO REAL SENSORS");
  LOG("═══════════════════════════════════════════════\n");

  powerOnModem();
  initModem();
  waitForNetwork();
  connectGPRS();
  registerWithRetry();

  bootTime = millis();
  LOG("\n[BOOT] Setup complete — entering main loop\n");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true};
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(NULL);
#endif
  LOG("[BOOT] Watchdog active");
}

void loop()
{
  ensureConnected();
  if (!registered)
    registerWithRetry();

  uint32_t now = millis();

  if (registered && (now - lastSendMs >= SEND_INTERVAL_MS))
  {
    lastSendMs = now;

    readAllSensors();   // generates mock data
    bool ok = sendSensorData();

    if (ok) successCount++;
    else    failCount++;

    uint32_t uptimeSec = (millis() - bootTime) / 1000;
    LOGF("[STATS] Uptime: %lum %lus | Sent: %lu ok, %lu fail\n",
         uptimeSec / 60, uptimeSec % 60, successCount, failCount);
  }

  esp_task_wdt_reset();

  uint32_t delayMs = LOOP_DELAY_MS;
#if ADAPTIVE_LOOP_DELAY
  if (registered && lastSendMs != 0)
  {
    uint32_t nextSend = lastSendMs + SEND_INTERVAL_MS;
    if (nextSend > now)
    {
      uint32_t until = nextSend - now;
      if (until < delayMs)
        delayMs = until;
    }
  }
  if (delayMs < (uint32_t)LOOP_DELAY_MIN_MS)
    delayMs = LOOP_DELAY_MIN_MS;
#endif
  delay(delayMs);
}

// ═════════════════════════════════════════════════════════════════════
//  MOCK SENSOR GENERATOR
//  Produces realistic, slowly drifting values — no hardware needed.
//
//  Ranges (matching real battery/environment expectations):
//    Temperature : 20.0 – 45.0 °C  (slow drift ±0.5 per cycle)
//    Humidity    : 30.0 – 80.0 %   (independent random walk)
//    Voltage     : 10.5 – 14.4 V   (slow drift ±0.05 per cycle)
//    Current     : 0.5  – 15.0 A   (random per cycle with smoothing)
//    Power       : V × I
//    SOC         : derived from voltage via same LUT as real firmware
// ═════════════════════════════════════════════════════════════════════

// Helper: random float in [lo, hi]
static float randFloat(float lo, float hi)
{
  return lo + (float)random(0, 10001) / 10000.0f * (hi - lo);
}

// Helper: clamp value to [lo, hi]
static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Same LUT as original firmware so SOC values are consistent
static float voltageToSOC_mock(float v)
{
  static const float LUT_V[]   = {10.50, 11.31, 11.58, 11.75, 11.90,
                                   12.06, 12.20, 12.32, 12.50, 12.70, 14.40};
  static const float LUT_SOC[] = {0.0,   10.0,  20.0,  30.0,  40.0,
                                   50.0,  60.0,  70.0,  80.0,  90.0, 100.0};
  const int N = 11;
  if (v <= LUT_V[0])   return 0.0;
  if (v >= LUT_V[N-1]) return 100.0;
  for (int i = 1; i < N; i++) {
    if (v <= LUT_V[i]) {
      float ratio = (v - LUT_V[i-1]) / (LUT_V[i] - LUT_V[i-1]);
      return LUT_SOC[i-1] + ratio * (LUT_SOC[i] - LUT_SOC[i-1]);
    }
  }
  return 100.0;
}

void readAllSensors()
{
  LOG("[MOCK] ── Generating sensor data ──");

  // ── Temperature: slow drift ±0.5 °C, bounded 20–45 °C ──────────
  mockTemp += randFloat(-0.5f, 0.5f);
  mockTemp  = clampf(mockTemp, 20.0f, 45.0f);
  sensorTemp = mockTemp;

  // ── Humidity: independent random walk, bounded 30–80 % ──────────
  static float mockHumid = 55.0f;
  mockHumid += randFloat(-1.0f, 1.0f);
  mockHumid  = clampf(mockHumid, 30.0f, 80.0f);
  sensorHumid = mockHumid;

  // ── Voltage: slow drift ±0.05 V, bounded 10.5–14.4 V ───────────
  mockVoltage += randFloat(-0.05f, 0.05f);
  mockVoltage  = clampf(mockVoltage, 10.5f, 14.4f);
  sensorVoltage = mockVoltage;

  // ── Current: random each cycle, 0.5–15.0 A ──────────────────────
  sensorCurrent = randFloat(0.5f, 15.0f);

  // ── Power: V × I (matches INA219 behaviour) ──────────────────────
  sensorPower = sensorVoltage * sensorCurrent;

  // ── SOC: derived from voltage via same LUT ───────────────────────
  sensorSOC = voltageToSOC_mock(sensorVoltage);
  // Add a tiny ±1% jitter so it doesn't look perfectly quantized
  sensorSOC += randFloat(-1.0f, 1.0f);
  sensorSOC = clampf(sensorSOC, 0.0f, 100.0f);

  LOGF("[MOCK]   Temp    : %.1f °C\n",  sensorTemp);
  LOGF("[MOCK]   Humid   : %.1f %%\n",  sensorHumid);
  LOGF("[MOCK]   Voltage : %.2f V\n",   sensorVoltage);
  LOGF("[MOCK]   Current : %.3f A\n",   sensorCurrent);
  LOGF("[MOCK]   Power   : %.2f W\n",   sensorPower);
  LOGF("[MOCK]   SOC     : %.1f %%\n",  sensorSOC);
  LOG("[MOCK] ────────────");
}

// ═════════════════════════════════════════════════════════════════════
//  MODEM — identical to v4
// ═════════════════════════════════════════════════════════════════════

void powerOnModem()
{
  LOG("[MODEM] Powering on EC200U...");
  pinMode(MODEM_POWER, OUTPUT);
  digitalWrite(MODEM_POWER, LOW);
  delay(100);
  digitalWrite(MODEM_POWER, HIGH);
  delay(1000);
  digitalWrite(MODEM_POWER, LOW);
  LOG("[MODEM] Power sequence done — waiting 5s for boot");
  delay(5000);
}

void initModem()
{
  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  LOG("[MODEM] Initializing...");
  if (!modem.init())
    LOG("[MODEM] init() failed — trying restart()");
  if (!modem.restart())
    LOG("[MODEM] restart() failed — continuing");

  String name = modem.getModemName();
  String info = modem.getModemInfo();
  LOG("[MODEM] Name: " + name);
  LOG("[MODEM] Info: " + info);
#if USE_HTTPS
  LOG("[MODEM] TLS: modem hardware stack");
#endif
  LOG("[MODEM] Ready\n");
}

void waitForNetwork()
{
  LOG("[NET] Scanning for 4G network...");
  if (!modem.waitForNetwork(NETWORK_TIMEOUT_MS, true))
  {
    LOG("[NET] ✗ No network — rebooting in 30s");
    delay(30000);
    ESP.restart();
  }
  LOGF("[NET] Registered (signal: %d/31)\n", modem.getSignalQuality());
}

void connectGPRS()
{
  LOG("[NET] Connecting GPRS...");
  if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS))
  {
    delay(10000);
    if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS))
    {
      delay(5000);
      ESP.restart();
    }
  }
  LOG("[NET] GPRS connected — IP: " + modem.getLocalIP());
}

void ensureConnected()
{
  if (!modem.isNetworkConnected())
  {
    LOG("[NET] Network lost — reconnecting");
    waitForNetwork();
    connectGPRS();
  }
  if (!modem.isGprsConnected())
  {
    LOG("[NET] GPRS lost — reconnecting");
    connectGPRS();
  }
}

// ═════════════════════════════════════════════════════════════════════
//  HTTP — identical to v4
// ═════════════════════════════════════════════════════════════════════

void registerWithRetry()
{
  LOG("[REG] Registering device...");

#if CRUX_USE_JSONDOC_V6
  StaticJsonDocument<JSON_DOC_CAPACITY> doc;
#else
  JsonDocument doc;
#endif
  doc["deviceId"]        = DEVICE_ID;
  doc["deviceSecret"]    = DEVICE_SECRET;
  doc["deviceName"]      = DEVICE_NAME;
  doc["firmwareVersion"] = FIRMWARE_VER;

  char jsonBuf[JSON_SERIAL_BUFFER];
  size_t jsonLen = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
  if (jsonLen == 0 || jsonLen >= sizeof(jsonBuf))
  {
    LOG("[REG] JSON serialize error or buffer full");
    return;
  }

  for (int i = 1; i <= 5; i++)
  {
    int code = httpPost(REGISTER_PATH, jsonBuf, jsonLen, nullptr, nullptr);
    if (code == 200 || code == 201)
    {
      registered = true;
      LOG("[REG] Registered\n");
      return;
    }
    LOGF("[REG] Attempt %d failed (HTTP %d)\n", i, code);
    delay(REGISTER_RETRY_MS);
  }
  LOG("[REG] Will retry in main loop\n");
}

bool sendSensorData()
{
  LOG("[DATA] Sending...");

#if CRUX_USE_JSONDOC_V6
  StaticJsonDocument<JSON_DOC_CAPACITY> doc;
#else
  JsonDocument doc;
#endif
  doc["temperature"] = round1(sensorTemp);
  doc["voltage"]     = round2(sensorVoltage);
  doc["power"]       = round2(sensorPower);
  doc["current"]     = round3(sensorCurrent);
  doc["soc"]         = round1(sensorSOC);

  char jsonBuf[JSON_SERIAL_BUFFER];
  size_t jsonLen = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
  if (jsonLen == 0 || jsonLen >= sizeof(jsonBuf))
  {
    LOG("[DATA] JSON serialize error");
    return false;
  }

  LOGF("[DATA] Payload: %s\n", jsonBuf);

  int code = httpPost(DATA_PATH, jsonBuf, jsonLen, DEVICE_ID, DEVICE_SECRET);

  if (code == 201)
  {
    LOG("[DATA] 201\n");
    return true;
  }

  LOGF("[DATA] HTTP %d\n", code);
  if (code == 404)
    registered = false;
  return false;
}

static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret)
{
  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++)
  {
    if (attempt > 1)
    {
      LOGF("[HTTP] Retry %d/%d\n", attempt, MAX_RETRIES);
      delay(RETRY_BACKOFF_MS * attempt);
    }

    LOGF("[HTTP] Connect %s:%d\n", SERVER_HOST, SERVER_PORT);
    if (!netClient.connect(SERVER_HOST, SERVER_PORT))
    {
      LOG("[HTTP] TCP failed");
      netClient.stop();
      continue;
    }

    char hdrs[HTTP_HEADER_BUFFER];
    int h = snprintf(
        hdrs, sizeof(hdrs),
        "POST %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n",
        path, SERVER_HOST, (unsigned)bodyLen);

    if (h <= 0 || (size_t)h >= sizeof(hdrs) - 96)
    {
      LOG("[HTTP] header base truncated");
      netClient.stop();
      continue;
    }

    int pos = h;
    if (devId && devSecret)
    {
      int a = snprintf(
          hdrs + pos, sizeof(hdrs) - (size_t)pos,
          "x-device-id: %s\r\nx-device-secret: %s\r\n",
          devId, devSecret);
      if (a <= 0 || pos + a >= (int)sizeof(hdrs) - 8)
      {
        LOG("[HTTP] auth header truncated");
        netClient.stop();
        continue;
      }
      pos += a;
    }

    int fin = snprintf(hdrs + pos, sizeof(hdrs) - (size_t)pos, "\r\n");
    if (fin < 2) { netClient.stop(); continue; }
    pos += fin;

    netClient.write((const uint8_t *)hdrs, (size_t)pos);
    netClient.write((const uint8_t *)body, bodyLen);

    uint32_t t0 = millis();
    while (!netClient.available() && millis() - t0 < HTTP_TIMEOUT_MS)
      delay(50);

    if (!netClient.available())
    {
      LOG("[HTTP] timeout");
      netClient.stop();
      continue;
    }

    String statusLine = netClient.readStringUntil('\n');
    int statusCode = -1;
    if (statusLine.length() > 12)
      statusCode = statusLine.substring(9, 12).toInt();
    LOGF("[HTTP] ← %d\n", statusCode);

    while (netClient.available())
    {
      String line = netClient.readStringUntil('\n');
      if (line.length() <= 1) break;
    }

    char respBuf[HTTP_RESP_BODY_MAX];
    size_t respN = 0;
    uint32_t readStart = millis();
    while (netClient.available() && millis() - readStart < 3000 && respN < sizeof(respBuf) - 1)
      respBuf[respN++] = (char)netClient.read();
    respBuf[respN] = '\0';
    if (respN > 0) LOGF("[HTTP] Body: %s\n", respBuf);

    netClient.stop();

    if (statusCode >= 200 && statusCode < 300) return statusCode;
    if (statusCode >= 400 && statusCode < 500 && statusCode != 408 && statusCode != 429)
      return statusCode;
  }

  LOG("[HTTP] all retries failed");
  return -1;
}

// ═════════════════════════════════════════════════════════════════════
float round1(float v) { return ((int)(v * 10   + 0.5f)) / 10.0f;   }
float round2(float v) { return ((int)(v * 100  + 0.5f)) / 100.0f;  }
float round3(float v) { return ((int)(v * 1000 + 0.5f)) / 1000.0f; }
