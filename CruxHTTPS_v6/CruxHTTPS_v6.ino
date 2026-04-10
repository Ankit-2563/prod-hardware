// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS_v3.ino — Crux IoT · v3 (heap-light HTTP/JSON, adaptive loop)
// ─────────────────────────────────────────────────────────────────────
//  Board     : VVM601 (ESP32-S3 + Quectel EC200U 4G LTE)
//  Libraries : TinyGSM, DHT (Adafruit), ArduinoJson v7+
//
//  v3 vs v2:
//    • JSON serialize + HTTP headers on stack → fewer String allocations
//    • Optional adaptive loop delay → tighter respect for SEND_INTERVAL_MS
//    • Watchdog reset during long sensor reads
//    • ENABLE_DEBUG defaults false (smaller / less Serial overhead)
//
//  Bring-up: set ENABLE_DEBUG true in config.h, copy secrets.h.example → secrets.h
// ═══════════════════════════════════════════════════════════════════════

#include "config.h"
#include <esp_task_wdt.h>
#include <Preferences.h>

// ── TinyGSM must be configured BEFORE the include ────────────────────
#define TINY_GSM_MODEM_BG96
#define TINY_GSM_USE_GPRS true
#define SerialAT Serial1
#define SerialMon Serial

#include <TinyGsmClient.h>
#include <ArduinoJson.h>

// ArduinoJson v7: JsonDocument(capacity). v6: StaticJsonDocument<N>.
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

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

DHT dhtSensors[TEMP_SENSOR_COUNT] = {
    DHT(DHT0_PIN, DHT0_TYPE),
    DHT(DHT1_PIN, DHT1_TYPE),
    DHT(DHT2_PIN, DHT2_TYPE),
    DHT(DHT3_PIN, DHT3_TYPE),
};

const uint8_t dhtPins[TEMP_SENSOR_COUNT] = {DHT0_PIN, DHT1_PIN, DHT2_PIN, DHT3_PIN};
const uint8_t dhtTypes[TEMP_SENSOR_COUNT] = {DHT0_TYPE, DHT1_TYPE, DHT2_TYPE, DHT3_TYPE};

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

float socEstimate = -1.0;
uint32_t lastCoulombMs = 0;
Preferences preferences;

// Forward decl. (stack-based body)
static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret);

// ═════════════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ═════════════════════════════════════════════════════════════════════

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(200);

  LOG("═══════════════════════════════════════════════");
  LOG("  Crux IoT · Firmware v" FIRMWARE_VER);
  LOG("  Device : " DEVICE_NAME);
  LOG("  ID     : " DEVICE_ID);
  LOG("  Server : " SERVER_HOST);
  LOGF("  Mode   : %s (port %d)\n", USE_HTTPS ? "HTTPS" : "HTTP", SERVER_PORT);
  LOG("  Build  : CruxHTTPS_v3");
  LOG("═══════════════════════════════════════════════\n");

  LOG("═══════════════════════════════════════════════\n");

  preferences.begin("crux", false);

  initSensors();
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

    readAllSensors();
    bool ok = sendSensorData();

    if (ok)
      successCount++;
    else
      failCount++;

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
//  SENSORS
// ═════════════════════════════════════════════════════════════════════

void initSensors()
{
  LOG("[SENSOR] Initializing...");

  for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
  {
    dhtSensors[i].begin();
    LOGF("[SENSOR]   DHT%d on GPIO %d  (sensor #%d)\n", dhtTypes[i], dhtPins[i], i);
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!ina219.begin()) {
    LOG("[SENSOR]   ⚠ Failed to find INA219 chip");
  } else {
    LOG("[SENSOR]   INA219 (I2C) ready");
  }

  LOG("[SENSOR] Ready\n");
}

void readAllSensors()
{
  LOG("[SENSOR] ── Reading ──");

  float tempSum = 0.0;
  int tempCount = 0;

  for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
  {
    float t = dhtSensors[i].readTemperature();
    if (!isnan(t))
    {
      tempSum += t;
      tempCount++;
      LOGF("[SENSOR]   Sensor #%d (DHT%d / GPIO %d) : %.1f °C\n",
           i, dhtTypes[i], dhtPins[i], t);
    }
    else
    {
      LOGF("[SENSOR]   Sensor #%d (DHT%d / GPIO %d) : ⚠ read failed — skipped\n",
           i, dhtTypes[i], dhtPins[i]);
    }
    if ((i & 1) == 1)
      esp_task_wdt_reset();
  }

  if (tempCount > 0)
  {
    sensorTemp = tempSum / tempCount;
    LOGF("[SENSOR]   Avg Temp (%d/%d sensors valid): %.1f °C\n",
         tempCount, TEMP_SENSOR_COUNT, sensorTemp);
  }
  else
  {
    LOG("[SENSOR]   ALL temp sensors failed — soft-reset DHT drivers");
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
      dhtSensors[i].begin();
  }

  float h = dhtSensors[0].readHumidity();
  if (!isnan(h))
    sensorHumid = h;
  else
    LOG("[SENSOR]   Humidity read failed, using previous value");

  esp_task_wdt_reset();

  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  
  sensorVoltage = busvoltage + (shuntvoltage / 1000.0);
  
  // Apply charge direction config to map INA219 flow to SOC expectations
  float signedCurrent = (current_mA / 1000.0) * ACS712_CHARGE_DIRECTION;
  sensorCurrent = fabs(signedCurrent);
  
  // Power from INA219 or calculated natively
  sensorPower = ina219.getPower_mW() / 1000.0;
  
  sensorSOC = calculateSOC(sensorVoltage, signedCurrent, sensorTemp);

  LOGF("[SENSOR]   Humid   : %.1f %%\n", sensorHumid);
  LOGF("[SENSOR]   Current : %.3f A\n", sensorCurrent);
  LOGF("[SENSOR]   Voltage : %.2f V\n", sensorVoltage);
  LOGF("[SENSOR]   Power   : %.2f W\n", sensorPower);
  LOGF("[SENSOR]   SOC     : %.1f %%\n", sensorSOC);
  LOG("[SENSOR] ────────────");
}



float voltageToSOC(float v)
{
  static const float LUT_V[] = {10.50, 11.31, 11.58, 11.75, 11.90,
                                12.06, 12.20, 12.32, 12.50, 12.70, 14.40};
  static const float LUT_SOC[] = {0.0, 10.0, 20.0, 30.0, 40.0,
                                  50.0, 60.0, 70.0, 80.0, 90.0, 100.0};
  const int N = 11;

  if (v <= LUT_V[0]) return 0.0;
  if (v >= LUT_V[N - 1]) return 100.0;

  for (int i = 1; i < N; i++) {
    if (v <= LUT_V[i]) {
      float ratio = (v - LUT_V[i - 1]) / (LUT_V[i] - LUT_V[i - 1]);
      return LUT_SOC[i - 1] + ratio * (LUT_SOC[i] - LUT_SOC[i - 1]);
    }
  }
  return 100.0;
}

float temperatureCompensate(float v, float tempC)
{
  const float CELLS = 6.0;
  const float MV_PER_CELL_PER_C = 0.003;
  return v + (25.0 - tempC) * CELLS * MV_PER_CELL_PER_C;
}

float calculateSOC(float rawVoltage, float signedCurrentA, float tempC)
{
  float compV = temperatureCompensate(rawVoltage, tempC);
  float voltageSoc = voltageToSOC(compV);

  uint32_t nowMs = millis();
  
  if (lastCoulombMs == 0) {
    lastCoulombMs = nowMs;
    float savedSoc = preferences.getFloat("soc", -1.0);
    
    if (savedSoc < 0.0) {
      socEstimate = voltageSoc;
      LOGF("[SOC] ★ No saved SOC found. Seeded from voltage: %.1f%%\n", socEstimate);
    } else {
      if (abs(savedSoc - voltageSoc) > 15.0) {
        socEstimate = voltageSoc;
        LOGF("[SOC] ★ Saved SOC (%.1f%%) deviates from Voltage SOC (%.1f%%) by >15%%. Overriding with Voltage SOC.\n", savedSoc, voltageSoc);
      } else {
        socEstimate = savedSoc;
        LOGF("[SOC] ★ Loaded authentic SOC from memory: %.1f%%\n", socEstimate);
      }
    }
    return socEstimate;
  }
  
  float elapsedHours = (nowMs - lastCoulombMs) / 3600000.0;
  lastCoulombMs = nowMs;

  float deltaSOC = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0;
  socEstimate += deltaSOC;

  if (fabs(signedCurrentA) < 0.5)
  {
    float before = socEstimate;
    socEstimate = 0.80 * socEstimate + 0.20 * voltageSoc;
    LOGF("[SOC] Idle drift correction: %.1f%% → %.1f%%\n", before, socEstimate);
  }

  socEstimate = constrain(socEstimate, 0.0, 100.0);

  float lastSavedSoc = preferences.getFloat("soc", -1.0);
  if (abs(socEstimate - lastSavedSoc) >= 1.0) {
    preferences.putFloat("soc", socEstimate);
    LOGF("[SOC] Flushed to NVS memory: %.1f%%\n", socEstimate);
  }

  LOGF("[SOC] V=%.2fV (comp=%.2fV) | T=%.1f°C | I=%+.3fA\n",
       rawVoltage, compV, tempC, signedCurrentA);
  LOGF("[SOC] Δt=%.4fh | ΔSOC=%+.3f%% | Coulomb→%.1f%% | VLookup=%.1f%%\n",
       elapsedHours, deltaSOC, socEstimate, voltageSoc);

  return socEstimate;
}

// ═════════════════════════════════════════════════════════════════════
//  MODEM
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
//  HTTP — stack headers + raw body (no Arduino String for request)
// ═════════════════════════════════════════════════════════════════════

void registerWithRetry()
{
  LOG("[REG] Registering device...");

#if CRUX_USE_JSONDOC_V6
  StaticJsonDocument<JSON_DOC_CAPACITY> doc;
#else
  JsonDocument doc;
#endif
  doc["deviceId"] = DEVICE_ID;
  doc["deviceSecret"] = DEVICE_SECRET;
  doc["deviceName"] = DEVICE_NAME;
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
  doc["voltage"] = round2(sensorVoltage);
  doc["power"] = round2(sensorPower);
  doc["current"] = round3(sensorCurrent);
  doc["soc"] = round1(sensorSOC);

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
    if (fin < 2)
    {
      netClient.stop();
      continue;
    }
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
      if (line.length() <= 1)
        break;
    }

    char respBuf[HTTP_RESP_BODY_MAX];
    size_t respN = 0;
    uint32_t readStart = millis();
    while (netClient.available() && millis() - readStart < 3000 && respN < sizeof(respBuf) - 1)
    {
      respBuf[respN++] = (char)netClient.read();
    }
    respBuf[respN] = '\0';
    if (respN > 0)
      LOGF("[HTTP] Body: %s\n", respBuf);

    netClient.stop();

    if (statusCode >= 200 && statusCode < 300)
      return statusCode;
    if (statusCode >= 400 && statusCode < 500 && statusCode != 408 && statusCode != 429)
      return statusCode;
  }

  LOG("[HTTP] all retries failed");
  return -1;
}

// ═════════════════════════════════════════════════════════════════════
float round1(float v) { return ((int)(v * 10 + 0.5f)) / 10.0f; }
float round2(float v) { return ((int)(v * 100 + 0.5f)) / 100.0f; }
float round3(float v) { return ((int)(v * 1000 + 0.5f)) / 1000.0f; }
