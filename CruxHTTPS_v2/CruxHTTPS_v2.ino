// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS.ino — Crux IoT Platform · Production Firmware
// ─────────────────────────────────────────────────────────────────────
//  Board     : VVM601 (ESP32-S3 + Quectel EC200U 4G LTE)
//  SIM       : Vi (Vodafone Idea) 4G
//  Server    : Express.js on AWS EC2 behind Nginx (HTTPS, self-signed)
//  Sensors   : DHT11 (GPIO 4), ACS712-5A (GPIO 36), ZMPT101B (GPIO 35)
//  Interval  : 2 requests per minute (every 30 seconds)
//
//  Install these libraries via Arduino Library Manager:
//    1. TinyGSM             — by Volodymyr Shymanskyy
//    2. SSLClient           — by OPEnSLab-OSU (govorox/SSLClient)
//    3. DHT sensor library  — by Adafruit
//    4. ArduinoJson         — by Benoit Blanchon (v7)
// ═══════════════════════════════════════════════════════════════════════

#include "config.h"
#include <esp_task_wdt.h>

#define WDT_TIMEOUT_SECONDS 120 // 2 minutes (allows time for 4G connect)

// ── TinyGSM must be configured BEFORE the include ────────────────────
#define TINY_GSM_MODEM_BG96
#define TINY_GSM_USE_GPRS true
#define SerialAT Serial1
#define SerialMon Serial

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// ── Network client (TLS or plain) ───────────────────────────────────
#if USE_HTTPS
TinyGsmClientSecure netClient(modem); // Modem's native TLS client
#else
TinyGsmClient netClient(modem); // Plain TCP client
#endif

// ── Sensors ─────────────────────────────────────────────────────────
#include <DHT.h>

// 4 temperature sensors: index 0 = DHT11, index 1-3 = DHT22
// Pins and types come from config.h
DHT dhtSensors[TEMP_SENSOR_COUNT] = {
    DHT(DHT0_PIN, DHT0_TYPE), // [0] DHT11 — GPIO 4
    DHT(DHT1_PIN, DHT1_TYPE), // [1] DHT22 — GPIO 5
    DHT(DHT2_PIN, DHT2_TYPE), // [2] DHT22 — GPIO 16
    DHT(DHT3_PIN, DHT3_TYPE), // [3] DHT22 — GPIO 17
};

const uint8_t dhtPins[TEMP_SENSOR_COUNT] = {DHT0_PIN, DHT1_PIN, DHT2_PIN, DHT3_PIN};
const uint8_t dhtTypes[TEMP_SENSOR_COUNT] = {DHT0_TYPE, DHT1_TYPE, DHT2_TYPE, DHT3_TYPE};

// ── JSON ────────────────────────────────────────────────────────────
#include <ArduinoJson.h>

// ═════════════════════════════════════════════════════════════════════
//  GLOBALS
// ═════════════════════════════════════════════════════════════════════
bool registered = false;
uint32_t lastSendMs = 0;
uint32_t bootTime = 0;
uint32_t successCount = 0;
uint32_t failCount = 0;

// Latest readings
float sensorTemp = 0.0;
float sensorHumid = 0.0;
float sensorCurrent = 0.0; // absolute value (A), sent to server
float sensorVoltage = 0.0;
float sensorPower = 0.0;
float sensorSOC = 0.0;

// SOC engine state — persists across loop() calls
float socEstimate = -1.0;   // -1 = not yet initialized
uint32_t lastCoulombMs = 0; // timestamp of last coulomb counting update

// ═════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(200);

  LOG("═══════════════════════════════════════════════");
  LOG("  Crux IoT Platform · Firmware v" FIRMWARE_VER);
  LOG("  Device : " DEVICE_NAME);
  LOG("  ID     : " DEVICE_ID);
  LOG("  Server : " SERVER_HOST);
  LOGF("  Mode   : %s (port %d)\n", USE_HTTPS ? "HTTPS" : "HTTP", SERVER_PORT);
  LOG("  Rate   : 2 req/min (every 30s)");
  LOG("═══════════════════════════════════════════════\n");

  // 1. Sensors
  initSensors();

  // 2. Modem
  powerOnModem();
  initModem();

  // 3. Network
  waitForNetwork();
  connectGPRS();

  // 4. Register with server
  registerWithRetry();

  bootTime = millis();
  LOG("\n[BOOT] Setup complete — entering main loop\n");

  // 5. Watchdog Timer (WDT)
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
  LOG("[BOOT] Watchdog Timer initialized");
}

// ═════════════════════════════════════════════════════════════════════
//  LOOP — runs forever
// ═════════════════════════════════════════════════════════════════════
void loop()
{
  // ── Keep network alive ────────────────────────────────────────────
  ensureConnected();

  // ── Re-register if needed ─────────────────────────────────────────
  if (!registered)
  {
    registerWithRetry();
  }

  // ── Send data every 30 seconds (2 req/min) ────────────────────────
  uint32_t now = millis();
  if (registered && (now - lastSendMs >= SEND_INTERVAL_MS))
  {
    lastSendMs = now;

    readAllSensors();
    bool ok = sendSensorData();

    if (ok)
    {
      successCount++;
    }
    else
    {
      failCount++;
    }

    // Stats
    uint32_t uptimeSec = (millis() - bootTime) / 1000;
    LOGF("[STATS] Uptime: %lum %lus | Sent: %lu ok, %lu fail\n",
         uptimeSec / 60, uptimeSec % 60, successCount, failCount);
  }

  // Pet the watchdog logic
  esp_task_wdt_reset();

  delay(1000);
}

// ═════════════════════════════════════════════════════════════════════
//  S E N S O R S
// ═════════════════════════════════════════════════════════════════════

void initSensors()
{
  LOG("[SENSOR] Initializing...");

  // Start all 4 temperature sensors
  for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
  {
    dhtSensors[i].begin();
    LOGF("[SENSOR]   DHT%d on GPIO %d  (sensor #%d)\n", dhtTypes[i], dhtPins[i], i);
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // Full 0–3.3 V range

  pinMode(ACS712_PIN, INPUT);
  LOGF("[SENSOR]   ACS712 (5A) on GPIO %d\n", ACS712_PIN);

  pinMode(ZMPT101B_PIN, INPUT);
  LOGF("[SENSOR]   ZMPT101B on GPIO %d\n", ZMPT101B_PIN);

  LOG("[SENSOR]  Ready\n");
}

void readAllSensors()
{
  LOG("[SENSOR] ── Reading ──");

  // ── Temperature: average all valid readings ─────────────────────
  //  Reads all 4 sensors. Skips any that return NaN (failed read).
  //  sensorTemp = average of only the sensors that returned a valid value.
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
  }

  if (tempCount > 0)
  {
    sensorTemp = tempSum / tempCount; // average of valid readings
    LOGF("[SENSOR]   Avg Temp (%d/%d sensors valid): %.1f °C\n",
         tempCount, TEMP_SENSOR_COUNT, sensorTemp);
  }
  else
  {
    LOG("[SENSOR]    ALL temp sensors failed — attempting soft reset and using previous value");
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
      dhtSensors[i].begin();
    }
  }

  // ── Humidity: from sensor #0 (DHT11 on GPIO 4) only ───────────
  //  DHT22s are more accurate for temperature; DHT11 gives humidity.
  float h = dhtSensors[0].readHumidity();
  if (!isnan(h))
    sensorHumid = h;
  else
    LOG("[SENSOR]    Humidity read failed, using previous value");

  // ── Current (ACS712-5A) ─────────────────────────────────────────
  //  Read signed first (needed by SOC engine), then store absolute for server
  float signedCurrent = readACS712Signed();
  sensorCurrent = fabs(signedCurrent); // server always gets positive

  // ── Voltage (ZMPT101B) ──────────────────────────────────────────
  sensorVoltage = readZMPT101B();

  // ── Derived values ──────────────────────────────────────────────
  sensorPower = sensorVoltage * sensorCurrent;

  // ── SOC (3-layer engine: lookup + temp compensation + coulomb) ──
  //  Passes compensated voltage, signed current, and avg temp.
  //  socEstimate persists in RAM between calls — coulomb state is kept.
  sensorSOC = calculateSOC(sensorVoltage, signedCurrent, sensorTemp);

  LOGF("[SENSOR]   Humid   : %.1f %%\n", sensorHumid);
  LOGF("[SENSOR]   Current : %.3f A\n", sensorCurrent);
  LOGF("[SENSOR]   Voltage : %.2f V\n", sensorVoltage);
  LOGF("[SENSOR]   Power   : %.2f W\n", sensorPower);
  LOGF("[SENSOR]   SOC     : %.1f %%\n", sensorSOC);
  LOG("[SENSOR] ────────────");
}

// ── ACS712 — signed current (positive = charging, negative = discharging) ──
//  This is the raw signed value used internally by the SOC engine.
//  Sign direction is corrected by ACS712_CHARGE_DIRECTION in config.h.
float readACS712Signed()
{
  long sum = 0;
  for (int i = 0; i < ACS712_SAMPLES; i++)
  {
    sum += analogRead(ACS712_PIN);
    delayMicroseconds(100);
  }

  float avgRaw = (float)sum / ACS712_SAMPLES;
  float voltage = (avgRaw / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  float current = (voltage - ACS712_ZERO_POINT) / ACS712_SENSITIVITY;

  // Dead-band: readings under 40mA are noise — treat as zero
  if (fabs(current) < 0.04)
    current = 0.0;

  // Apply wiring direction correction from config.h
  return current * ACS712_CHARGE_DIRECTION;
}

// ── ACS712 — absolute value for server payload ───────────────────────────
//  Server receives magnitude only (always positive).
float readACS712()
{
  return fabs(readACS712Signed());
}

// ═════════════════════════════════════════════════════════════════════
//  S O C   E N G I N E
//
//  Three-layer calculation:
//    1. Voltage lookup table  — non-linear OCV→SOC map for lead-acid
//    2. Temperature compensation — corrects voltage for battery temp
//    3. Coulomb counting      — integrates current over time for accuracy
//       + drift correction   — re-anchors coulomb SOC to voltage when idle
// ═════════════════════════════════════════════════════════════════════

// ── Layer 1: OCV lookup table ─────────────────────────────────────────
//  Standard 12V lead-acid open-circuit voltage curve.
//  Each entry is { voltage, SOC% } at rest (no load, no charge).
//  Values from manufacturers' datasheets — far more accurate than linear.
//
//  ┌──────────┬───────┐
//  │ OCV (V)  │ SOC % │
//  ├──────────┼───────┤
//  │  10.50   │   0   │  ← hard cut-off (BATTERY_EMPTY_V)
//  │  11.31   │  10   │
//  │  11.58   │  20   │
//  │  11.75   │  30   │
//  │  11.90   │  40   │
//  │  12.06   │  50   │  ← nominal "half charge"
//  │  12.20   │  60   │
//  │  12.32   │  70   │
//  │  12.50   │  80   │
//  │  12.70   │  90   │
//  │  14.40   │ 100   │  ← bulk charge voltage (BATTERY_FULL_V)
//  └──────────┴───────┘
float voltageToSOC(float v)
{
  static const float LUT_V[] = {10.50, 11.31, 11.58, 11.75, 11.90,
                                12.06, 12.20, 12.32, 12.50, 12.70, 14.40};
  static const float LUT_SOC[] = {0.0, 10.0, 20.0, 30.0, 40.0,
                                  50.0, 60.0, 70.0, 80.0, 90.0, 100.0};
  const int N = 11;

  if (v <= LUT_V[0])
    return 0.0;
  if (v >= LUT_V[N - 1])
    return 100.0;

  // Find the two surrounding table points and interpolate linearly between them
  for (int i = 1; i < N; i++)
  {
    if (v <= LUT_V[i])
    {
      float ratio = (v - LUT_V[i - 1]) / (LUT_V[i] - LUT_V[i - 1]);
      return LUT_SOC[i - 1] + ratio * (LUT_SOC[i] - LUT_SOC[i - 1]);
    }
  }
  return 100.0;
}

// ── Layer 2: Temperature compensation ────────────────────────────────
//  Lead-acid chemistry: battery voltage drops in cold, rises in heat.
//  Rule: –3mV per cell per °C away from 25°C reference.
//  12V battery = 6 cells → –0.018 V/°C
//
//  Example: real voltage 12.0V at 5°C →
//    compensated = 12.0 + (25 – 5) × 0.018 = 12.0 + 0.36 = 12.36V
//  Without compensation that reads as ~65% SOC.
//  With compensation it correctly reads as ~71% SOC.
float temperatureCompensate(float v, float tempC)
{
  const float CELLS = 6.0;               // 12V battery = 6 lead-acid cells
  const float MV_PER_CELL_PER_C = 0.003; // 3mV/cell/°C = industry standard
  return v + (25.0 - tempC) * CELLS * MV_PER_CELL_PER_C;
}

// ── Layer 3: Coulomb counting + drift correction ──────────────────────
//  calculateSOC() is called every SEND_INTERVAL_MS (30s).
//
//  What it does each call:
//    a) Temperature-compensate the raw voltage
//    b) Look up voltage-based SOC (ground truth)
//    c) On first call  → initialize socEstimate from voltage lookup
//    d) On later calls → update socEstimate using coulomb counting:
//         ΔSOC = (current_A × elapsed_hours / capacity_Ah) × 100
//    e) Drift correction: when current < 0.5A (battery at rest),
//         gently blend coulomb SOC 80% toward voltage SOC 20%.
//         This prevents the coulomb counter from drifting over days.
//
//  Returns: SOC % (0.0 – 100.0), stored in socEstimate between calls.
float calculateSOC(float rawVoltage, float signedCurrentA, float tempC)
{

  // a) Temperature-compensate voltage
  float compV = temperatureCompensate(rawVoltage, tempC);

  // b) Voltage lookup (used for init and drift correction)
  float voltageSoc = voltageToSOC(compV);

  // c) First-ever call — no history yet, bootstrap from voltage
  if (socEstimate < 0.0)
  {
    socEstimate = voltageSoc;
    lastCoulombMs = millis();
    LOGF("[SOC] ★ Initialized from voltage lookup: %.1f%%\n", socEstimate);
    return socEstimate;
  }

  // d) Coulomb counting
  //    Δt in hours (millis → hours)
  uint32_t nowMs = millis();
  float elapsedHours = (nowMs - lastCoulombMs) / 3600000.0;
  lastCoulombMs = nowMs;

  //    ΔAh = I(A) × Δt(h)  — positive = energy added (charging)
  //    ΔSOC% = (ΔAh / capacity_Ah) × 100
  float deltaSOC = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0;
  socEstimate += deltaSOC;
  socEstimate = constrain(socEstimate, 0.0, 100.0);

  LOGF("[SOC] V=%.2fV (comp=%.2fV) | T=%.1f°C | I=%+.3fA\n",
       rawVoltage, compV, tempC, signedCurrentA);
  LOGF("[SOC] Δt=%.4fh | ΔSOC=%+.3f%% | Coulomb→%.1f%% | VLookup=%.1f%%\n",
       elapsedHours, deltaSOC, socEstimate, voltageSoc);

  // e) Drift correction
  //    When nearly no current flows the battery is at rest → OCV is valid.
  //    Pull the coulomb estimate gently toward the voltage estimate (20% blend).
  //    This self-corrects any accumulated error without causing sudden jumps.
  if (fabs(signedCurrentA) < 0.5)
  {
    float before = socEstimate;
    socEstimate = 0.80 * socEstimate + 0.20 * voltageSoc;
    LOGF("[SOC] Idle drift correction: %.1f%% → %.1f%%\n", before, socEstimate);
  }

  return socEstimate;
}

// ── ZMPT101B — multi-sample averaged voltage reading ────────────────
float readZMPT101B()
{
  long sum = 0;
  for (int i = 0; i < ZMPT101B_SAMPLES; i++)
  {
    sum += analogRead(ZMPT101B_PIN);
    delayMicroseconds(100);
  }

  float avgRaw = (float)sum / ZMPT101B_SAMPLES;
  float sensorV = (avgRaw / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  float actualV = (sensorV - ZMPT101B_ZERO_POINT) * ZMPT101B_CAL_FACTOR;

  if (actualV < 0.0)
    actualV = 0.0;
  if (actualV > 25.0)
    actualV = 25.0;

  return actualV;
}

// ═════════════════════════════════════════════════════════════════════
//  M O D E M   &   N E T W O R K
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
  {
    LOG("[MODEM]  init() failed — trying restart()");
  }
  if (!modem.restart())
  {
    LOG("[MODEM]  restart() failed — continuing");
  }

  String name = modem.getModemName();
  String info = modem.getModemInfo();
  LOG("[MODEM] Name: " + name);
  LOG("[MODEM] Info: " + info);

// Configure modem native TLS
#if USE_HTTPS
  // By using TinyGsmClientSecure, we offload the TLS handshake to the
  // Quectel modem's internal IP stack, which is much more stable over 4G.
  LOG("[MODEM] TLS: using modem's built-in hardware TLS stack");
#endif

  LOG("[MODEM]  Ready\n");
}

void waitForNetwork()
{
  LOG("[NET] Scanning for Vi 4G network...");

  if (!modem.waitForNetwork(NETWORK_TIMEOUT_MS, true))
  {
    LOG("[NET]  No network! Check SIM / antenna / coverage");
    LOG("[NET]   Rebooting in 30s...");
    delay(30000);
    ESP.restart();
  }

  int csq = modem.getSignalQuality();
  LOGF("[NET]  Registered (signal: %d/31)\n", csq);
}

void connectGPRS()
{
  LOG("[NET] Connecting GPRS...");

  if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS))
  {
    LOG("[NET]  GPRS failed — retrying in 10s");
    delay(10000);
    if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS))
    {
      LOG("[NET]  GPRS failed again — rebooting");
      delay(5000);
      ESP.restart();
    }
  }

  LOG("[NET]  GPRS connected");
  LOG("[NET]   IP: " + modem.getLocalIP());
}

void ensureConnected()
{
  if (!modem.isNetworkConnected())
  {
    LOG("[NET]  Network lost — reconnecting");
    waitForNetwork();
    connectGPRS();
  }
  if (!modem.isGprsConnected())
  {
    LOG("[NET]  GPRS lost — reconnecting");
    connectGPRS();
  }
}

// ═════════════════════════════════════════════════════════════════════
//  H T T P   C O M M U N I C A T I O N
// ═════════════════════════════════════════════════════════════════════

// ── Device registration (called on boot) ────────────────────────────
//  POST /api/hardware/register
//  Body: { deviceId, deviceSecret, deviceName, firmwareVersion }
// ─────────────────────────────────────────────────────────────────────
void registerWithRetry()
{
  LOG("[REG] Registering device...");

  // Build JSON body
  JsonDocument doc;
  doc["deviceId"] = DEVICE_ID;
  doc["deviceSecret"] = DEVICE_SECRET;
  doc["deviceName"] = DEVICE_NAME;
  doc["firmwareVersion"] = FIRMWARE_VER;

  String body;
  serializeJson(doc, body);

  // Try up to 5 times
  for (int i = 1; i <= 5; i++)
  {
    int code = httpPost(REGISTER_PATH, body, NULL, NULL);

    if (code == 200 || code == 201)
    {
      registered = true;
      LOG("[REG]  Registered successfully\n");
      return;
    }

    LOGF("[REG]  Attempt %d failed (HTTP %d)\n", i, code);
    delay(REGISTER_RETRY_MS);
  }

  LOG("[REG]  All attempts failed — will retry in main loop\n");
}

// ── Send sensor data ────────────────────────────────────────────────
//  POST /api/hardware/data
//  Headers: x-device-id, x-device-secret
//  Body: { temperature, voltage, power, current, soc }
// ─────────────────────────────────────────────────────────────────────
bool sendSensorData()
{
  LOG("[DATA] Sending to server...");

  // Build JSON — field names match server's batteryDataSchema exactly
  JsonDocument doc;
  doc["temperature"] = round1(sensorTemp);
  doc["voltage"] = round2(sensorVoltage);
  doc["power"] = round2(sensorPower);
  doc["current"] = round3(sensorCurrent);
  doc["soc"] = round1(sensorSOC);

  String body;
  serializeJson(doc, body);
  LOGF("[DATA] Payload: %s\n", body.c_str());

  int code = httpPost(DATA_PATH, body, DEVICE_ID, DEVICE_SECRET);

  if (code == 201)
  {
    LOG("[DATA]  Accepted (201)\n");
    return true;
  }

  LOGF("[DATA]  Rejected (HTTP %d)\n", code);

  // 404 = device not found → need to re-register
  if (code == 404)
  {
    LOG("[DATA]   Marking device as unregistered");
    registered = false;
  }

  return false;
}

// ═════════════════════════════════════════════════════════════════════
//  RAW HTTP POST — handles connect, TLS, headers, body, response
//  Returns HTTP status code, or -1 on connection failure
// ═════════════════════════════════════════════════════════════════════
int httpPost(const char *path, const String &body,
             const char *devId, const char *devSecret)
{

  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++)
  {
    if (attempt > 1)
    {
      LOGF("[HTTP] Retry %d/%d\n", attempt, MAX_RETRIES);
      delay(RETRY_BACKOFF_MS * attempt);
    }

    // ── TCP / TLS connect ────────────────────────────────────────────
    LOGF("[HTTP] Connecting to %s:%d...\n", SERVER_HOST, SERVER_PORT);

    if (!netClient.connect(SERVER_HOST, SERVER_PORT))
    {
      LOG("[HTTP]  TCP connect failed");
      netClient.stop();
      continue;
    }

    // ── Write request ────────────────────────────────────────────────
    netClient.print(String("POST ") + path + " HTTP/1.1\r\n");
    netClient.print(String("Host: ") + SERVER_HOST + "\r\n");
    netClient.print("Content-Type: application/json\r\n");
    netClient.print("Content-Length: " + String(body.length()) + "\r\n");
    netClient.print("Connection: close\r\n");

    // Device auth headers (for /data endpoint)
    if (devId && devSecret)
    {
      netClient.print(String("x-device-id: ") + devId + "\r\n");
      netClient.print(String("x-device-secret: ") + devSecret + "\r\n");
    }

    netClient.print("\r\n"); // End headers
    netClient.print(body);   // JSON body

    // ── Wait for response ────────────────────────────────────────────
    uint32_t t0 = millis();
    while (!netClient.available() && millis() - t0 < HTTP_TIMEOUT_MS)
    {
      delay(50);
    }

    if (!netClient.available())
    {
      LOG("[HTTP]  Response timeout");
      netClient.stop();
      continue;
    }

    // ── Parse status line: "HTTP/1.1 201 Created" ────────────────────
    String statusLine = netClient.readStringUntil('\n');
    int statusCode = -1;
    if (statusLine.length() > 12)
    {
      statusCode = statusLine.substring(9, 12).toInt();
    }
    LOGF("[HTTP] ← %d\n", statusCode);

    // Skip headers
    while (netClient.available())
    {
      String line = netClient.readStringUntil('\n');
      if (line.length() <= 1)
        break;
    }

    // Read response body (for debug) - optimized to avoid String fragmentation
    char respBuffer[256];
    int respIndex = 0;
    uint32_t readStart = millis();
    while (netClient.available() && millis() - readStart < 3000)
    {
      char c = netClient.read();
      if (respIndex < (sizeof(respBuffer) - 1))
      {
        respBuffer[respIndex++] = c;
      }
    }
    respBuffer[respIndex] = '\0';
    if (respIndex > 0)
    {
      LOGF("[HTTP] Body: %s\n", respBuffer);
    }

    netClient.stop();

    // ── Success? ─────────────────────────────────────────────────────
    if (statusCode >= 200 && statusCode < 300)
    {
      return statusCode;
    }

    // 4xx client errors (except 408, 429) → don't retry
    if (statusCode >= 400 && statusCode < 500 && statusCode != 408 && statusCode != 429)
    {
      return statusCode;
    }
  }

  LOG("[HTTP]  All retries failed");
  return -1;
}

// ═════════════════════════════════════════════════════════════════════
//  UTILS
// ═════════════════════════════════════════════════════════════════════
float round1(float v) { return ((int)(v * 10 + 0.5)) / 10.0; }
float round2(float v) { return ((int)(v * 100 + 0.5)) / 100.0; }
float round3(float v) { return ((int)(v * 1000 + 0.5)) / 1000.0; }
