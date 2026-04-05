// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS_v6.ino — Crux IoT · v6
// ─────────────────────────────────────────────────────────────────────
//  Board     : VVM601 (ESP32-S3 + Quectel EC200U 4G LTE)
//  Libraries : TinyGSM, DHT (Adafruit), ArduinoJson v7+, Adafruit INA219
//
//  v6 vs v4/v5:
//    • SOC engine from v5: 3S Li-ion LUT (9.0–12.6 V), voltage glitch
//      rejection, charge-state detection, gentle 2% idle drift correction,
//      per-cell temperature compensation with CELL_COUNT
//    • 3 × DHT22 sensors only (DHT11 removed)
//    • INA219 calibrated to 32V / 2A range on init
//    • Signed current (+ = charging, − = discharging) stored & sent
//    • chargeState string included in every server payload
//    • Serial dashboard printed every READ_INTERVAL_MS (2 s)
//    • Server POST once per SEND_INTERVAL_MS (60 s) via 4G modem
//    • Adaptive loop delay + watchdog from v4
//
//  Bring-up: set ENABLE_DEBUG true in config.h, fill secrets.h
// ═══════════════════════════════════════════════════════════════════════

#include "config.h"
#include <esp_task_wdt.h>

// ── TinyGSM must be configured BEFORE the include ────────────────────
#define TINY_GSM_MODEM_BG96
#define TINY_GSM_USE_GPRS true
#define SerialAT  Serial1
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

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// ─────────────────────────────────────────────────────────────────────
//  SENSOR OBJECTS  — 3 × DHT22
// ─────────────────────────────────────────────────────────────────────
Adafruit_INA219 ina219;

DHT dhtSensors[TEMP_SENSOR_COUNT] = {
    DHT(DHT0_PIN, DHT0_TYPE),
    DHT(DHT1_PIN, DHT1_TYPE),
    DHT(DHT2_PIN, DHT2_TYPE),
};

const uint8_t dhtPins[TEMP_SENSOR_COUNT]  = { DHT0_PIN,  DHT1_PIN,  DHT2_PIN  };
const uint8_t dhtTypes[TEMP_SENSOR_COUNT] = { DHT0_TYPE, DHT1_TYPE, DHT2_TYPE };

// ─────────────────────────────────────────────────────────────────────
//  CHARGE STATE
// ─────────────────────────────────────────────────────────────────────
enum ChargeState { CHARGING, DISCHARGING, IDLE_FLOAT };
ChargeState chargeState = IDLE_FLOAT;

const char *chargeStateStr(ChargeState s)
{
    switch (s)
    {
        case CHARGING:    return "CHARGING";
        case DISCHARGING: return "DISCHARGING";
        case IDLE_FLOAT:  return "IDLE_FLOAT";
    }
    return "UNKNOWN";
}

// ─────────────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────────────
bool     registered    = false;
uint32_t lastSendMs    = 0;
uint32_t lastReadMs    = 0;
uint32_t bootTime      = 0;
uint32_t readCount     = 0;
uint32_t successCount  = 0;
uint32_t failCount     = 0;

float sensorTemp     = 0.0;
float sensorHumid    = 0.0;
float sensorCurrent  = 0.0;   // signed A: + = charging, − = discharging
float sensorVoltage  = 0.0;
float sensorPower    = 0.0;
float sensorSOC      = 0.0;

// SOC state
float    socEstimate     = -1.0;
uint32_t lastCoulombMs   = 0;
float    lastValidVoltage = 0.0;

// Forward declaration
static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret);

// ═════════════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ═════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);

    LOG("═══════════════════════════════════════════════");
    LOG("  Crux IoT · Firmware v" FIRMWARE_VER);
    LOG("  Device : " DEVICE_NAME);
    LOG("  ID     : " DEVICE_ID);
    LOG("  Server : " SERVER_HOST);
    LOGF("  Mode   : %s (port %d)\n", USE_HTTPS ? "HTTPS" : "HTTP", SERVER_PORT);
    LOG("  Build  : CruxHTTPS_v6  (3×DHT22 + INA219 + 3S SOC)");
    LOG("═══════════════════════════════════════════════\n");

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
        .timeout_ms      = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask  = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic   = true
    };
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
    uint32_t now = millis();

    // ── Fast sensor read + dashboard every READ_INTERVAL_MS ──────────
    if (now - lastReadMs >= READ_INTERVAL_MS)
    {
        lastReadMs = now;
        readCount++;
        readAllSensors();
        printDashboard();
    }

    // ── Server POST every SEND_INTERVAL_MS ───────────────────────────
    ensureConnected();
    if (!registered)
        registerWithRetry();

    now = millis();
    if (registered && (now - lastSendMs >= SEND_INTERVAL_MS))
    {
        lastSendMs = now;
        bool ok = sendSensorData();
        if (ok) successCount++;
        else    failCount++;

        uint32_t uptimeSec = (millis() - bootTime) / 1000;
        LOGF("[STATS] Uptime: %lum %lus | Sent: %lu ok, %lu fail\n",
             uptimeSec / 60, uptimeSec % 60, successCount, failCount);
    }

    esp_task_wdt_reset();

    // ── Adaptive loop delay ───────────────────────────────────────────
    uint32_t delayMs = LOOP_DELAY_MS;
#if ADAPTIVE_LOOP_DELAY
    if (registered && lastSendMs != 0)
    {
        uint32_t nextSend = lastSendMs + SEND_INTERVAL_MS;
        now = millis();
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
//  SENSOR INIT
// ═════════════════════════════════════════════════════════════════════

void initSensors()
{
    LOG("[SENSOR] Initializing...");

    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
        dhtSensors[i].begin();
        LOGF("[SENSOR]   DHT%d on GPIO %d  (sensor #%d)\n",
             dhtTypes[i], dhtPins[i], i);
    }

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!ina219.begin())
    {
        LOG("[SENSOR]   WARNING: INA219 NOT found — check wiring!");
        LOGF("[SENSOR]   SDA=GPIO%d  SCL=GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    }
    else
    {
        // 32V / 2A range covers 12.6V battery and normal charge/discharge
        ina219.setCalibration_32V_2A();
        LOG("[SENSOR]   INA219 ready  (32V / 2A calibration)");
    }

    LOG("[SENSOR] Init complete\n");
}

// ═════════════════════════════════════════════════════════════════════
//  SENSOR READ
// ═════════════════════════════════════════════════════════════════════

void readAllSensors()
{
    LOG("[SENSOR] ── Reading ──");

    // ── Temperature (average of valid DHT22 readings) ─────────────────
    float tempSum  = 0.0;
    int   tempCount = 0;

    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
        float t = dhtSensors[i].readTemperature();
        if (!isnan(t))
        {
            tempSum += t;
            tempCount++;
            LOGF("[SENSOR]   DHT22 #%d (GPIO %d): %.1f °C\n", i, dhtPins[i], t);
        }
        else
        {
            LOGF("[SENSOR]   DHT22 #%d (GPIO %d): ⚠ read failed — skipped\n",
                 i, dhtPins[i]);
        }
        // Pet watchdog between reads (DHT is slow)
        esp_task_wdt_reset();
    }

    if (tempCount > 0)
    {
        sensorTemp = tempSum / tempCount;
        LOGF("[SENSOR]   Avg Temp (%d/%d valid): %.1f °C\n",
             tempCount, TEMP_SENSOR_COUNT, sensorTemp);
    }
    else
    {
        LOG("[SENSOR]   ALL temp sensors failed — reinitializing DHT drivers");
        for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
            dhtSensors[i].begin();
    }

    // ── Humidity (first DHT22) ────────────────────────────────────────
    float h = dhtSensors[0].readHumidity();
    if (!isnan(h))
        sensorHumid = h;
    else
        LOG("[SENSOR]   Humidity read failed — using previous value");

    esp_task_wdt_reset();

    // ── INA219  ───────────────────────────────────────────────────────
    float shuntVoltage_mV = ina219.getShuntVoltage_mV();
    float busVoltage_V    = ina219.getBusVoltage_V();
    float current_mA      = ina219.getCurrent_mA();

    float rawVoltage = busVoltage_V + (shuntVoltage_mV / 1000.0);

    // Voltage glitch rejection: ignore readings below 3V (wiring noise)
    if (rawVoltage >= 3.0)
    {
        sensorVoltage     = rawVoltage;
        lastValidVoltage  = rawVoltage;
    }
    else if (lastValidVoltage > 0.0)
    {
        sensorVoltage = lastValidVoltage;
        LOG("[SENSOR]   Voltage glitch (<3V) — holding last valid reading");
    }
    else
    {
        sensorVoltage = rawVoltage;   // no valid reading yet
    }

    // Signed current: +charging / −discharging
    float signedCurrent_A = (current_mA / 1000.0) * CHARGE_DIRECTION;
    sensorCurrent = signedCurrent_A;

    sensorPower = ina219.getPower_mW() / 1000.0;

    // ── Charge state ──────────────────────────────────────────────────
    if (signedCurrent_A > CHARGE_CURRENT_THRESH_A)
        chargeState = CHARGING;
    else if (signedCurrent_A < -DISCHARGE_CURRENT_THRESH_A)
        chargeState = DISCHARGING;
    else
        chargeState = IDLE_FLOAT;

    // ── SOC ───────────────────────────────────────────────────────────
    sensorSOC = calculateSOC(sensorVoltage, signedCurrent_A, sensorTemp);

    LOGF("[SENSOR]   Humid   : %.1f %%\n",   sensorHumid);
    LOGF("[SENSOR]   Voltage : %.2f V\n",    sensorVoltage);
    LOGF("[SENSOR]   Current : %+.3f A\n",   sensorCurrent);
    LOGF("[SENSOR]   Power   : %.2f W\n",    sensorPower);
    LOGF("[SENSOR]   SOC     : %.1f %%\n",   sensorSOC);
    LOGF("[SENSOR]   State   : %s\n",        chargeStateStr(chargeState));
    LOG("[SENSOR] ────────────");
}

// ═════════════════════════════════════════════════════════════════════
//  SOC — v5 engine (3S Li-ion, voltage glitch-safe, gentle drift fix)
// ═════════════════════════════════════════════════════════════════════

// Voltage-to-SOC LUT for 3S 18650 Li-ion pack (9.0 – 12.6 V)
// Per-cell curve × 3: 3.00 V (0%) → 4.20 V (100%)
float voltageToSOC(float v)
{
    static const float LUT_V[] = {
         9.00,   // 0%   BMS cutoff
         9.90,   // 10%
        10.50,   // 20%
        10.80,   // 30%
        11.10,   // 40%
        11.25,   // 50%
        11.40,   // 60%
        11.55,   // 70%
        11.85,   // 80%
        12.30,   // 90%
        12.60    // 100% fully charged
    };
    static const float LUT_SOC[] = {
          0.0,  10.0,  20.0,  30.0,  40.0,
         50.0,  60.0,  70.0,  80.0,  90.0, 100.0
    };
    const int N = 11;

    if (v <= LUT_V[0])     return 0.0;
    if (v >= LUT_V[N - 1]) return 100.0;

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

// Per-cell temperature compensation — ~3 mV/°C per cell, referenced to 25°C
float temperatureCompensate(float v, float tempC)
{
    const float CELLS              = (float)CELL_COUNT;
    const float MV_PER_CELL_PER_C = 0.003f;
    return v + (25.0f - tempC) * CELLS * MV_PER_CELL_PER_C;
}

float calculateSOC(float rawVoltage, float signedCurrentA, float tempC)
{
    float compV      = temperatureCompensate(rawVoltage, tempC);
    float voltageSoc = voltageToSOC(compV);

    // Seed from voltage on first call
    if (socEstimate < 0.0f)
    {
        socEstimate   = voltageSoc;
        lastCoulombMs = millis();
        LOGF("[SOC] ★ Initialized from voltage lookup: %.1f%%\n", socEstimate);
        return socEstimate;
    }

    // Coulomb counting: dSOC = (I × dt / C_ah) × 100
    uint32_t nowMs        = millis();
    float    elapsedHours = (nowMs - lastCoulombMs) / 3600000.0f;
    lastCoulombMs = nowMs;

    float deltaSOC = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0f;
    socEstimate   += deltaSOC;
    socEstimate    = constrain(socEstimate, 0.0f, 100.0f);

    LOGF("[SOC] V=%.2fV (comp=%.2fV) | T=%.1f°C | I=%+.3fA\n",
         rawVoltage, compV, tempC, signedCurrentA);
    LOGF("[SOC] Δt=%.4fh | ΔSOC=%+.3f%% | Coulomb→%.1f%% | VLookup=%.1f%%\n",
         elapsedHours, deltaSOC, socEstimate, voltageSoc);

    // Gentle idle drift correction (2% blend) only when no meaningful current
    if (fabsf(signedCurrentA) < CHARGE_CURRENT_THRESH_A)
    {
        float before = socEstimate;
        socEstimate  = 0.98f * socEstimate + 0.02f * voltageSoc;
        LOGF("[SOC] Idle drift correction: %.1f%% → %.1f%%\n", before, socEstimate);
    }

    return socEstimate;
}

// ═════════════════════════════════════════════════════════════════════
//  SERIAL DASHBOARD  (prints every READ_INTERVAL_MS)
// ═════════════════════════════════════════════════════════════════════

void printDashboard()
{
    uint32_t uptimeSec = (millis() - bootTime) / 1000;

    // ANSI clear-screen (works in PuTTY / minicom; Arduino IDE appends)
    Serial.print("\033[2J\033[H");

    Serial.println("+----------------------------------------------+");
    Serial.println("|          CRUX BATTERY MONITOR  v6            |");
    Serial.println("+----------------------------------------------+");
    LOGF("|  Reading #%-6lu       Uptime: %02lu:%02lu:%02lu    |\n",
         readCount,
         uptimeSec / 3600, (uptimeSec / 60) % 60, uptimeSec % 60);
    Serial.println("+----------------------------------------------+");

    LOGF("|  Status  : %-33s|\n", chargeStateStr(chargeState));

    // SOC bar
    const int barLen = 20;
    int filled = (int)(sensorSOC / 100.0f * barLen);
    if (filled > barLen) filled = barLen;
    char bar[22];
    for (int i = 0; i < barLen; i++)
        bar[i] = (i < filled) ? '#' : '-';
    bar[barLen] = '\0';
    LOGF("|  SOC     : %5.1f %%  [%s]  |\n", sensorSOC, bar);

    Serial.println("+----------------------------------------------+");
    LOGF("|  Voltage : %6.2f V                          |\n", sensorVoltage);
    LOGF("|  Current : %+7.3f A                         |\n", sensorCurrent);
    LOGF("|  Power   : %6.2f W                          |\n", sensorPower);
    LOGF("|  Per-Cell: %5.2f V  (%dS)                    |\n",
         sensorVoltage / CELL_COUNT, CELL_COUNT);
    Serial.println("+----------------------------------------------+");
    LOGF("|  Temp    : %5.1f C                            |\n", sensorTemp);
    LOGF("|  Humid   : %5.1f %%                            |\n", sensorHumid);
    Serial.println("+----------------------------------------------+");

    // ── Network status ────────────────────────────────────────────────
    LOGF("|  Server  : %-33s|\n", registered ? "Registered" : "Not registered");
    LOGF("|  TX OK   : %-6lu   TX Fail: %-6lu            |\n",
         successCount, failCount);
    Serial.println("+----------------------------------------------+");

    // ── Alerts ────────────────────────────────────────────────────────
    if (sensorVoltage > BATTERY_FULL_V + 0.1f)
        Serial.println("|  !! WARNING: VOLTAGE ABOVE FULL CHARGE      |");
    if (sensorVoltage < BATTERY_EMPTY_V)
        Serial.println("|  !! WARNING: VOLTAGE BELOW BMS CUTOFF       |");
    if (chargeState == CHARGING && sensorSOC >= 99.5f)
        Serial.println("|  ✓  FULL: Battery fully charged (float)      |");
    if (chargeState == DISCHARGING && sensorSOC < 10.0f)
        Serial.println("|  !! LOW BATTERY: Connect charger             |");

    Serial.println("+----------------------------------------------+");
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
        LOG("[MODEM] restart() failed — continuing anyway");

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
//  HTTP  (stack-allocated headers, no Arduino String in request path)
// ═════════════════════════════════════════════════════════════════════

void registerWithRetry()
{
    LOG("[REG] Registering device...");

#if CRUX_USE_JSONDOC_V6
    StaticJsonDocument<JSON_DOC_CAPACITY> doc;
#else
    JsonDocument doc;
#endif
    doc["deviceId"]       = DEVICE_ID;
    doc["deviceSecret"]   = DEVICE_SECRET;
    doc["deviceName"]     = DEVICE_NAME;
    doc["firmwareVersion"] = FIRMWARE_VER;

    char   jsonBuf[JSON_SERIAL_BUFFER];
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
            LOG("[REG] Registered ✓\n");
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
    doc["humidity"]    = round1(sensorHumid);
    doc["voltage"]     = round2(sensorVoltage);
    doc["current"]     = round3(sensorCurrent);  // signed
    doc["power"]       = round2(sensorPower);
    doc["soc"]         = round1(sensorSOC);

    char   jsonBuf[JSON_SERIAL_BUFFER];
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
        LOG("[DATA] ✓ 201 Accepted\n");
        return true;
    }

    LOGF("[DATA] HTTP %d\n", code);
    if (code == 404)
        registered = false;   // trigger re-registration
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

        LOGF("[HTTP] Connecting %s:%d\n", SERVER_HOST, SERVER_PORT);
        if (!netClient.connect(SERVER_HOST, SERVER_PORT))
        {
            LOG("[HTTP] TCP connect failed");
            netClient.stop();
            continue;
        }

        // Build request headers on the stack
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
            LOG("[HTTP] Header base truncated");
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
                LOG("[HTTP] Auth header truncated");
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

        // Wait for response
        uint32_t t0 = millis();
        while (!netClient.available() && millis() - t0 < HTTP_TIMEOUT_MS)
            delay(50);

        if (!netClient.available())
        {
            LOG("[HTTP] Response timeout");
            netClient.stop();
            continue;
        }

        // Parse status line
        String statusLine = netClient.readStringUntil('\n');
        int    statusCode = -1;
        if (statusLine.length() > 12)
            statusCode = statusLine.substring(9, 12).toInt();
        LOGF("[HTTP] ← %d\n", statusCode);

        // Drain headers
        while (netClient.available())
        {
            String line = netClient.readStringUntil('\n');
            if (line.length() <= 1)
                break;
        }

        // Log response body snippet
        char     respBuf[HTTP_RESP_BODY_MAX];
        size_t   respN     = 0;
        uint32_t readStart = millis();
        while (netClient.available() &&
               millis() - readStart < 3000 &&
               respN < sizeof(respBuf) - 1)
        {
            respBuf[respN++] = (char)netClient.read();
        }
        respBuf[respN] = '\0';
        if (respN > 0)
            LOGF("[HTTP] Body: %s\n", respBuf);

        netClient.stop();

        if (statusCode >= 200 && statusCode < 300)
            return statusCode;
        // Hard client errors — no point retrying (except 408/429)
        if (statusCode >= 400 && statusCode < 500 &&
            statusCode != 408 && statusCode != 429)
            return statusCode;
    }

    LOG("[HTTP] All retries failed");
    return -1;
}

// ─────────────────────────────────────────────────────────────────────
//  ROUNDING HELPERS
// ─────────────────────────────────────────────────────────────────────
float round1(float v) { return ((int)(v * 10.0f   + 0.5f)) / 10.0f;   }
float round2(float v) { return ((int)(v * 100.0f  + 0.5f)) / 100.0f;  }
float round3(float v) { return ((int)(v * 1000.0f + 0.5f)) / 1000.0f; }
