// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS_v8.ino — Crux IoT · v8
// ─────────────────────────────────────────────────────────────────────
//  Board     : VVM601 (ESP32-S3 + Quectel EC200U 4G LTE)
//  Libraries : TinyGSM, DHT (Adafruit), ArduinoJson v7+,
//              Adafruit INA219, Preferences (built-in ESP32)
//
//  v8 fixes over v6/v7:
//    ✔ Voltage LUT seeds SOC on first boot (no more "100%" default)
//    ✔ Saved SOC sanity-checked against live voltage on every reboot
//    ✔ Temperature compensation restored
//    ✔ Idle drift correction restored (voltage blend when current ≈ 0)
//    ✔ DS3231 RTC used for Δt  (falls back to millis() if not found)
//    ✔ NVS (Preferences) used for persistence — built-in wear levelling
//    ✔ Self-discharge correction when restoring SOC after power-off
//
//  Wiring (all three chips share one I2C bus):
//    ESP32 GPIO 8  (SDA) ─── INA219 SDA  ─── DS3231 SDA
//    ESP32 GPIO 9  (SCL) ─── INA219 SCL  ─── DS3231 SCL
// ═══════════════════════════════════════════════════════════════════════

#include "config.h"
#include <esp_task_wdt.h>

// ── TinyGSM ──────────────────────────────────────────────────────────
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

// ── Hardware ──────────────────────────────────────────────────────────
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <Preferences.h>

Adafruit_INA219 ina219;
RTC_DS3231      rtc;
Preferences     prefs;

DHT dhtSensors[TEMP_SENSOR_COUNT] = {
    DHT(DHT0_PIN, DHT0_TYPE),
    DHT(DHT1_PIN, DHT1_TYPE),
    DHT(DHT2_PIN, DHT2_TYPE),
};
const uint8_t dhtPins[TEMP_SENSOR_COUNT]  = {DHT0_PIN, DHT1_PIN, DHT2_PIN};
const uint8_t dhtTypes[TEMP_SENSOR_COUNT] = {DHT0_TYPE, DHT1_TYPE, DHT2_TYPE};

// ═════════════════════════════════════════════════════════════════════
//  GLOBALS
// ═════════════════════════════════════════════════════════════════════
bool     registered   = false;
uint32_t lastSendMs   = 0;
uint32_t bootTime     = 0;
uint32_t successCount = 0;
uint32_t failCount    = 0;

float sensorTemp    = 0.0;
float sensorHumid   = 0.0;
float sensorCurrent = 0.0;   // absolute (A), for server
float sensorVoltage = 0.0;
float sensorPower   = 0.0;
float sensorSOC     = 0.0;

// SOC engine state
float    socEstimate      = -1.0;   // -1 = not yet seeded
uint32_t lastCoulombUnix  = 0;      // RTC unix timestamp of last update
uint32_t lastCoulombMs    = 0;      // millis() fallback timestamp
float    lastValidVoltage = 0.0;    // for glitch rejection

bool rtcAvailable = false;
bool rtcTimeValid = false;

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

    LOG("═══════════════════════════════════════════════");
    LOG("  Crux IoT · Firmware v" FIRMWARE_VER);
    LOG("  Device : " DEVICE_NAME "  ID: " DEVICE_ID);
    LOG("  Build  : CruxHTTPS_v8");
    LOG("═══════════════════════════════════════════════\n");

    initSensors();
    restoreSOCFromNVS();   // load + sanity-check before counting starts

    powerOnModem();
    initModem();
    waitForNetwork();
    connectGPRS();
    registerWithRetry();

    bootTime = millis();
    LOG("\n[BOOT] Setup complete — entering main loop\n");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic  = true};
    esp_task_wdt_init(&wdt_cfg);
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
    if (!registered) registerWithRetry();

    uint32_t now = millis();

    if (registered && (now - lastSendMs >= SEND_INTERVAL_MS))
    {
        lastSendMs = now;

        readAllSensors();
        saveSOCToNVS();
        bool ok = sendSensorData();

        if (ok) successCount++;
        else    failCount++;

        uint32_t upSec = (millis() - bootTime) / 1000;
        LOGF("[STATS] Uptime: %lum %lus | ok:%lu fail:%lu\n",
             upSec / 60, upSec % 60, successCount, failCount);
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
            if (until < delayMs) delayMs = until;
        }
    }
    if (delayMs < (uint32_t)LOOP_DELAY_MIN_MS) delayMs = LOOP_DELAY_MIN_MS;
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
        LOGF("[SENSOR]   DHT%d on GPIO %d (sensor #%d)\n",
             dhtTypes[i], dhtPins[i], i);
    }

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // Quick I2C scan for diagnostics
    LOG("[I2C] Scanning bus:");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
            LOGF("[I2C]   0x%02X\n", addr);
    }

    // INA219 at 0x40
    if (!ina219.begin())
        LOG("[SENSOR]   WARNING: INA219 not found at 0x40");
    else
    {
        ina219.setCalibration_32V_2A();
        LOG("[SENSOR]   INA219 ready");
    }

    // DS3231 at 0x68
    if (!rtc.begin())
    {
        LOG("[RTC]   DS3231 not found — Δt uses millis()");
        rtcAvailable = false;
    }
    else
    {
        rtcAvailable = true;

        if (rtc.lostPower())
        {
            // No coin cell — set time to compile time so year() is sane
            LOG("[RTC]   Lost power / no coin cell — setting to compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }

        DateTime t = rtc.now();
        // Treat year >= 2024 as valid
        rtcTimeValid = (t.year() >= 2024);

        char buf[32];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                 t.year(), t.month(), t.day(),
                 t.hour(), t.minute(), t.second());
        LOGF("[RTC]   DS3231 ready — %s  (valid: %s)\n",
             buf, rtcTimeValid ? "yes" : "no");
    }

    LOG("[SENSOR] Ready\n");
}

// ═════════════════════════════════════════════════════════════════════
//  SENSOR READING
// ═════════════════════════════════════════════════════════════════════

void readAllSensors()
{
    LOG("[SENSOR] ── Reading ──");

    // ── Temperature ────────────────────────────────────────────────
    float tempSum = 0.0; int tempCount = 0;
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
        float t = dhtSensors[i].readTemperature();
        if (!isnan(t)) { tempSum += t; tempCount++; LOGF("[SENSOR]   DHT #%d: %.1f°C\n", i, t); }
        else           LOGF("[SENSOR]   DHT #%d: ⚠ failed\n", i);
        if ((i & 1) == 1) esp_task_wdt_reset();
    }
    if (tempCount > 0)
        sensorTemp = tempSum / tempCount;
    else
    {
        LOG("[SENSOR]   All DHT failed — soft-reset");
        for (int i = 0; i < TEMP_SENSOR_COUNT; i++) dhtSensors[i].begin();
    }

    // ── Humidity ───────────────────────────────────────────────────
    float h = dhtSensors[0].readHumidity();
    if (!isnan(h)) sensorHumid = h;
    else LOG("[SENSOR]   Humidity failed, keeping previous");

    esp_task_wdt_reset();

    // ── INA219 ─────────────────────────────────────────────────────
    float shunt_mV   = ina219.getShuntVoltage_mV();
    float bus_V      = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float rawVoltage = bus_V + (shunt_mV / 1000.0);

    // Glitch rejection: sub-3V means disconnected wire or noise
    if (rawVoltage >= 3.0)
    {
        sensorVoltage    = rawVoltage;
        lastValidVoltage = rawVoltage;
    }
    else if (lastValidVoltage > 0.0)
    {
        sensorVoltage = lastValidVoltage;
        LOG("[SENSOR]   Voltage glitch — using last valid");
    }
    else
    {
        sensorVoltage = rawVoltage;
    }

    float signedCurrent_A = (current_mA / 1000.0) * (float)ACS712_CHARGE_DIRECTION;
    sensorCurrent = fabs(signedCurrent_A);
    sensorPower   = ina219.getPower_mW() / 1000.0;

    // ── SOC ────────────────────────────────────────────────────────
    sensorSOC = calculateSOC(sensorVoltage, signedCurrent_A, sensorTemp);

    LOGF("[SENSOR]   Humid   : %.1f %%\n", sensorHumid);
    LOGF("[SENSOR]   Voltage : %.2f V\n",  sensorVoltage);
    LOGF("[SENSOR]   Current : %.3f A (%s)\n",
         sensorCurrent, signedCurrent_A >= 0 ? "charging" : "discharging");
    LOGF("[SENSOR]   Power   : %.2f W\n",  sensorPower);
    LOGF("[SENSOR]   SOC     : %.1f %%\n", sensorSOC);
    LOG("[SENSOR] ────────────");
}

// ═════════════════════════════════════════════════════════════════════
//  SOC ENGINE
//
//  Three layers, all restored from v4/v5 and improved:
//    1. Voltage LUT — seeds SOC on first boot and on reboot sanity check
//    2. Temperature compensation — corrects voltage reading for battery temp
//    3. Coulomb counting — integrates signed current × Δt
//       + Idle drift correction — blends toward voltage when current ≈ 0
//
//  Δt source: DS3231 UNIX timestamps when available, millis() fallback.
// ═════════════════════════════════════════════════════════════════════

// Layer 1: OCV → SOC lookup (12V lead-acid, same LUT used in v4)
float voltageToSOC(float v)
{
    static const float LUT_V[]   = {10.50, 11.31, 11.58, 11.75, 11.90,
                                     12.06, 12.20, 12.32, 12.50, 12.70, 14.40};
    static const float LUT_SOC[] = {  0.0,  10.0,  20.0,  30.0,  40.0,
                                      50.0,  60.0,  70.0,  80.0,  90.0, 100.0};
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

// Layer 2: temperature compensation for lead-acid
// −3 mV per cell per °C from 25 °C reference; 12 V battery = 6 cells
float temperatureCompensate(float v, float tempC)
{
    return v + (25.0 - tempC) * 6.0 * 0.003;
}

// Layer 3: full SOC engine
float calculateSOC(float rawVoltage, float signedCurrentA, float tempC)
{
    float compV      = temperatureCompensate(rawVoltage, tempC);
    float voltageSoc = voltageToSOC(compV);

    // ── First call: seed from voltage, never from a hardcoded value ──
    if (socEstimate < 0.0)
    {
        if (rawVoltage < 3.0)
        {
            LOG("[SOC] Waiting for valid battery voltage...");
            return 0.0;
        }
        socEstimate = voltageSoc;

        // Seed timestamps
        if (rtcAvailable && rtcTimeValid)
            lastCoulombUnix = (uint32_t)rtc.now().unixtime();
        lastCoulombMs = millis();

        LOGF("[SOC] ★ Seeded from voltage LUT: %.1f%%  (V=%.2f comp=%.2f)\n",
             socEstimate, rawVoltage, compV);
        return socEstimate;
    }

    // ── Compute elapsed time ─────────────────────────────────────────
    float elapsedHours = 0.0;

    if (rtcAvailable && rtcTimeValid && lastCoulombUnix > 0)
    {
        uint32_t nowUnix  = (uint32_t)rtc.now().unixtime();
        uint32_t deltaSec = (nowUnix > lastCoulombUnix) ? (nowUnix - lastCoulombUnix) : 0;

        // Sanity guard: if dt > 10 min something is wrong (watchdog reset?)
        // Cap at SEND_INTERVAL_MS × 3 to limit coulomb overshoot
        uint32_t maxSec = (SEND_INTERVAL_MS / 1000) * 3;
        if (deltaSec > maxSec) deltaSec = maxSec;

        elapsedHours    = deltaSec / 3600.0;
        lastCoulombUnix = nowUnix;
        LOGF("[SOC] Δt: DS3231  %lu s = %.5f h\n", deltaSec, elapsedHours);
    }
    else
    {
        uint32_t nowMs    = millis();
        uint32_t deltaMs  = nowMs - lastCoulombMs;
        uint32_t maxMs    = SEND_INTERVAL_MS * 3;
        if (deltaMs > maxMs) deltaMs = maxMs;

        elapsedHours  = deltaMs / 3600000.0;
        lastCoulombMs = nowMs;
        LOGF("[SOC] Δt: millis()  %.5f h\n", elapsedHours);
    }

    // ── Coulomb counting ─────────────────────────────────────────────
    //   positive signedCurrentA = charging  → SOC increases
    //   negative signedCurrentA = discharging → SOC decreases
    float deltaSOC  = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0;
    socEstimate    += deltaSOC;
    socEstimate     = constrain(socEstimate, 0.0, 100.0);

    LOGF("[SOC] V=%.2f(comp=%.2f) T=%.1f°C I=%+.3fA\n",
         rawVoltage, compV, tempC, signedCurrentA);
    LOGF("[SOC] ΔSOC=%+.3f%%  Coulomb→%.1f%%  VLookup=%.1f%%\n",
         deltaSOC, socEstimate, voltageSoc);

    // ── Idle drift correction ────────────────────────────────────────
    //   When battery is at rest, OCV is a reliable ground-truth.
    //   Gently blend toward voltage SOC (SOC_VOLTAGE_BLEND weight).
    //   This self-corrects coulomb drift without causing sudden jumps.
    if (fabs(signedCurrentA) < SOC_IDLE_CURRENT_THRESH_A)
    {
        float before = socEstimate;
        socEstimate  = (1.0f - SOC_VOLTAGE_BLEND) * socEstimate
                       + SOC_VOLTAGE_BLEND * voltageSoc;
        LOGF("[SOC] Idle correction: %.1f%% → %.1f%%\n", before, socEstimate);
    }

    return socEstimate;
}

// ═════════════════════════════════════════════════════════════════════
//  NVS PERSISTENCE
// ═════════════════════════════════════════════════════════════════════

void saveSOCToNVS()
{
    if (socEstimate < 0.0) return;

    prefs.begin("crux", false);
    prefs.putFloat("soc", socEstimate);
    if (rtcAvailable && rtcTimeValid)
        prefs.putUInt("ts", (uint32_t)rtc.now().unixtime());
    prefs.end();

    LOGF("[NVS] Saved SOC=%.1f%%\n", socEstimate);
}

void restoreSOCFromNVS()
{
    prefs.begin("crux", true);
    float    savedSOC = prefs.getFloat("soc", -1.0);
    uint32_t savedTS  = prefs.getUInt("ts",    0);
    prefs.end();

    // ── No valid saved SOC ───────────────────────────────────────────
    // calculateSOC() will seed from voltage on first sensor read.
    if (savedSOC < 0.0 || savedSOC > 100.0)
    {
        LOG("[NVS] No saved SOC — will seed from voltage on first read");
        return;
    }

    // ── Sanity-check saved SOC against live voltage ──────────────────
    //   Read voltage directly here (before main loop starts).
    //   If saved SOC and voltage LUT disagree by more than SOC_REBOOT_MAX_DEVIATION,
    //   discard the saved SOC and re-seed from voltage.
    //   This catches cases where the battery was charged/discharged
    //   while the device was off.
    float shunt_mV   = ina219.getShuntVoltage_mV();
    float bus_V      = ina219.getBusVoltage_V();
    float liveVoltage = bus_V + (shunt_mV / 1000.0);

    if (liveVoltage >= 3.0)
    {
        float liveSOC = voltageToSOC(liveVoltage);
        float diff    = fabs(savedSOC - liveSOC);
        LOGF("[NVS] Saved=%.1f%%  LiveV=%.2fV  VLookup=%.1f%%  Diff=%.1f%%\n",
             savedSOC, liveVoltage, liveSOC, diff);

        if (diff > SOC_REBOOT_MAX_DEVIATION)
        {
            LOGF("[NVS] Deviation %.1f%% > %.1f%% limit — discarding saved SOC, re-seeding from voltage\n",
                 diff, SOC_REBOOT_MAX_DEVIATION);
            // socEstimate stays -1.0 → calculateSOC() seeds from voltage
            return;
        }
    }

    // ── Apply self-discharge for time board was off ──────────────────
    float restored = savedSOC;

    if (rtcAvailable && rtcTimeValid && savedTS > 0)
    {
        uint32_t nowUnix = (uint32_t)rtc.now().unixtime();
        uint32_t offSec  = (nowUnix > savedTS) ? (nowUnix - savedTS) : 0;

        if (offSec > NVS_MAX_RESTORE_SECONDS)
        {
            LOGF("[NVS] Off for %lu s > %lu s limit — re-seeding from voltage\n",
                 offSec, NVS_MAX_RESTORE_SECONDS);
            return;
        }

        float offHours  = offSec / 3600.0;
        float selfDisch = offHours * SELF_DISCHARGE_PCT_PER_HOUR;
        restored       -= selfDisch;
        restored        = constrain(restored, 0.0, 100.0);
        LOGF("[NVS] Off %.2fh → self-disch -%.2f%% → restored %.1f%%\n",
             offHours, selfDisch, restored);
    }
    else
    {
        // No RTC timestamp — accept saved SOC as-is (already passed voltage check)
        LOGF("[NVS] Restored SOC=%.1f%% (no RTC timestamp for off-time)\n", restored);
    }

    socEstimate     = restored;
    lastCoulombUnix = savedTS;
    lastCoulombMs   = millis();
}

// ═════════════════════════════════════════════════════════════════════
//  MODEM
// ═════════════════════════════════════════════════════════════════════

void powerOnModem()
{
    LOG("[MODEM] Powering on EC200U...");
    pinMode(MODEM_POWER, OUTPUT);
    digitalWrite(MODEM_POWER, LOW);  delay(100);
    digitalWrite(MODEM_POWER, HIGH); delay(1000);
    digitalWrite(MODEM_POWER, LOW);
    LOG("[MODEM] Power sequence done — waiting 5s for boot");
    delay(5000);
}

void initModem()
{
    SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    LOG("[MODEM] Initializing...");
    if (!modem.init())    LOG("[MODEM] init() failed — trying restart()");
    if (!modem.restart()) LOG("[MODEM] restart() failed — continuing");
    LOG("[MODEM] Name: " + modem.getModemName());
    LOG("[MODEM] Info: " + modem.getModemInfo());
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
        { delay(5000); ESP.restart(); }
    }
    LOG("[NET] GPRS connected — IP: " + modem.getLocalIP());
}

void ensureConnected()
{
    if (!modem.isNetworkConnected())
    { LOG("[NET] Network lost — reconnecting"); waitForNetwork(); connectGPRS(); }
    if (!modem.isGprsConnected())
    { LOG("[NET] GPRS lost — reconnecting"); connectGPRS(); }
}

// ═════════════════════════════════════════════════════════════════════
//  HTTP
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
    { LOG("[REG] JSON error"); return; }

    for (int i = 1; i <= 5; i++)
    {
        int code = httpPost(REGISTER_PATH, jsonBuf, jsonLen, nullptr, nullptr);
        if (code == 200 || code == 201)
        { registered = true; LOG("[REG] Registered\n"); return; }
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
    { LOG("[DATA] JSON error"); return false; }

    LOGF("[DATA] Payload: %s\n", jsonBuf);

    int code = httpPost(DATA_PATH, jsonBuf, jsonLen, DEVICE_ID, DEVICE_SECRET);
    if (code == 201) { LOG("[DATA] 201\n"); return true; }
    LOGF("[DATA] HTTP %d\n", code);
    if (code == 404) registered = false;
    return false;
}

static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret)
{
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++)
    {
        if (attempt > 1)
        { LOGF("[HTTP] Retry %d/%d\n", attempt, MAX_RETRIES); delay(RETRY_BACKOFF_MS * attempt); }

        LOGF("[HTTP] Connect %s:%d\n", SERVER_HOST, SERVER_PORT);
        if (!netClient.connect(SERVER_HOST, SERVER_PORT))
        { LOG("[HTTP] TCP failed"); netClient.stop(); continue; }

        char hdrs[HTTP_HEADER_BUFFER];
        int h = snprintf(hdrs, sizeof(hdrs),
            "POST %s HTTP/1.1\r\nHost: %s\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %u\r\nConnection: close\r\n",
            path, SERVER_HOST, (unsigned)bodyLen);

        if (h <= 0 || (size_t)h >= sizeof(hdrs) - 96)
        { LOG("[HTTP] header truncated"); netClient.stop(); continue; }

        int pos = h;
        if (devId && devSecret)
        {
            int a = snprintf(hdrs + pos, sizeof(hdrs) - pos,
                "x-device-id: %s\r\nx-device-secret: %s\r\n", devId, devSecret);
            if (a <= 0 || pos + a >= (int)sizeof(hdrs) - 8)
            { LOG("[HTTP] auth header truncated"); netClient.stop(); continue; }
            pos += a;
        }

        int fin = snprintf(hdrs + pos, sizeof(hdrs) - pos, "\r\n");
        if (fin < 2) { netClient.stop(); continue; }
        pos += fin;

        netClient.write((const uint8_t *)hdrs, pos);
        netClient.write((const uint8_t *)body, bodyLen);

        uint32_t t0 = millis();
        while (!netClient.available() && millis() - t0 < HTTP_TIMEOUT_MS) delay(50);

        if (!netClient.available())
        { LOG("[HTTP] timeout"); netClient.stop(); continue; }

        String statusLine = netClient.readStringUntil('\n');
        int statusCode = -1;
        if (statusLine.length() > 12) statusCode = statusLine.substring(9, 12).toInt();
        LOGF("[HTTP] ← %d\n", statusCode);

        while (netClient.available())
        { String line = netClient.readStringUntil('\n'); if (line.length() <= 1) break; }

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
