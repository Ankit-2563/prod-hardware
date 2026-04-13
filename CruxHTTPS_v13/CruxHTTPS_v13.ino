// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS_v11.ino — Crux IoT · v11
// ─────────────────────────────────────────────────────────────────────
//  Combines the best of all versions:
//    • SOC engine from ChargeSDLogger (Coulomb counting + rest-anchor +
//      NVS persistence + voltage LUT for 3S Li-ion)
//    • Temperature sensors from v10 (3x DHT22, 1x DHT11, averaged)
//    • Server registration + data sending from v4 (HTTPS via nginx
//      reverse proxy, stack-based headers, no String heap churn)
//    • SD card fallback logging when SIM absent or 4G unavailable
//    • Offline flush: on reconnection all unsynced SD rows are posted
//      to the server before resuming normal 60-second cadence
//
//  Wiring:
//    INA219  SDA→8   SCL→9   VCC→5V  GND→GND
//    DS3231  SDA→14  SCL→21  VCC→5V  GND→GND
//    SD Card MOSI→11 MISO→13 SCK→12  CS→10
//    DHT0    GPIO 4  (DHT22)
//    DHT1    GPIO 5  (DHT22)
//    DHT2    GPIO 16 (DHT22)
//    DHT3    GPIO 17 (DHT11)
//    EC200U  RX→40   TX→41   PWR→42
//
//  SD CSV columns:
//    row, timestamp, temperature_C, humidity_pct, voltage_V,
//    current_A, power_W, soc_pct, state, synced
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
#include <SPI.h>
#include <SD.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <Preferences.h>

// ─────────────────────────────────────────────────────────────────────
//  PERIPHERAL OBJECTS
// ─────────────────────────────────────────────────────────────────────
Adafruit_INA219 ina219;
RTC_DS3231      rtc;
Preferences     prefs;

DHT dhtSensors[TEMP_SENSOR_COUNT] = {
    DHT(DHT0_PIN, DHT0_TYPE),
    DHT(DHT1_PIN, DHT1_TYPE),
    DHT(DHT2_PIN, DHT2_TYPE),
    DHT(DHT3_PIN, DHT3_TYPE),
};
const uint8_t dhtPins[TEMP_SENSOR_COUNT]  = { DHT0_PIN, DHT1_PIN, DHT2_PIN, DHT3_PIN };
const uint8_t dhtTypes[TEMP_SENSOR_COUNT] = { DHT0_TYPE, DHT1_TYPE, DHT2_TYPE, DHT3_TYPE };

// ─────────────────────────────────────────────────────────────────────
//  RUNTIME FLAGS & COUNTERS
// ─────────────────────────────────────────────────────────────────────
bool     rtcAvailable   = false;
bool     rtcTimeValid   = false;
bool     sdReady        = false;
bool     modemReady     = false;
bool     networkUp      = false;
bool     registered     = false;

uint32_t bootTime       = 0;
uint32_t lastSendMs     = 0;
uint32_t lastLocalLogMs = 0;
uint32_t logRowCount    = 0;
uint32_t successCount   = 0;
uint32_t failCount      = 0;

// ─────────────────────────────────────────────────────────────────────
//  LIVE SENSOR VALUES (updated each read cycle)
// ─────────────────────────────────────────────────────────────────────
float sensorTemp    = 0.0f;
float sensorHumid   = 0.0f;
float sensorVoltage = 0.0f;
float sensorCurrent = 0.0f;   // always positive (magnitude)
float sensorPower   = 0.0f;
float sensorSOC     = 0.0f;
float signedCurrentA = 0.0f;  // sign = charging direction

// ─────────────────────────────────────────────────────────────────────
//  SOC ENGINE STATE  (identical to ChargeSDLogger)
// ─────────────────────────────────────────────────────────────────────
float    socEstimate      = -1.0f;
uint32_t lastCoulombUnix  = 0;
uint32_t lastCoulombMs    = 0;
float    lastValidVoltage = 0.0f;

// Rest-detection
uint32_t restStartMs = 0;
bool     isResting   = false;
bool     anchorDone  = false;

// ─────────────────────────────────────────────────────────────────────
//  CHARGE STATE ENUM
// ─────────────────────────────────────────────────────────────────────
typedef enum { STATE_IDLE, STATE_CHARGING, STATE_DISCHARGING } ChargeState;
ChargeState prevState = STATE_IDLE;

const char *stateLabel(ChargeState s)
{
    switch (s) {
        case STATE_CHARGING:    return "CHARGING";
        case STATE_DISCHARGING: return "DISCHARGING";
        default:                return "IDLE";
    }
}
ChargeState currentToState(float a)
{
    if (a >=  IDLE_THRESHOLD_A) return STATE_CHARGING;
    if (a <= -IDLE_THRESHOLD_A) return STATE_DISCHARGING;
    return STATE_IDLE;
}

// ─────────────────────────────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────
static int  httpPost(const char *path, const char *body, size_t bodyLen,
                     const char *devId, const char *devSecret);
void        flushOfflineRows();

// ═════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);

    LOG("═══════════════════════════════════════════════════════");
    LOG("  Crux IoT · Firmware v" FIRMWARE_VER);
    LOG("  Device : " DEVICE_NAME);
    LOG("  ID     : " DEVICE_ID);
    LOG("  Server : " SERVER_HOST);
    LOGF("  Mode   : %s (port %d)\n", USE_HTTPS ? "HTTPS" : "HTTP", SERVER_PORT);
    LOG("  Build  : CruxHTTPS_v11 (SD offline + ChargeSDLogger SOC)");
    LOG("═══════════════════════════════════════════════════════\n");

    initPeripherals();
    restoreSOCFromNVS();

    // ── Try to bring modem/network up (non-fatal) ─────────────────────
    modemReady = tryInitModem();
    if (modemReady) {
        networkUp = tryConnectNetwork();
        if (networkUp) registerWithRetry();
    } else {
        LOG("[BOOT] No modem — offline SD-only mode");
    }

    bootTime        = millis();
    lastLocalLogMs  = millis() - LOCAL_LOG_INTERVAL_MS;  // log immediately on boot
    lastSendMs      = millis() - SEND_INTERVAL_MS;

    // ── Watchdog ─────────────────────────────────────────────────────
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
    LOG("\n[BOOT] Setup complete — entering main loop\n");
}

// ═════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════

void loop()
{
    uint32_t now = millis();

    // ── Read sensors every second ────────────────────────────────────
    readAllSensors();
    saveSOCToNVS();

    // ── Local SD log every LOCAL_LOG_INTERVAL_MS (default 60 s) ──────
    if (now - lastLocalLogMs >= LOCAL_LOG_INTERVAL_MS) {
        lastLocalLogMs = now;
        logRowToSD();
    }

    // ── Network path ─────────────────────────────────────────────────
    if (modemReady) {
        // Re-check / re-establish connection
        if (!networkUp) {
            networkUp = tryConnectNetwork();
            if (networkUp && !registered) registerWithRetry();
        } else {
            ensureConnected();
            if (!registered) registerWithRetry();
        }

        // If we just got online, flush any offline SD rows first
        if (networkUp && registered) {
            flushOfflineRows();
        }

        // Normal 60-second server push
        if (networkUp && registered && (now - lastSendMs >= SEND_INTERVAL_MS)) {
            lastSendMs = now;
            bool ok = sendSensorData();
            if (ok) successCount++;
            else    failCount++;
            uint32_t upSec = (millis() - bootTime) / 1000;
            LOGF("[STATS] Uptime: %lum %lus | Sent: %lu ok, %lu fail\n",
                 upSec / 60, upSec % 60, successCount, failCount);
        }
    }

    esp_task_wdt_reset();
    delay(LOOP_DELAY_MS);
}

// ═════════════════════════════════════════════════════════════════════
//  PERIPHERAL INITIALISATION
// ═════════════════════════════════════════════════════════════════════

void initPeripherals()
{
    LOG("[INIT] Initializing peripherals...");

    // ── DHT temperature sensors ───────────────────────────────────────
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++) {
        dhtSensors[i].begin();
        LOGF("[INIT]   DHT%d on GPIO %d  (sensor #%d)\n", dhtTypes[i], dhtPins[i], i);
    }

    // ── INA219 on Wire (SDA 8, SCL 9) ────────────────────────────────
    Wire.begin(I2C_INA_SDA_PIN, I2C_INA_SCL_PIN);
    LOG("[I2C] Scanning INA bus (Wire):");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) LOGF("[I2C]   0x%02X\n", addr);
    }
    if (!ina219.begin(&Wire)) {
        LOG("[WARN] INA219 not found on SDA 8 / SCL 9");
    } else {
        ina219.setCalibration_32V_2A();
        LOG("[OK] INA219 ready");
    }

    // ── DS3231 on Wire1 (SDA 14, SCL 21) ─────────────────────────────
    Wire1.begin(I2C_RTC_SDA_PIN, I2C_RTC_SCL_PIN);
    LOG("[I2C] Scanning RTC bus (Wire1):");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) LOGF("[I2C]   0x%02X\n", addr);
    }
    if (!rtc.begin(&Wire1)) {
        LOG("[WARN] DS3231 not found — millis() fallback for Δt & timestamps");
        rtcAvailable = false;
    } else {
        rtcAvailable = true;
        if (rtc.lostPower()) {
            LOG("[RTC] Lost power — setting to compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        DateTime t   = rtc.now();
        rtcTimeValid = (t.year() >= 2024);
        LOGF("[OK] DS3231 ready — %04d-%02d-%02d %02d:%02d:%02d (valid: %s)\n",
             t.year(), t.month(), t.day(),
             t.hour(), t.minute(), t.second(),
             rtcTimeValid ? "yes" : "no");
    }

    // ── SD Card (SPI: MOSI 11, MISO 13, SCK 12, CS 10) ───────────────
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN)) {
        LOG("[WARN] SD card not found — local CSV logging disabled");
        sdReady = false;
    } else {
        sdReady = true;
        uint64_t cardMB = SD.cardSize() / (1024ULL * 1024ULL);
        LOGF("[OK] SD ready — Card size: %llu MB\n", cardMB);
        ensureCSVHeader();

        // Count existing rows so row numbering survives reboot
        File f = SD.open(LOG_FILE, FILE_READ);
        if (f) {
            while (f.available()) {
                if (f.read() == '\n') logRowCount++;
            }
            f.close();
            if (logRowCount > 0) logRowCount--;  // subtract header line
            LOGF("[SD] Existing log rows: %lu\n", logRowCount);
        }
    }

    LOG("[INIT] Peripherals ready\n");
}

// ═════════════════════════════════════════════════════════════════════
//  READ ALL SENSORS
// ═════════════════════════════════════════════════════════════════════

void readAllSensors()
{
    // ── DHT (temperature average) ─────────────────────────────────────
    float tempSum   = 0.0f;
    int   tempCount = 0;

    for (int i = 0; i < TEMP_SENSOR_COUNT; i++) {
        float t = dhtSensors[i].readTemperature();
        if (!isnan(t)) {
            tempSum += t;
            tempCount++;
        } else {
            LOGF("[SENSOR] DHT%d (GPIO %d) read failed — skipped\n", dhtTypes[i], dhtPins[i]);
        }
        if ((i & 1) == 1) esp_task_wdt_reset();
    }

    if (tempCount > 0) {
        sensorTemp = tempSum / (float)tempCount;
    } else {
        LOG("[SENSOR] ALL temp sensors failed — soft-reset DHT drivers");
        for (int i = 0; i < TEMP_SENSOR_COUNT; i++) dhtSensors[i].begin();
    }

    float h = dhtSensors[0].readHumidity();
    if (!isnan(h)) sensorHumid = h;

    esp_task_wdt_reset();

    // ── INA219 (voltage, current, power) ─────────────────────────────
    float shunt_mV   = ina219.getShuntVoltage_mV();
    float bus_V      = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float rawVoltage = bus_V + (shunt_mV / 1000.0f);

    // Voltage glitch filter
    if (rawVoltage >= 3.0f) {
        sensorVoltage    = rawVoltage;
        lastValidVoltage = rawVoltage;
    } else if (lastValidVoltage > 0.0f) {
        sensorVoltage = lastValidVoltage;
        LOG("[SENSOR] Voltage glitch — using last valid");
    } else {
        sensorVoltage = rawVoltage;
    }

    // Apply wiring direction: positive signedA = charging
    signedCurrentA = (current_mA / 1000.0f) * (float)ACS712_CHARGE_DIRECTION;
    sensorCurrent  = fabs(signedCurrentA);
    sensorPower    = ina219.getPower_mW() / 1000.0f;

    // ── SOC ───────────────────────────────────────────────────────────
    sensorSOC = calculateSOC(sensorVoltage, signedCurrentA);

    ChargeState state = currentToState(signedCurrentA);
    if (state != prevState) {
        LOGF("[STATUS] %s → %s\n", stateLabel(prevState), stateLabel(state));
        prevState = state;
    }

    LOGF("[SENSOR] T=%.1f°C H=%.1f%% V=%.2fV I=%+.3fA P=%.2fW SOC=%.1f%% [%s]\n",
         sensorTemp, sensorHumid, sensorVoltage,
         signedCurrentA, sensorPower, sensorSOC,
         stateLabel(state));

    if (rtcAvailable && rtcTimeValid) {
        DateTime t = rtc.now();
        LOGF("[RTC] %04d-%02d-%02d %02d:%02d:%02d\n",
             t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
    }
}

// ═════════════════════════════════════════════════════════════════════
//  SOC ENGINE  — identical logic to ChargeSDLogger.ino
// ═════════════════════════════════════════════════════════════════════

float voltageToSOC(float v, float currentA)
{
    const float IR_OHM = 0.25f;
    float restingV = v + fabs(currentA) * IR_OHM;

    // 3S Li-ion (18650) resting voltage → SOC lookup table
    static const float LUT_V[]   = { 9.00f,  9.90f, 10.65f, 10.95f, 11.10f,
                                    11.25f, 11.40f, 11.55f, 11.70f, 12.00f, 12.60f };
    static const float LUT_SOC[] = {  0.0f,  10.0f,  20.0f,  30.0f,  40.0f,
                                     50.0f,  60.0f,  70.0f,  80.0f,  90.0f, 100.0f };
    const int N = 11;

    if (restingV <= LUT_V[0])    return 0.0f;
    if (restingV >= LUT_V[N-1]) return 100.0f;
    for (int i = 1; i < N; i++) {
        if (restingV <= LUT_V[i]) {
            float ratio = (restingV - LUT_V[i-1]) / (LUT_V[i] - LUT_V[i-1]);
            return LUT_SOC[i-1] + ratio * (LUT_SOC[i] - LUT_SOC[i-1]);
        }
    }
    return 100.0f;
}

float calculateSOC(float rawVoltage, float signedCurrentA)
{
    float voltageSoc = voltageToSOC(rawVoltage, signedCurrentA);

    // ── Initial seed ─────────────────────────────────────────────────
    if (socEstimate < 0.0f) {
        if (rawVoltage < 3.0f) {
            LOG("[SOC] Waiting for valid battery voltage...");
            return 0.0f;
        }
        socEstimate = voltageSoc;
        if (rtcAvailable && rtcTimeValid)
            lastCoulombUnix = (uint32_t)rtc.now().unixtime();
        lastCoulombMs = millis();
        LOGF("[SOC] ★ Seeded from LUT: %.1f%%  (V=%.2f)\n", socEstimate, rawVoltage);
        return socEstimate;
    }

    // ── Elapsed time (RTC preferred, millis fallback) ─────────────────
    float elapsedHours = 0.0f;

    if (rtcAvailable && rtcTimeValid && lastCoulombUnix > 0) {
        uint32_t nowUnix  = (uint32_t)rtc.now().unixtime();
        uint32_t deltaSec = (nowUnix > lastCoulombUnix) ? (nowUnix - lastCoulombUnix) : 0;
        if (deltaSec > 180) deltaSec = 180;
        elapsedHours    = deltaSec / 3600.0f;
        lastCoulombUnix = nowUnix;
        LOGF("[SOC] Δt: DS3231 %lu s = %.5f h\n", deltaSec, elapsedHours);
    } else {
        uint32_t nowMs   = millis();
        uint32_t deltaMs = nowMs - lastCoulombMs;
        if (deltaMs > 180000) deltaMs = 180000;
        elapsedHours  = deltaMs / 3600000.0f;
        lastCoulombMs = nowMs;
        LOGF("[SOC] Δt: millis() %.5f h\n", elapsedHours);
    }

    // ── Coulomb integration ───────────────────────────────────────────
    float deltaSOC = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0f;
    socEstimate   += deltaSOC;
    socEstimate    = constrain(socEstimate, 0.0f, 100.0f);

    LOGF("[SOC] ΔSOC=%+.3f%%  Coulomb→%.1f%%  VLookup=%.1f%%\n",
         deltaSOC, socEstimate, voltageSoc);

    // ── Rest detection (5-minute confirmation window) ─────────────────
    uint32_t nowMs = millis();

    if (fabs(signedCurrentA) < SOC_IDLE_CURRENT_THRESH_A) {
        if (!isResting) {
            isResting   = true;
            restStartMs = nowMs;
            anchorDone  = false;
            LOG("[SOC] Rest timer started — waiting 5 min");
        } else if (!anchorDone && (nowMs - restStartMs >= 300000UL)) {
            float before = socEstimate;
            socEstimate  = (0.90f * socEstimate) + (0.10f * voltageSoc);
            anchorDone   = true;
            LOGF("[SOC] ★ 5min REST ANCHOR: %.1f%% → %.1f%%  (VLookup=%.1f%%)\n",
                 before, socEstimate, voltageSoc);
        } else if (!anchorDone) {
            uint32_t waited = (nowMs - restStartMs) / 1000;
            LOGF("[SOC] Resting %lus/300s — Coulomb only\n", waited);
        } else {
            LOG("[SOC] Idle — Coulomb only");
        }
    } else {
        if (isResting) {
            isResting  = false;
            anchorDone = false;
            LOG("[SOC] Load/charge detected — rest cancelled");
        }
        if (signedCurrentA < 0)
            LOGF("[SOC] Discharging %.3fA — Coulomb only\n", fabs(signedCurrentA));
        else
            LOGF("[SOC] Charging %.3fA — Coulomb only\n", signedCurrentA);
    }

    return socEstimate;
}

// ═════════════════════════════════════════════════════════════════════
//  NVS PERSISTENCE
// ═════════════════════════════════════════════════════════════════════

void saveSOCToNVS()
{
    if (socEstimate < 0.0f) return;
    prefs.begin("crux", false);
    prefs.putFloat("soc", socEstimate);
    if (rtcAvailable && rtcTimeValid)
        prefs.putUInt("ts", (uint32_t)rtc.now().unixtime());
    prefs.end();
}

void restoreSOCFromNVS()
{
    prefs.begin("crux", true);
    float    savedSOC = prefs.getFloat("soc", -1.0f);
    uint32_t savedTS  = prefs.getUInt("ts",   0);
    prefs.end();

    if (savedSOC < 0.0f || savedSOC > 100.0f) {
        LOG("[NVS] No saved SOC — will seed from voltage on first read");
        return;
    }

    float shunt_mV    = ina219.getShuntVoltage_mV();
    float bus_V       = ina219.getBusVoltage_V();
    float liveVoltage = bus_V + (shunt_mV / 1000.0f);

    if (liveVoltage >= 3.0f) {
        float liveSOC = voltageToSOC(liveVoltage, 0.0f);
        float diff    = fabs(savedSOC - liveSOC);
        LOGF("[NVS] Saved=%.1f%%  LiveV=%.2fV  VLookup=%.1f%%  Diff=%.1f%%\n",
             savedSOC, liveVoltage, liveSOC, diff);
        if (diff > SOC_REBOOT_MAX_DEVIATION) {
            LOGF("[NVS] Deviation %.1f%% > %.1f%% — discarding saved SOC\n",
                 diff, SOC_REBOOT_MAX_DEVIATION);
            return;
        }
    }

    float restored = savedSOC;

    if (rtcAvailable && rtcTimeValid && savedTS > 0) {
        uint32_t nowUnix = (uint32_t)rtc.now().unixtime();
        uint32_t offSec  = (nowUnix > savedTS) ? (nowUnix - savedTS) : 0;
        if (offSec > NVS_MAX_RESTORE_SECONDS) {
            LOGF("[NVS] Off %lu s > %lu s limit — re-seeding from voltage\n",
                 offSec, NVS_MAX_RESTORE_SECONDS);
            return;
        }
        float offHours  = offSec / 3600.0f;
        float selfDisch = offHours * SELF_DISCHARGE_PCT_PER_HOUR;
        restored       -= selfDisch;
        restored        = constrain(restored, 0.0f, 100.0f);
        LOGF("[NVS] Off %.2fh → self-disch -%.2f%% → restored %.1f%%\n",
             offHours, selfDisch, restored);
    } else {
        LOGF("[NVS] Restored SOC=%.1f%% (no RTC timestamp)\n", restored);
    }

    socEstimate     = restored;
    lastCoulombUnix = savedTS;
    lastCoulombMs   = millis();
}

// ═════════════════════════════════════════════════════════════════════
//  SD CARD HELPERS
// ═════════════════════════════════════════════════════════════════════

// Returns "YYYY-MM-DD HH:MM:SS" or "T+NNNNs"
String getTimestamp()
{
    if (rtcAvailable && rtcTimeValid) {
        DateTime t = rtc.now();
        char buf[24];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                 t.year(), t.month(), t.day(),
                 t.hour(), t.minute(), t.second());
        return String(buf);
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "T+%lus", millis() / 1000);
    return String(buf);
}

void ensureCSVHeader()
{
    File f = SD.open(LOG_FILE, FILE_READ);
    bool hasContent = f && (f.size() > 0);
    if (f) f.close();
    if (hasContent) return;

    File w = SD.open(LOG_FILE, FILE_WRITE);
    if (w) {
        w.println("row,timestamp,temperature_C,humidity_pct,voltage_V,"
                  "current_A,power_W,soc_pct,state,synced");
        w.close();
        LOG("[SD] CSV header written");
    } else {
        LOG("[SD] ERROR: Could not write CSV header");
    }
}

void logRowToSD()
{
    if (!sdReady) {
        LOG("[SD] Not ready — skipping local log");
        return;
    }

    logRowCount++;
    String ts       = getTimestamp();
    ChargeState st  = currentToState(signedCurrentA);

    File f = SD.open(LOG_FILE, FILE_APPEND);
    if (!f) {
        LOGF("[SD] ✗ FAILED to open log file for row %lu\n", logRowCount);
        logRowCount--;
        return;
    }

    // row,timestamp,temperature_C,humidity_pct,voltage_V,
    // current_A,power_W,soc_pct,state,synced
    f.print(logRowCount);           f.print(",");
    f.print(ts);                    f.print(",");
    f.print(sensorTemp,  1);        f.print(",");
    f.print(sensorHumid, 1);        f.print(",");
    f.print(sensorVoltage, 3);      f.print(",");
    f.print(signedCurrentA, 4);     f.print(",");
    f.print(sensorPower,   3);      f.print(",");
    f.print(sensorSOC,     1);      f.print(",");
    f.print(stateLabel(st));        f.print(",");
    f.println("0");   // synced = 0 (not yet sent)
    f.close();

    LOGF("[SD] ✓ Row %lu saved  @ %s  [offline]\n", logRowCount, ts.c_str());
}

// ─────────────────────────────────────────────────────────────────────
//  Mark a row as synced by rewriting its 'synced' column to '1'.
//  SD cards don't support random-byte writes, so we rewrite the whole
//  file.  For large logs a proper append-only approach is to write a
//  separate "synced.txt" index file; here we use the simpler full-
//  rewrite strategy since offline windows are expected to be short.
// ─────────────────────────────────────────────────────────────────────
void markRowsSynced(uint32_t upToRow)
{
    // Read entire file into memory
    File src = SD.open(LOG_FILE, FILE_READ);
    if (!src) return;

    // Build updated content in a temporary String (RAM permitting)
    // For very large files, use a temp file approach instead.
    String updated = "";
    uint32_t lineNum = 0;  // 0 = header

    while (src.available()) {
        String line = src.readStringUntil('\n');
        line.trim();

        if (lineNum == 0 || line.length() == 0) {
            // Keep header / empty lines unchanged
            updated += line + "\n";
        } else {
            // Find last comma and check synced column
            int lastComma = line.lastIndexOf(',');
            if (lastComma >= 0) {
                String rowNumStr = line.substring(0, line.indexOf(','));
                uint32_t rowN = (uint32_t)rowNumStr.toInt();
                if (rowN <= upToRow) {
                    // Replace synced column
                    updated += line.substring(0, lastComma + 1) + "1\n";
                } else {
                    updated += line + "\n";
                }
            } else {
                updated += line + "\n";
            }
        }
        lineNum++;
    }
    src.close();

    // Rewrite
    SD.remove(LOG_FILE);
    File dst = SD.open(LOG_FILE, FILE_WRITE);
    if (dst) {
        dst.print(updated);
        dst.close();
        LOGF("[SD] Marked rows 1-%lu as synced\n", upToRow);
    }
}

// ═════════════════════════════════════════════════════════════════════
//  OFFLINE FLUSH  — send all unsynced SD rows to server on reconnect
// ═════════════════════════════════════════════════════════════════════

void flushOfflineRows()
{
    if (!sdReady) return;

    File f = SD.open(LOG_FILE, FILE_READ);
    if (!f) return;

    bool       headerSkipped = false;
    uint32_t   lastSyncedRow = 0;
    bool       anySent       = false;

    LOG("[FLUSH] Scanning SD for unsynced rows...");

    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        if (!headerSkipped) { headerSkipped = true; continue; }

        // Parse CSV: row,timestamp,temp,humid,volt,curr,pwr,soc,state,synced
        // Quick split by counting commas
        int fields[10];  // positions of commas
        int fc = 0;
        for (int i = 0; i < (int)line.length() && fc < 9; i++) {
            if (line.charAt(i) == ',') fields[fc++] = i;
        }
        if (fc < 9) continue;

        // Extract synced flag (last field)
        String syncedStr = line.substring(fields[8] + 1);
        syncedStr.trim();
        if (syncedStr == "1") continue;  // already sent

        // Extract fields
        String rowStr   = line.substring(0, fields[0]);
        String ts       = line.substring(fields[0]+1, fields[1]);
        String tempStr  = line.substring(fields[1]+1, fields[2]);
        String humStr   = line.substring(fields[2]+1, fields[3]);
        String voltStr  = line.substring(fields[3]+1, fields[4]);
        String currStr  = line.substring(fields[4]+1, fields[5]);
        String pwrStr   = line.substring(fields[5]+1, fields[6]);
        String socStr   = line.substring(fields[6]+1, fields[7]);
        String stateStr = line.substring(fields[7]+1, fields[8]);

        uint32_t rowN = (uint32_t)rowStr.toInt();

        // Build JSON payload (same structure as live sendSensorData)
#if CRUX_USE_JSONDOC_V6
        StaticJsonDocument<JSON_DOC_CAPACITY> doc;
#else
        JsonDocument doc;
#endif
        doc["temperature"] = round1(tempStr.toFloat());
        doc["voltage"]     = round2(voltStr.toFloat());
        doc["power"]       = round2(pwrStr.toFloat());
        doc["current"]     = round3(fabs(currStr.toFloat()));
        doc["soc"]         = round1(socStr.toFloat());
        doc["timestamp"]   = ts;          // historical timestamp
        doc["offline"]     = true;        // flag so server knows it's backfill

        char   jsonBuf[JSON_SERIAL_BUFFER];
        size_t jsonLen = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
        if (jsonLen == 0 || jsonLen >= sizeof(jsonBuf)) continue;

        LOGF("[FLUSH] Sending offline row %lu  @ %s\n", rowN, ts.c_str());

        int code = httpPost(DATA_PATH, jsonBuf, jsonLen, DEVICE_ID, DEVICE_SECRET);
        if (code == 200 || code == 201) {
            lastSyncedRow = rowN;
            anySent = true;
            LOGF("[FLUSH] ✓ Row %lu accepted (HTTP %d)\n", rowN, code);
        } else {
            LOGF("[FLUSH] ✗ Row %lu rejected (HTTP %d) — stopping flush\n", rowN, code);
            break;  // stop on first failure; retry next connection cycle
        }

        esp_task_wdt_reset();
    }

    f.close();

    if (anySent) {
        markRowsSynced(lastSyncedRow);
    } else {
        LOG("[FLUSH] No new rows to flush");
    }
}

// ═════════════════════════════════════════════════════════════════════
//  MODEM  (from v4 — power sequence + init + network)
// ═════════════════════════════════════════════════════════════════════

bool tryInitModem()
{
    LOG("[MODEM] Powering on EC200U...");
    pinMode(MODEM_POWER, OUTPUT);
    digitalWrite(MODEM_POWER, LOW);  delay(100);
    digitalWrite(MODEM_POWER, HIGH); delay(1000);
    digitalWrite(MODEM_POWER, LOW);
    LOG("[MODEM] Power sequence done — waiting 5s for boot");
    delay(5000);

    SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    LOG("[MODEM] Initializing...");

    if (!modem.init()) {
        LOG("[MODEM] init() failed — trying restart()");
        if (!modem.restart()) {
            LOG("[MODEM] restart() also failed — modem unavailable");
            return false;
        }
    }

    String name = modem.getModemName();
    String info = modem.getModemInfo();
    LOGF("[MODEM] Name: %s\n", name.c_str());
    LOGF("[MODEM] Info: %s\n", info.c_str());

#if USE_HTTPS
    // Hand the self-signed certificate to the TLS client so the modem
    // can verify the nginx reverse proxy during the handshake.
    netClient.setCACert(SERVER_CERT);
    LOG("[MODEM] TLS: CA cert loaded (nginx self-signed, via modem hardware stack)");
#endif

    LOG("[MODEM] Ready");
    return true;
}

bool tryConnectNetwork()
{
    LOG("[NET] Scanning for 4G network...");
    if (!modem.waitForNetwork(NETWORK_TIMEOUT_MS, true)) {
        LOG("[NET] No network found — staying offline");
        return false;
    }
    LOGF("[NET] Registered (signal: %d/31)\n", modem.getSignalQuality());

    LOG("[NET] Connecting GPRS...");
    if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
        delay(5000);
        if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
            LOG("[NET] GPRS connect failed — staying offline");
            return false;
        }
    }
    LOG("[NET] GPRS connected — IP: " + modem.getLocalIP());
    return true;
}

void ensureConnected()
{
    if (!modem.isNetworkConnected()) {
        LOG("[NET] Network lost — reconnecting");
        networkUp = tryConnectNetwork();
    } else if (!modem.isGprsConnected()) {
        LOG("[NET] GPRS lost — reconnecting");
        if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) networkUp = false;
    }
}

// ═════════════════════════════════════════════════════════════════════
//  HTTP  — registration + live data push (v4 style: stack headers, no
//          Arduino String allocations in the hot path)
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

    char   jsonBuf[JSON_SERIAL_BUFFER];
    size_t jsonLen = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (jsonLen == 0 || jsonLen >= sizeof(jsonBuf)) {
        LOG("[REG] JSON serialize error");
        return;
    }

    for (int i = 1; i <= 5; i++) {
        int code = httpPost(REGISTER_PATH, jsonBuf, jsonLen, nullptr, nullptr);
        if (code == 200 || code == 201) {
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
    LOG("[DATA] Sending live data...");

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

    char   jsonBuf[JSON_SERIAL_BUFFER];
    size_t jsonLen = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (jsonLen == 0 || jsonLen >= sizeof(jsonBuf)) {
        LOG("[DATA] JSON serialize error");
        return false;
    }

    LOGF("[DATA] Payload: %s\n", jsonBuf);

    int code = httpPost(DATA_PATH, jsonBuf, jsonLen, DEVICE_ID, DEVICE_SECRET);
    if (code == 200 || code == 201) {
        LOGF("[DATA] ✓ HTTP %d\n\n", code);
        return true;
    }
    LOGF("[DATA] ✗ HTTP %d\n", code);
    if (code == 404) registered = false;
    return false;
}

// ─────────────────────────────────────────────────────────────────────
//  Core HTTP POST  — v4 stack-based header build, 3-retry with backoff
//  Works with HTTPS via nginx reverse proxy (modem TLS termination)
// ─────────────────────────────────────────────────────────────────────
static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret)
{
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        if (attempt > 1) {
            LOGF("[HTTP] Retry %d/%d\n", attempt, MAX_RETRIES);
            delay(RETRY_BACKOFF_MS * attempt);
        }

        LOGF("[HTTP] Connect %s:%d\n", SERVER_HOST, SERVER_PORT);
        if (!netClient.connect(SERVER_HOST, SERVER_PORT)) {
            LOG("[HTTP] TCP connect failed");
            netClient.stop();
            continue;
        }

        // ── Build request headers on stack ────────────────────────────
        char hdrs[HTTP_HEADER_BUFFER];
        int  h = snprintf(
            hdrs, sizeof(hdrs),
            "POST %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %u\r\n"
            "Connection: close\r\n",
            path, SERVER_HOST, (unsigned)bodyLen);

        if (h <= 0 || (size_t)h >= sizeof(hdrs) - 96) {
            LOG("[HTTP] header base truncated");
            netClient.stop();
            continue;
        }

        int pos = h;
        if (devId && devSecret) {
            int a = snprintf(
                hdrs + pos, sizeof(hdrs) - (size_t)pos,
                "x-device-id: %s\r\nx-device-secret: %s\r\n",
                devId, devSecret);
            if (a <= 0 || pos + a >= (int)sizeof(hdrs) - 8) {
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

        // ── Wait for response ─────────────────────────────────────────
        uint32_t t0 = millis();
        while (!netClient.available() && millis() - t0 < HTTP_TIMEOUT_MS) delay(50);
        if (!netClient.available()) {
            LOG("[HTTP] timeout waiting for response");
            netClient.stop();
            continue;
        }

        // ── Parse status code ─────────────────────────────────────────
        String statusLine = netClient.readStringUntil('\n');
        int statusCode = -1;
        if (statusLine.length() > 12)
            statusCode = statusLine.substring(9, 12).toInt();
        LOGF("[HTTP] ← %d\n", statusCode);

        // Skip response headers
        while (netClient.available()) {
            String line = netClient.readStringUntil('\n');
            if (line.length() <= 1) break;
        }

        // Drain body (log snippet)
        char   respBuf[HTTP_RESP_BODY_MAX];
        size_t respN = 0;
        uint32_t readStart = millis();
        while (netClient.available() && millis() - readStart < 3000
               && respN < sizeof(respBuf) - 1) {
            respBuf[respN++] = (char)netClient.read();
        }
        respBuf[respN] = '\0';
        if (respN > 0) LOGF("[HTTP] Body: %s\n", respBuf);

        netClient.stop();

        if (statusCode >= 200 && statusCode < 300) return statusCode;
        if (statusCode >= 400 && statusCode < 500
            && statusCode != 408 && statusCode != 429)
            return statusCode;
    }

    LOG("[HTTP] All retries failed");
    return -1;
}

// ═════════════════════════════════════════════════════════════════════
//  HELPERS
// ═════════════════════════════════════════════════════════════════════
float round1(float v) { return ((int)(v * 10   + 0.5f)) / 10.0f;   }
float round2(float v) { return ((int)(v * 100  + 0.5f)) / 100.0f;  }
float round3(float v) { return ((int)(v * 1000 + 0.5f)) / 1000.0f; }
