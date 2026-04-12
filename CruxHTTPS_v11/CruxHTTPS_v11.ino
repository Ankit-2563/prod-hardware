// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS_v11.ino — Crux IoT · v11
// ─────────────────────────────────────────────────────────────────────
//  Combined firmware:
//    - Temperature sensors from v4 (3x DHT22, 1x DHT11)
//    - Server registration + data sending from v4 (1 req/min)
//    - SOC engine from v9 (RTC Coulomb counting + NVS + rest-detection)
//    - Two separate I2C buses (INA219 on Wire, DS3231 on Wire1)
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

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>


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
float sensorCurrent = 0.0;
float sensorVoltage = 0.0;
float sensorPower   = 0.0;
float sensorSOC     = 0.0;

// SOC engine state
float    socEstimate      = -1.0;
uint32_t lastCoulombUnix  = 0;
uint32_t lastCoulombMs    = 0;
float    lastValidVoltage = 0.0;

bool rtcAvailable = false;
bool rtcTimeValid = false;

// Rest-detection state for idle correction
uint32_t restStartMs = 0;
bool     isResting   = false;
bool     anchorDone  = false;

// ── SD state ─────────────────────────────────────────────────────
bool     sdAvailable       = false;
uint32_t chargeCycles      = 0;
bool     cycleInProgress   = false;   // true once SOC dropped below LOW mark

// Forward declaration
static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret);

// ═════════════════════════════════════════════════════════════════════
//  SD CARD LOGIC
// ═════════════════════════════════════════════════════════════════════

void initSD()
{
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN))
    {
        LOG("[SD] Not found — offline buffering disabled");
        sdAvailable = false;
        return;
    }
    sdAvailable = true;
    LOG("[SD] Ready");

    if (SD.exists(SD_CYCLE_FILE))
    {
        File f = SD.open(SD_CYCLE_FILE, FILE_READ);
        if (f)
        {
            chargeCycles = f.parseInt();
            f.close();
            LOGF("[SD] Restored charge cycles: %lu\n", chargeCycles);
        }
    }
}

void bufferToDisk()
{
    if (!sdAvailable) return;

    File f = SD.open(SD_DATA_FILE, FILE_APPEND);
    if (!f)
    {
        LOG("[SD] Failed to open buffer file");
        return;
    }

    uint32_t ts = rtcAvailable && rtcTimeValid ? (uint32_t)rtc.now().unixtime() : 0;
    f.printf("%lu,%.1f,%.2f,%.2f,%.3f,%.1f\n",
             ts, sensorTemp, sensorVoltage, sensorPower, sensorCurrent, sensorSOC);
    f.close();
    LOGF("[SD] Buffered to disk (ts=%lu)\n", ts);
}

void flushSDBuffer()
{
    if (!sdAvailable || !registered) return;
    if (!SD.exists(SD_DATA_FILE))    return;

    File f = SD.open(SD_DATA_FILE, FILE_READ);
    if (!f || f.size() == 0) { if (f) f.close(); return; }

    LOG("[SD] Flushing offline buffer...");

    String lines[SD_FLUSH_BATCH_SIZE];
    int count = 0;

    while (f.available() && count < SD_FLUSH_BATCH_SIZE)
        lines[count++] = f.readStringUntil('\n');

    String remainder = "";
    while (f.available())
        remainder += f.readStringUntil('\n') + "\n";
    f.close();

    int sent = 0;
    for (int i = 0; i < count; i++)
    {
        float t, v, p, c, s;
        uint32_t ts;
        int parsed = sscanf(lines[i].c_str(), "%lu,%f,%f,%f,%f,%f", &ts, &t, &v, &p, &c, &s);
        if (parsed < 6) continue;

        char body[192];
        snprintf(body, sizeof(body),
                 "{\"temperature\":%.1f,\"voltage\":%.2f,\"power\":%.2f,\"current\":%.3f,\"soc\":%.1f,\"recordedAt\":\"%lu\"}",
                 t, v, p, c, s, ts);

        int code = httpPost(DATA_PATH, body, strlen(body), DEVICE_ID, DEVICE_SECRET);
        if (code == 201) sent++;
        else break;
    }

    LOGF("[SD] Flushed %d/%d records\n", sent, count);
    SD.remove(SD_DATA_FILE);

    if (count - sent > 0 || remainder.length() > 0)
    {
        File fw = SD.open(SD_DATA_FILE, FILE_WRITE);
        if (fw)
        {
            for (int i = sent; i < count; i++)
                fw.println(lines[i]);
            fw.print(remainder);
            fw.close();
        }
    }
}

void updateChargeCycles(float soc)
{
    if (!sdAvailable) return;

    if (!cycleInProgress && soc <= CHARGE_CYCLE_SOC_LOW)
    {
        cycleInProgress = true;
        LOG("[CYCLE] SOC dropped below 20% — watching for full recharge");
    }

    if (cycleInProgress && soc >= CHARGE_CYCLE_SOC_HIGH)
    {
        cycleInProgress = false;
        chargeCycles++;
        LOGF("[CYCLE] ★ Charge cycle complete — total: %lu\n", chargeCycles);

        SD.remove(SD_CYCLE_FILE);
        File f = SD.open(SD_CYCLE_FILE, FILE_WRITE);
        if (f) { f.print(chargeCycles); f.close(); }
    }
}

// ═════════════════════════════════════════════════════════════════════
//  SETUP
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
    LOG("═══════════════════════════════════════════════\n");

    initSensors();
    initSD();
    restoreSOCFromNVS();

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

// ═════════════════════════════════════════════════════════════════════
//  LOOP — read sensors every second, send data every 60 seconds
// ═════════════════════════════════════════════════════════════════════

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
        saveSOCToNVS();

        bool ok = sendSensorData();
        if (ok)
        {
            successCount++;
            flushSDBuffer();
        }
        else
        {
            failCount++;
            bufferToDisk();
        }

        updateChargeCycles(sensorSOC);

        uint32_t upSec = (millis() - bootTime) / 1000;
        LOGF("[STATS] Uptime: %lum %lus | Sent: %lu ok, %lu fail\n",
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

    // DHT temperature sensors
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
        dhtSensors[i].begin();
        LOGF("[SENSOR]   DHT%d on GPIO %d  (sensor #%d)\n", dhtTypes[i], dhtPins[i], i);
    }

    // INA219 on Wire (Bus 1)
    Wire.begin(I2C_INA_SDA_PIN, I2C_INA_SCL_PIN);

    LOG("[I2C] Scanning INA Bus (Wire):");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) LOGF("[I2C]   0x%02X\n", addr);
    }

    if (!ina219.begin(&Wire))
    {
        LOG("[SENSOR] WARNING: INA219 not found on SDA 8 / SCL 9");
    }
    else
    {
        ina219.setCalibration_32V_2A();
        LOG("[SENSOR] INA219 ready");
    }

    // DS3231 on Wire1 (Bus 2)
    Wire1.begin(I2C_RTC_SDA_PIN, I2C_RTC_SCL_PIN);

    LOG("[I2C] Scanning RTC Bus (Wire1):");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) LOGF("[I2C]   0x%02X\n", addr);
    }

    if (!rtc.begin(&Wire1))
    {
        LOG("[RTC] DS3231 not found on SDA 14 / SCL 21 — Δt will use millis()");
        rtcAvailable = false;
    }
    else
    {
        rtcAvailable = true;

        if (rtc.lostPower())
        {
            LOG("[RTC] Lost power — setting to compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }

        DateTime t = rtc.now();
        rtcTimeValid = (t.year() >= 2024);

        char buf[32];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                 t.year(), t.month(), t.day(),
                 t.hour(), t.minute(), t.second());
        LOGF("[RTC] DS3231 ready — %s (valid: %s)\n",
             buf, rtcTimeValid ? "yes" : "no");
    }

    LOG("[SENSOR] Ready\n");
}

// ═════════════════════════════════════════════════════════════════════
//  READ ALL SENSORS
// ═════════════════════════════════════════════════════════════════════

void readAllSensors()
{
    LOG("[SENSOR] ── Reading ──");

    // ── Temperature (DHT average) ────────────────────────────────────
    float tempSum  = 0.0;
    int   tempCount = 0;

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

    // Humidity from sensor 0
    float h = dhtSensors[0].readHumidity();
    if (!isnan(h))
        sensorHumid = h;
    else
        LOG("[SENSOR]   Humidity read failed, using previous value");

    esp_task_wdt_reset();

    // ── INA219 (voltage, current, power) ─────────────────────────────
    float shunt_mV   = ina219.getShuntVoltage_mV();
    float bus_V      = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float rawVoltage = bus_V + (shunt_mV / 1000.0);

    if (rawVoltage >= 3.0)
    {
        sensorVoltage    = rawVoltage;
        lastValidVoltage = rawVoltage;
    }
    else if (lastValidVoltage > 0.0)
    {
        sensorVoltage = lastValidVoltage;
        LOG("[SENSOR] Voltage glitch — using last valid");
    }
    else
    {
        sensorVoltage = rawVoltage;
    }

    float signedCurrent_A = (current_mA / 1000.0) * (float)ACS712_CHARGE_DIRECTION;
    sensorCurrent = fabs(signedCurrent_A);
    sensorPower   = ina219.getPower_mW() / 1000.0;

    // ── SOC ──────────────────────────────────────────────────────────
    sensorSOC = calculateSOC(sensorVoltage, signedCurrent_A);

    // ── Print everything ─────────────────────────────────────────────
    LOGF("[SENSOR]   Temperature: %.1f °C\n", sensorTemp);
    LOGF("[SENSOR]   Humidity   : %.1f %%\n", sensorHumid);
    LOGF("[SENSOR]   Voltage    : %.2f V\n",  sensorVoltage);
    LOGF("[SENSOR]   Current    : %.3f A (%s)\n",
         sensorCurrent, signedCurrent_A >= 0 ? "charging" : "discharging");
    LOGF("[SENSOR]   Power      : %.2f W\n",  sensorPower);
    LOGF("[SENSOR]   SOC        : %.1f %%\n", sensorSOC);

    if (rtcAvailable && rtcTimeValid)
    {
        DateTime t = rtc.now();
        LOGF("[SENSOR]   RTC Time   : %04d-%02d-%02d %02d:%02d:%02d\n",
             t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
    }

    LOG("[SENSOR] ────────────\n");
}

// ═════════════════════════════════════════════════════════════════════
//  SOC ENGINE (from v9 — RTC Coulomb counting + rest-detection)
// ═════════════════════════════════════════════════════════════════════

float voltageToSOC(float v, float currentA)
{
    // IR compensation: estimate resting voltage from loaded reading
    const float INTERNAL_RESISTANCE_OHM = 0.25f;
    float restingV = v + fabs(currentA) * INTERNAL_RESISTANCE_OHM;

    // 3S Li-ion (18650) Discharge Voltage Map (Resting)
    static const float LUT_V[]   = { 9.00,  9.90, 10.65, 10.95, 11.10,
                                    11.25, 11.40, 11.55, 11.70, 12.00, 12.60};
    static const float LUT_SOC[] = {  0.0,  10.0,  20.0,  30.0,  40.0,
                                     50.0,  60.0,  70.0,  80.0,  90.0, 100.0};
    const int N = 11;

    if (restingV <= LUT_V[0])     return 0.0;
    if (restingV >= LUT_V[N - 1]) return 100.0;
    for (int i = 1; i < N; i++)
    {
        if (restingV <= LUT_V[i])
        {
            float ratio = (restingV - LUT_V[i - 1]) / (LUT_V[i] - LUT_V[i - 1]);
            return LUT_SOC[i - 1] + ratio * (LUT_SOC[i] - LUT_SOC[i - 1]);
        }
    }
    return 100.0;
}

float calculateSOC(float rawVoltage, float signedCurrentA)
{
    // IR-compensated voltage → SOC
    float voltageSoc = voltageToSOC(rawVoltage, signedCurrentA);

    // Initial seed from LUT
    if (socEstimate < 0.0)
    {
        if (rawVoltage < 3.0)
        {
            LOG("[SOC] Waiting for valid battery voltage...");
            return 0.0;
        }
        socEstimate = voltageSoc;

        if (rtcAvailable && rtcTimeValid)
            lastCoulombUnix = (uint32_t)rtc.now().unixtime();
        lastCoulombMs = millis();

        LOGF("[SOC] ★ Seeded from voltage LUT: %.1f%%  (V=%.2f)\n",
             socEstimate, rawVoltage);
        return socEstimate;
    }

    // Time elapsed — prefer RTC, fall back to millis()
    float elapsedHours = 0.0;

    if (rtcAvailable && rtcTimeValid && lastCoulombUnix > 0)
    {
        uint32_t nowUnix  = (uint32_t)rtc.now().unixtime();
        uint32_t deltaSec = (nowUnix > lastCoulombUnix) ? (nowUnix - lastCoulombUnix) : 0;

        uint32_t maxSec = 180; // Sanity cap
        if (deltaSec > maxSec) deltaSec = maxSec;

        elapsedHours    = deltaSec / 3600.0;
        lastCoulombUnix = nowUnix;
        LOGF("[SOC] Δt: DS3231  %lu s = %.5f h\n", deltaSec, elapsedHours);
    }
    else
    {
        uint32_t nowMs   = millis();
        uint32_t deltaMs = nowMs - lastCoulombMs;
        uint32_t maxMs   = 180000;
        if (deltaMs > maxMs) deltaMs = maxMs;

        elapsedHours  = deltaMs / 3600000.0;
        lastCoulombMs = nowMs;
        LOGF("[SOC] Δt: millis()  %.5f h\n", elapsedHours);
    }

    // Coulomb integration
    float deltaSOC  = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0;
    socEstimate    += deltaSOC;
    socEstimate     = constrain(socEstimate, 0.0, 100.0);

    LOGF("[SOC] V=%.2f I=%+.3fA\n", rawVoltage, signedCurrentA);
    LOGF("[SOC] ΔSOC=%+.3f%%  Coulomb→%.1f%%  VLookup=%.1f%%\n",
         deltaSOC, socEstimate, voltageSoc);

    // ── Rest detection with 5-minute confirmation ────────────────────
    uint32_t nowMs = millis();

    if (fabs(signedCurrentA) < SOC_IDLE_CURRENT_THRESH_A)
    {
        // Current below 10mA — possible rest
        if (!isResting)
        {
            isResting   = true;
            restStartMs = nowMs;
            anchorDone  = false;
            LOG("[SOC] Rest timer started — waiting 5 min");
        }
        else if (!anchorDone && (nowMs - restStartMs >= 300000UL))
        {
            // 5 minutes confirmed rest — voltage is stable, safe to anchor
            float before = socEstimate;
            socEstimate  = (0.90f * socEstimate) + (0.10f * voltageSoc);
            anchorDone   = true;
            LOGF("[SOC] ★ 5min REST ANCHOR: %.1f%% → %.1f%%"
                 " (VLookup=%.1f%%)\n",
                 before, socEstimate, voltageSoc);
        }
        else if (!anchorDone)
        {
            uint32_t waited = (nowMs - restStartMs) / 1000;
            LOGF("[SOC] Resting %lus/300s — Coulomb only\n", waited);
        }
        else
        {
            LOG("[SOC] Idle — Coulomb only");
        }
    }
    else
    {
        // Active current — discharge or charge
        if (isResting)
        {
            isResting  = false;
            anchorDone = false;
            LOG("[SOC] Load detected — rest cancelled");
        }

        if (signedCurrentA < 0)
            LOGF("[SOC] Discharging %.3fA — Coulomb only\n",
                 fabs(signedCurrentA));
        else
            LOGF("[SOC] Charging %.3fA — Coulomb only\n",
                 signedCurrentA);
    }

    return socEstimate;
}

// ═════════════════════════════════════════════════════════════════════
//  NVS PERSISTENCE (from v9 — save/restore SOC across reboots)
// ═════════════════════════════════════════════════════════════════════

void saveSOCToNVS()
{
    if (socEstimate < 0.0) return;

    prefs.begin("crux", false);
    prefs.putFloat("soc", socEstimate);
    if (rtcAvailable && rtcTimeValid)
        prefs.putUInt("ts", (uint32_t)rtc.now().unixtime());
    prefs.end();
}

void restoreSOCFromNVS()
{
    prefs.begin("crux", true);
    float    savedSOC = prefs.getFloat("soc", -1.0);
    uint32_t savedTS  = prefs.getUInt("ts",    0);
    prefs.end();

    if (savedSOC < 0.0 || savedSOC > 100.0)
    {
        LOG("[NVS] No saved SOC — will seed from voltage on first read");
        return;
    }

    float shunt_mV    = ina219.getShuntVoltage_mV();
    float bus_V       = ina219.getBusVoltage_V();
    float liveVoltage = bus_V + (shunt_mV / 1000.0);

    if (liveVoltage >= 3.0)
    {
        float liveSOC = voltageToSOC(liveVoltage, 0.0f);
        float diff    = fabs(savedSOC - liveSOC);
        LOGF("[NVS] Saved=%.1f%%  LiveV=%.2fV  VLookup=%.1f%%  Diff=%.1f%%\n",
             savedSOC, liveVoltage, liveSOC, diff);

        if (diff > SOC_REBOOT_MAX_DEVIATION)
        {
            LOGF("[NVS] Deviation %.1f%% > %.1f%% limit — discarding saved SOC\n",
                 diff, SOC_REBOOT_MAX_DEVIATION);
            return;
        }
    }

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
        LOGF("[NVS] Restored SOC=%.1f%% (no RTC timestamp for off-time)\n", restored);
    }

    socEstimate     = restored;
    lastCoulombUnix = savedTS;
    lastCoulombMs   = millis();
}

// ═════════════════════════════════════════════════════════════════════
//  MODEM (from v4)
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
//  HTTP — registration + data sending (from v4)
// ═════════════════════════════════════════════════════════════════════

void registerWithRetry()
{
    LOG("[REG] Registering device...");

#if CRUX_USE_JSONDOC_V6
    StaticJsonDocument<JSON_DOC_CAPACITY> doc;
#else
    JsonDocument doc;
#endif
    doc["deviceId"]          = DEVICE_ID;
    doc["deviceSecret"]      = DEVICE_SECRET;
    doc["deviceName"]        = DEVICE_NAME;
    doc["firmwareVersion"]   = FIRMWARE_VER;

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

        char   respBuf[HTTP_RESP_BODY_MAX];
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
//  HELPERS
// ═════════════════════════════════════════════════════════════════════
float round1(float v) { return ((int)(v * 10 + 0.5f)) / 10.0f; }
float round2(float v) { return ((int)(v * 100 + 0.5f)) / 100.0f; }
float round3(float v) { return ((int)(v * 1000 + 0.5f)) / 1000.0f; }
