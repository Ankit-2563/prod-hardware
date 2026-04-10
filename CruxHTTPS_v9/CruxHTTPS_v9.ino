// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS_v9.ino — Crux IoT · v9
// ─────────────────────────────────────────────────────────────────────
//  Logic of v4 (Mock environment for temperature) and v8 (SOC via Flash and RTC)
//  - Uses INA219 & DS3231
//  - NO SERVER DATA SENT, just serial monitor
//  - Two separate I2C Buses
// ═══════════════════════════════════════════════════════════════════════

#include "config.h"
#include <esp_task_wdt.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <Preferences.h>

Adafruit_INA219 ina219;
RTC_DS3231      rtc;
Preferences     prefs;

// ═════════════════════════════════════════════════════════════════════
//  GLOBALS
// ═════════════════════════════════════════════════════════════════════

uint32_t lastPrintMs  = 0;
uint32_t bootTime     = 0;

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

// ═════════════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ═════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(200);

    LOG("═══════════════════════════════════════════════");
    LOG("  Crux IoT · Firmware v9 (Standalone Edition)");
    LOG("  - No GSM/HTTP logic");
    LOG("  - Outputting to Serial Monitor only");
    LOG("  - Uses RTC for exact Coulomb counting");
    LOG("═══════════════════════════════════════════════\n");

    initSensors();
    restoreSOCFromNVS();

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
    uint32_t now = millis();

    // Print to serial monitor exactly once per second
    if (now - lastPrintMs >= LOOP_DELAY_MS)
    {
        lastPrintMs = now;

        readAllSensors();
        saveSOCToNVS();
        
        uint32_t upSec = (millis() - bootTime) / 1000;
        LOGF("[STATS] Uptime: %lum %lus\n", upSec / 60, upSec % 60);
    }

    esp_task_wdt_reset();
    delay(10);
}

// ═════════════════════════════════════════════════════════════════════
//  SENSOR INIT
// ═════════════════════════════════════════════════════════════════════

void initSensors()
{
    LOG("[SENSOR] Initializing...");

    // 1. INA219 on standard Wire
    Wire.begin(I2C_INA_SDA_PIN, I2C_INA_SCL_PIN);
    
    // Quick scan of Wire (INA219 bus)
    LOG("[I2C] Scanning INA Bus (Wire):");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) LOGF("[I2C]   0x%02X\n", addr);
    }

    // Usually begins on the default Wire since we specified the pins for it
    if (!ina219.begin(&Wire))
    {
        LOG("[SENSOR] WARNING: INA219 not found on SDA 8 / SCL 9");
    }
    else
    {
        ina219.setCalibration_32V_2A();
        LOG("[SENSOR] INA219 ready");
    }

    // 2. DS3231 on Wire1
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
//  SENSOR MOCKING AND READING
// ═════════════════════════════════════════════════════════════════════

void readAllSensors()
{
    LOG("[SENSOR] ── Reading ──");

    // ── Real INA219 Data ─────────────────────────────────────────────
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

    // ── Output to Serial ─────────────────────────────────────────────
    LOGF("[SENSOR]   Voltage    : %.2f V\n",  sensorVoltage);
    LOGF("[SENSOR]   Current    : %.3f A (%s)\n",
         sensorCurrent, signedCurrent_A >= 0 ? "charging" : "discharging");
    LOGF("[SENSOR]   Power      : %.2f W\n",  sensorPower);
    LOGF("[SENSOR]   SOC        : %.1f %%\n", sensorSOC);
    
    if (rtcAvailable && rtcTimeValid) {
        DateTime t = rtc.now();
        LOGF("[SENSOR]   RTC Time   : %04d-%02d-%02d %02d:%02d:%02d\n", 
             t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
    }

    LOG("[SENSOR] ────────────\n");
}

// ═════════════════════════════════════════════════════════════════════
//  SOC ENGINE (With Logic form v8 Flash)
// ═════════════════════════════════════════════════════════════════════

float voltageToSOC(float v, float currentA)
{
    // IR compensation: estimate resting voltage from loaded reading
    // Typical 3S 18650 pack internal resistance ≈ 150–300 mΩ total — tune if needed
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
    // IR-compensated voltage → SOC (uses resting-voltage LUT, corrected for load sag)
    float voltageSoc = voltageToSOC(rawVoltage, signedCurrentA);

    // Initial Seed from LUT
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

    // Time Elapsed
    float elapsedHours = 0.0;

    if (rtcAvailable && rtcTimeValid && lastCoulombUnix > 0)
    {
        uint32_t nowUnix  = (uint32_t)rtc.now().unixtime();
        uint32_t deltaSec = (nowUnix > lastCoulombUnix) ? (nowUnix - lastCoulombUnix) : 0;

        uint32_t maxSec = 180; // Sanity Cap
        if (deltaSec > maxSec) deltaSec = maxSec;

        elapsedHours    = deltaSec / 3600.0;
        lastCoulombUnix = nowUnix;
        LOGF("[SOC] Δt: DS3231  %lu s = %.5f h\n", deltaSec, elapsedHours);
    }
    else
    {
        uint32_t nowMs    = millis();
        uint32_t deltaMs  = nowMs - lastCoulombMs;
        uint32_t maxMs    = 180000;
        if (deltaMs > maxMs) deltaMs = maxMs;

        elapsedHours  = deltaMs / 3600000.0;
        lastCoulombMs = nowMs;
        LOGF("[SOC] Δt: millis()  %.5f h\n", elapsedHours);
    }

    // Coulomb Integration
    float deltaSOC  = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0;
    socEstimate    += deltaSOC;
    socEstimate     = constrain(socEstimate, 0.0, 100.0);

    LOGF("[SOC] V=%.2f I=%+.3fA\n",
         rawVoltage, signedCurrentA);
    LOGF("[SOC] ΔSOC=%+.3f%%  Coulomb→%.1f%%  VLookup=%.1f%%\n",
         deltaSOC, socEstimate, voltageSoc);

    // Voltage-LUT blending — only when truly idle; skip during active discharge/charge
    if (fabs(signedCurrentA) < SOC_IDLE_CURRENT_THRESH_A)
    {
        // Truly idle (<50 mA): gently drift toward resting-voltage SOC
        float before = socEstimate;
        socEstimate  = (1.0f - SOC_VOLTAGE_BLEND) * socEstimate
                       + SOC_VOLTAGE_BLEND * voltageSoc;
        LOGF("[SOC] Idle correction: %.1f%% → %.1f%%\n", before, socEstimate);
    }
    else if (signedCurrentA < -0.05f)
    {
        // DISCHARGING: voltage sags below resting — LUT gives falsely low SOC, skip blend
        LOGF("[SOC] Discharging — Coulomb only (no V-blend)\n");
    }
    else
    {
        // CHARGING: voltage is elevated above resting — LUT gives falsely high SOC, skip blend
        LOGF("[SOC] Charging — Coulomb only (no V-blend)\n");
    }

    return socEstimate;
}

// ═════════════════════════════════════════════════════════════════════
//  NVS PERSISTENCE (Flash logic)
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

    float shunt_mV   = ina219.getShuntVoltage_mV();
    float bus_V      = ina219.getBusVoltage_V();
    float liveVoltage = bus_V + (shunt_mV / 1000.0);

    if (liveVoltage >= 3.0)
    {
        float liveSOC = voltageToSOC(liveVoltage, 0.0f);  // At-boot read: no load, no IR sag
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
