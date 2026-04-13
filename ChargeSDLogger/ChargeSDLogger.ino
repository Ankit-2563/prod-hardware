// ═══════════════════════════════════════════════════════════════════════
//  ChargeSDLogger.ino — Crux IoT
// ─────────────────────────────────────────────────────────────────────
//  Combines ChargeStatus + SDLogger:
//    • Reads voltage, current, power, SOC, charge-state via INA219
//    • Full SOC engine (Coulomb counting + rest-anchor + NVS persistence)
//    • Prints all metrics to Serial every second
//    • Appends one CSV row to SD card every 60 seconds
//    • Reports SD write success/failure to Serial after every log attempt
//
//  Wiring:
//    INA219  SDA→8   SCL→9   VCC→5V  GND→GND
//    DS3231  SDA→14  SCL→21  VCC→5V  GND→GND
//    SD Card MOSI→11 MISO→13 SCK→12  CS→10   VCC→5V  GND→GND
//
//  SD CSV columns:
//    row, timestamp, voltage_V, current_A, power_W, soc_pct, state
// ═══════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <Preferences.h>

// ─────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────────────
#define I2C_INA_SDA  8
#define I2C_INA_SCL  9
#define I2C_RTC_SDA  14
#define I2C_RTC_SCL  21

#define SD_MOSI      11
#define SD_MISO      13
#define SD_SCK       12
#define SD_CS        10

// ─────────────────────────────────────────────────────────────────────
//  BATTERY CONFIG
// ─────────────────────────────────────────────────────────────────────
#define BATTERY_CAPACITY_AH     8.0f   // 8 Ah 3S Li-ion pack
#define BATTERY_FULL_V          12.6f
#define BATTERY_EMPTY_V         9.0f

// +1  → positive INA219 current means CHARGING  (VIN- = Battery+)
// -1  → positive INA219 current means DISCHARGING (reversed wiring)
#define CHARGE_DIRECTION        1

// ─────────────────────────────────────────────────────────────────────
//  SOC TUNING
// ─────────────────────────────────────────────────────────────────────
#define IDLE_THRESHOLD_A            0.020f
#define SOC_IDLE_CURRENT_THRESH_A   0.010f
#define SOC_REBOOT_MAX_DEVIATION    25.0f
#define NVS_MAX_RESTORE_SECONDS     86400UL
#define SELF_DISCHARGE_PCT_PER_HOUR 0.003f

// ─────────────────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────────────────
#define PRINT_INTERVAL_MS   1000UL    // Serial print every 1 s
#define LOG_INTERVAL_MS     60000UL   // SD card log every 60 s

// ─────────────────────────────────────────────────────────────────────
//  SD FILE
// ─────────────────────────────────────────────────────────────────────
#define LOG_FILE "/chargelog.csv"

// ─────────────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────────────
Adafruit_INA219 ina219;
RTC_DS3231      rtc;
Preferences     prefs;

bool     rtcAvailable   = false;
bool     rtcTimeValid   = false;
bool     sdReady        = false;

uint32_t lastPrintMs    = 0;
uint32_t lastLogMs      = 0;
uint32_t bootTime       = 0;
uint32_t logRowCount    = 0;

// SOC engine state
float    socEstimate      = -1.0f;
uint32_t lastCoulombUnix  = 0;
uint32_t lastCoulombMs    = 0;
float    lastValidVoltage = 0.0f;

// Rest-detection state
uint32_t restStartMs  = 0;
bool     isResting    = false;
bool     anchorDone   = false;

typedef enum { STATE_IDLE, STATE_CHARGING, STATE_DISCHARGING } ChargeState;
ChargeState prevState = STATE_IDLE;

// ─────────────────────────────────────────────────────────────────────
//  HELPERS — charge state
// ─────────────────────────────────────────────────────────────────────
const char *stateLabel(ChargeState s)
{
    switch (s) {
        case STATE_CHARGING:    return "CHARGING";
        case STATE_DISCHARGING: return "DISCHARGING";
        default:                return "IDLE";
    }
}

ChargeState currentToState(float signedA)
{
    if (signedA >=  IDLE_THRESHOLD_A) return STATE_CHARGING;
    if (signedA <= -IDLE_THRESHOLD_A) return STATE_DISCHARGING;
    return STATE_IDLE;
}

// ─────────────────────────────────────────────────────────────────────
//  SOC ENGINE — Voltage LUT
// ─────────────────────────────────────────────────────────────────────
float voltageToSOC(float v, float currentA)
{
    const float IR_OHM = 0.25f;
    float restingV = v + fabs(currentA) * IR_OHM;

    static const float LUT_V[]   = {9.00f, 9.90f,10.65f,10.95f,11.10f,
                                    11.25f,11.40f,11.55f,11.70f,12.00f,12.60f};
    static const float LUT_SOC[] = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f,
                                    50.0f, 60.0f, 70.0f, 80.0f, 90.0f,100.0f};
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

// ─────────────────────────────────────────────────────────────────────
//  SOC ENGINE — Coulomb counting + rest-detection anchor
// ─────────────────────────────────────────────────────────────────────
float calculateSOC(float rawVoltage, float signedCurrentA)
{
    float voltageSoc = voltageToSOC(rawVoltage, signedCurrentA);

    // ── Initial seed ─────────────────────────────────────────────────
    if (socEstimate < 0.0f) {
        if (rawVoltage < 3.0f) {
            Serial.println("[SOC] Waiting for valid battery voltage...");
            return 0.0f;
        }
        socEstimate = voltageSoc;
        if (rtcAvailable && rtcTimeValid)
            lastCoulombUnix = (uint32_t)rtc.now().unixtime();
        lastCoulombMs = millis();
        Serial.printf("[SOC] ★ Seeded from LUT: %.1f%%  (V=%.2f)\n",
                      socEstimate, rawVoltage);
        return socEstimate;
    }

    // ── Elapsed time (RTC preferred, millis fallback) ─────────────────
    float elapsedHours = 0.0f;

    if (rtcAvailable && rtcTimeValid && lastCoulombUnix > 0) {
        uint32_t nowUnix = (uint32_t)rtc.now().unixtime();
        uint32_t deltaSec = (nowUnix > lastCoulombUnix) ? (nowUnix - lastCoulombUnix) : 0;
        if (deltaSec > 180) deltaSec = 180;
        elapsedHours    = deltaSec / 3600.0f;
        lastCoulombUnix = nowUnix;
        Serial.printf("[SOC] Δt: DS3231  %lu s = %.5f h\n", deltaSec, elapsedHours);
    } else {
        uint32_t nowMs   = millis();
        uint32_t deltaMs = nowMs - lastCoulombMs;
        if (deltaMs > 180000) deltaMs = 180000;
        elapsedHours  = deltaMs / 3600000.0f;
        lastCoulombMs = nowMs;
        Serial.printf("[SOC] Δt: millis()  %.5f h\n", elapsedHours);
    }

    // ── Coulomb integration ───────────────────────────────────────────
    float deltaSOC = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0f;
    socEstimate   += deltaSOC;
    socEstimate    = constrain(socEstimate, 0.0f, 100.0f);

    Serial.printf("[SOC] V=%.2f I=%+.3fA\n", rawVoltage, signedCurrentA);
    Serial.printf("[SOC] ΔSOC=%+.3f%%  Coulomb→%.1f%%  VLookup=%.1f%%\n",
                  deltaSOC, socEstimate, voltageSoc);

    // ── Rest detection (5-minute confirmation) ────────────────────────
    uint32_t nowMs = millis();

    if (fabs(signedCurrentA) < SOC_IDLE_CURRENT_THRESH_A) {
        if (!isResting) {
            isResting    = true;
            restStartMs  = nowMs;
            anchorDone   = false;
            Serial.println("[SOC] Rest timer started — waiting 5 min");
        } else if (!anchorDone && (nowMs - restStartMs >= 300000UL)) {
            float before  = socEstimate;
            socEstimate   = (0.90f * socEstimate) + (0.10f * voltageSoc);
            anchorDone    = true;
            Serial.printf("[SOC] ★ 5min REST ANCHOR: %.1f%% → %.1f%%"
                          " (VLookup=%.1f%%)\n", before, socEstimate, voltageSoc);
        } else if (!anchorDone) {
            uint32_t waited = (nowMs - restStartMs) / 1000;
            Serial.printf("[SOC] Resting %lus/300s — Coulomb only\n", waited);
        } else {
            Serial.println("[SOC] Idle — Coulomb only");
        }
    } else {
        if (isResting) {
            isResting  = false;
            anchorDone = false;
            Serial.println("[SOC] Load/charge detected — rest cancelled");
        }
        if (signedCurrentA < 0)
            Serial.printf("[SOC] Discharging %.3fA — Coulomb only\n", fabs(signedCurrentA));
        else
            Serial.printf("[SOC] Charging %.3fA — Coulomb only\n", signedCurrentA);
    }

    return socEstimate;
}

// ─────────────────────────────────────────────────────────────────────
//  NVS PERSISTENCE
// ─────────────────────────────────────────────────────────────────────
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
    uint32_t savedTS  = prefs.getUInt ("ts",  0);
    prefs.end();

    if (savedSOC < 0.0f || savedSOC > 100.0f) {
        Serial.println("[NVS] No saved SOC — will seed from voltage on first read");
        return;
    }

    float shunt_mV  = ina219.getShuntVoltage_mV();
    float bus_V     = ina219.getBusVoltage_V();
    float liveVoltage = bus_V + (shunt_mV / 1000.0f);

    if (liveVoltage >= 3.0f) {
        float liveSOC = voltageToSOC(liveVoltage, 0.0f);
        float diff    = fabs(savedSOC - liveSOC);
        Serial.printf("[NVS] Saved=%.1f%%  LiveV=%.2fV  VLookup=%.1f%%  Diff=%.1f%%\n",
                      savedSOC, liveVoltage, liveSOC, diff);
        if (diff > SOC_REBOOT_MAX_DEVIATION) {
            Serial.printf("[NVS] Deviation %.1f%% > %.1f%% — discarding saved SOC\n",
                          diff, SOC_REBOOT_MAX_DEVIATION);
            return;
        }
    }

    float restored = savedSOC;

    if (rtcAvailable && rtcTimeValid && savedTS > 0) {
        uint32_t nowUnix = (uint32_t)rtc.now().unixtime();
        uint32_t offSec  = (nowUnix > savedTS) ? (nowUnix - savedTS) : 0;
        if (offSec > NVS_MAX_RESTORE_SECONDS) {
            Serial.printf("[NVS] Off for %lu s > %lu s limit — re-seeding from voltage\n",
                          offSec, NVS_MAX_RESTORE_SECONDS);
            return;
        }
        float offHours  = offSec / 3600.0f;
        float selfDisch = offHours * SELF_DISCHARGE_PCT_PER_HOUR;
        restored       -= selfDisch;
        restored        = constrain(restored, 0.0f, 100.0f);
        Serial.printf("[NVS] Off %.2fh → self-disch -%.2f%% → restored %.1f%%\n",
                      offHours, selfDisch, restored);
    } else {
        Serial.printf("[NVS] Restored SOC=%.1f%% (no RTC timestamp)\n", restored);
    }

    socEstimate     = restored;
    lastCoulombUnix = savedTS;
    lastCoulombMs   = millis();
}

// ─────────────────────────────────────────────────────────────────────
//  RTC HELPERS
// ─────────────────────────────────────────────────────────────────────
void printRTCTime()
{
    if (!rtcAvailable || !rtcTimeValid) return;
    DateTime t = rtc.now();
    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d  ",
                  t.year(), t.month(), t.day(),
                  t.hour(), t.minute(), t.second());
}

// Returns timestamp string: "YYYY-MM-DD HH:MM:SS" or "T+NNNNs"
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

// ─────────────────────────────────────────────────────────────────────
//  SD HELPERS
// ─────────────────────────────────────────────────────────────────────
void ensureCSVHeader()
{
    File f = SD.open(LOG_FILE, FILE_READ);
    bool hasContent = f && (f.size() > 0);
    if (f) f.close();
    if (hasContent) return;   // Header already present

    File w = SD.open(LOG_FILE, FILE_WRITE);
    if (w) {
        w.println("row,timestamp,voltage_V,current_A,power_W,soc_pct,state");
        w.close();
        Serial.println("[SD] CSV header written");
    } else {
        Serial.println("[SD] ERROR: Could not write CSV header");
    }
}

// Returns true on success, false on failure
bool logRowToSD(uint32_t row, const String &ts,
                float voltage, float current,
                float power,   float soc,
                const char *state)
{
    File f = SD.open(LOG_FILE, FILE_APPEND);
    if (!f) return false;

    f.print(row);       f.print(",");
    f.print(ts);        f.print(",");
    f.print(voltage, 3); f.print(",");
    f.print(current, 4); f.print(",");
    f.print(power,   3); f.print(",");
    f.print(soc,     1); f.print(",");
    f.println(state);
    f.close();
    return true;
}

// ─────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println("═══════════════════════════════════════════════════════");
    Serial.println("  ChargeSDLogger — INA219 + DS3231 + SD Card");
    Serial.println("  SOC: Coulomb counting + rest-detection + NVS");
    Serial.println("  Logs: Serial every 1s  |  SD CSV every 60s");
    Serial.println("═══════════════════════════════════════════════════════");

    // ── INA219 on Wire (SDA 8, SCL 9) ───────────────────────────────
    Wire.begin(I2C_INA_SDA, I2C_INA_SCL);
    Serial.println("[I2C] Scanning INA bus (Wire):");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
            Serial.printf("[I2C]   0x%02X\n", addr);
    }
    if (!ina219.begin(&Wire)) {
        Serial.println("[ERROR] INA219 not found — check SDA 8 / SCL 9. Halting.");
        while (true) delay(1000);
    }
    ina219.setCalibration_32V_2A();
    Serial.println("[OK] INA219 ready");

    // ── DS3231 on Wire1 (SDA 14, SCL 21) ────────────────────────────
    Wire1.begin(I2C_RTC_SDA, I2C_RTC_SCL);
    Serial.println("[I2C] Scanning RTC bus (Wire1):");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0)
            Serial.printf("[I2C]   0x%02X\n", addr);
    }
    if (!rtc.begin(&Wire1)) {
        Serial.println("[WARN] DS3231 not found — millis() fallback for SOC Δt & timestamps");
        rtcAvailable = false;
    } else {
        rtcAvailable = true;
        if (rtc.lostPower()) {
            Serial.println("[RTC] Lost power — setting to compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        DateTime t = rtc.now();
        rtcTimeValid = (t.year() >= 2024);
        Serial.printf("[OK] DS3231 ready — %04d-%02d-%02d %02d:%02d:%02d (valid: %s)\n",
                      t.year(), t.month(), t.day(),
                      t.hour(), t.minute(), t.second(),
                      rtcTimeValid ? "yes" : "no");
    }

    // ── SD Card (SPI: MOSI 11, MISO 13, SCK 12, CS 10) ──────────────
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
        Serial.println("[WARN] SD card not found or init failed — CSV logging disabled");
        Serial.println("       Check wiring: MOSI=11 MISO=13 SCK=12 CS=10");
        sdReady = false;
    } else {
        sdReady = true;
        uint64_t cardMB = SD.cardSize() / (1024ULL * 1024ULL);
        Serial.printf("[OK] SD ready — Card size: %llu MB\n", cardMB);
        ensureCSVHeader();

        // Count existing data rows so numbering continues correctly after reboot
        File f = SD.open(LOG_FILE, FILE_READ);
        if (f) {
            while (f.available()) {
                if (f.read() == '\n') logRowCount++;
            }
            f.close();
            if (logRowCount > 0) logRowCount--;   // subtract header line
            Serial.printf("[SD] Existing rows in log: %lu\n", logRowCount);
        }
    }

    // ── Restore SOC from NVS ─────────────────────────────────────────
    restoreSOCFromNVS();

    bootTime   = millis();
    lastLogMs  = millis() - LOG_INTERVAL_MS;   // Log immediately on boot

    Serial.println();
    Serial.println("Timestamp            Voltage   Current    Power     SOC      State");
    Serial.println("──────────────────── ───────── ──────────  ───────── ──────── ─────────────");
}

// ─────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────
void loop()
{
    uint32_t now = millis();

    // ── Only proceed on 1-second tick ───────────────────────────────
    if (now - lastPrintMs < PRINT_INTERVAL_MS) return;
    lastPrintMs = now;

    // ── Read INA219 ──────────────────────────────────────────────────
    float shunt_mV  = ina219.getShuntVoltage_mV();
    float bus_V     = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW   = ina219.getPower_mW();
    float rawVoltage = bus_V + (shunt_mV / 1000.0f);

    // Voltage glitch filter
    float voltage;
    if (rawVoltage >= 3.0f) {
        voltage          = rawVoltage;
        lastValidVoltage = rawVoltage;
    } else if (lastValidVoltage > 0.0f) {
        voltage = lastValidVoltage;
        Serial.println("[SENSOR] Voltage glitch — using last valid");
    } else {
        voltage = rawVoltage;
    }

    // Apply wiring direction: positive signedA = charging
    float signedA = (current_mA / 1000.0f) * (float)CHARGE_DIRECTION;
    float powerW  = power_mW / 1000.0f;

    // ── SOC calculation ──────────────────────────────────────────────
    float soc = calculateSOC(voltage, signedA);

    // ── Save SOC to NVS every cycle ──────────────────────────────────
    saveSOCToNVS();

    // ── Determine charge state ───────────────────────────────────────
    ChargeState state = currentToState(signedA);

    // ── Print 1-second summary row ───────────────────────────────────
    printRTCTime();
    Serial.printf("V: %5.2fV  I: %+7.3fA  P: %6.3fW  SOC: %5.1f%%  → %s\n",
                  voltage, signedA, powerW, soc, stateLabel(state));

    // ── State-change alert ───────────────────────────────────────────
    if (state != prevState) {
        Serial.println();
        Serial.printf("  *** STATUS CHANGED: %s → %s ***\n\n",
                      stateLabel(prevState), stateLabel(state));
        prevState = state;
    }

    // ── SD card log (every 60 seconds) ──────────────────────────────
    if (now - lastLogMs >= LOG_INTERVAL_MS) {
        lastLogMs = now;
        logRowCount++;
        String ts = getTimestamp();

        Serial.println();
        Serial.printf("[SD] Logging row %lu  @ %s\n", logRowCount, ts.c_str());

        if (!sdReady) {
            Serial.println("[SD] ✗ SD not ready — row NOT saved");
        } else {
            bool ok = logRowToSD(logRowCount, ts,
                                 voltage, signedA, powerW, soc,
                                 stateLabel(state));
            if (ok) {
                Serial.printf("[SD] ✓ Row %lu saved: %s, %.3fV, %+.4fA, "
                              "%.3fW, %.1f%%, %s\n",
                              logRowCount, ts.c_str(),
                              voltage, signedA, powerW, soc,
                              stateLabel(state));
            } else {
                Serial.printf("[SD] ✗ FAILED to write row %lu — check SD card\n",
                              logRowCount);
            }
        }
        Serial.println();
    }
}
