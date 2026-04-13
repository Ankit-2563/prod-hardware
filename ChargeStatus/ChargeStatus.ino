// ═══════════════════════════════════════════════════════════════════════
//  ChargeStatus.ino — Crux IoT
// ─────────────────────────────────────────────────────────────────────
//  Detects battery charge state (CHARGING / DISCHARGING / IDLE) via
//  INA219 current direction.  Logs to Serial Monitor every second.
//
//  Wiring:
//    INA219  SDA→8  SCL→9   VCC→5V  GND→GND
//    DS3231  SDA→14 SCL→21  VCC→5V  GND→GND
//    Battery (+) → INA219 VIN(-)
//    INA219 VIN(+) → Load (+)
//    Battery (−) / Load (−) → GND
//    Charger (+) → Battery (+) terminal   [same node as VIN-]
//    Charger (−) → GND
// ═══════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>

// ─────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────────────
#define I2C_INA_SDA   8
#define I2C_INA_SCL   9
#define I2C_RTC_SDA   14
#define I2C_RTC_SCL   21

// ─────────────────────────────────────────────────────────────────────
//  TUNING
// ─────────────────────────────────────────────────────────────────────

// Minimum current (A) to count as charging or discharging.
// Below this = IDLE (sensor noise, leakage, etc.)
#define IDLE_THRESHOLD_A   0.020f   // 20 mA

// +1 if positive INA219 current means CHARGING
// -1 if positive INA219 current means DISCHARGING (reversed wiring)
#define CHARGE_DIRECTION   1

// Print interval
#define PRINT_INTERVAL_MS  1000

// ─────────────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────────────
Adafruit_INA219 ina219;
RTC_DS3231      rtc;

bool rtcReady = false;
uint32_t lastPrintMs = 0;

typedef enum {
    STATE_IDLE,
    STATE_CHARGING,
    STATE_DISCHARGING
} ChargeState;

ChargeState prevState = STATE_IDLE;

// ─────────────────────────────────────────────────────────────────────
//  HELPERS
// ─────────────────────────────────────────────────────────────────────
const char* stateLabel(ChargeState s)
{
    switch (s) {
        case STATE_CHARGING:    return "CHARGING";
        case STATE_DISCHARGING: return "DISCHARGING";
        default:                return "IDLE";
    }
}

ChargeState currentToState(float signedA)
{
    if (signedA >= IDLE_THRESHOLD_A)   return STATE_CHARGING;
    if (signedA <= -IDLE_THRESHOLD_A)  return STATE_DISCHARGING;
    return STATE_IDLE;
}

void printRTCTime()
{
    if (!rtcReady) return;
    DateTime t = rtc.now();
    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d  ",
                  t.year(), t.month(), t.day(),
                  t.hour(), t.minute(), t.second());
}

// ─────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(200);

    Serial.println("═══════════════════════════════════════════");
    Serial.println("  Charge Status Monitor — INA219 + DS3231");
    Serial.println("═══════════════════════════════════════════");

    // ── INA219 on Wire ───────────────────────────────────────────────
    Wire.begin(I2C_INA_SDA, I2C_INA_SCL);

    if (!ina219.begin(&Wire)) {
        Serial.println("[ERROR] INA219 not found on SDA 8 / SCL 9");
        Serial.println("        Check wiring, then reset.");
        while (true) delay(1000);   // Halt — no point running without sensor
    }
    ina219.setCalibration_32V_2A();
    Serial.println("[OK] INA219 ready");

    // ── DS3231 on Wire1 ──────────────────────────────────────────────
    Wire1.begin(I2C_RTC_SDA, I2C_RTC_SCL);

    if (!rtc.begin(&Wire1)) {
        Serial.println("[WARN] DS3231 not found — timestamps disabled");
        rtcReady = false;
    } else {
        if (rtc.lostPower()) {
            Serial.println("[RTC] Lost power — setting to compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        rtcReady = true;
        DateTime t = rtc.now();
        Serial.printf("[OK] DS3231 ready — %04d-%02d-%02d %02d:%02d:%02d\n",
                      t.year(), t.month(), t.day(),
                      t.hour(), t.minute(), t.second());
    }

    Serial.println();
    Serial.println("Timestamp            Voltage   Current    Power     State");
    Serial.println("──────────────────── ───────── ──────────  ───────── ─────────────");
}

// ─────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────
void loop()
{
    if (millis() - lastPrintMs < PRINT_INTERVAL_MS) return;
    lastPrintMs = millis();

    // ── Read INA219 ──────────────────────────────────────────────────
    float shunt_mV   = ina219.getShuntVoltage_mV();
    float bus_V      = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW   = ina219.getPower_mW();
    float voltage    = bus_V + (shunt_mV / 1000.0f);

    // Apply wiring direction: positive = charging
    // current_mA is positive when current flows VIN+ → VIN- (into battery)
    float signedA = (current_mA / 1000.0f) * (float)CHARGE_DIRECTION;

    ChargeState state = currentToState(signedA);

    // ── Print row ────────────────────────────────────────────────────
    printRTCTime();     // prints nothing (no trailing space) if RTC absent

    Serial.printf("V: %5.2f V   I: %+7.3f A   P: %6.2f W   → %s\n",
                  voltage,
                  signedA,
                  power_mW / 1000.0f,
                  stateLabel(state));

    // ── State-change alert ───────────────────────────────────────────
    if (state != prevState) {
        Serial.println();
        Serial.printf("  *** STATUS CHANGED: %s → %s ***\n\n",
                      stateLabel(prevState), stateLabel(state));
        prevState = state;
    }
}
