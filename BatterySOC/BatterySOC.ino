/*
 * =============================================================================
 *  BatterySOC — Real-Time Battery Monitor for VVM601 (ESP32-S3)
 * =============================================================================
 *
 *  Battery : 18650 Li-ion 3S1P 3000 mAh  (11.1 V nom / 12.6 V full / 9.0 V cutoff)
 *  Sensor  : Adafruit INA219 Current-Sensor Breakout (I2C 0x40)
 *  Load    : 2 × 12 V DC Motors (max combined ~3 A  →  1C)
 *  Board   : VVM601 ESP32-S3  (I2C on GPIO 8 SDA, GPIO 9 SCL)
 *
 *  Features
 *  --------
 *  1.  Real SOC  — Coulomb counting (mAh in/out) corrected by voltage lookup
 *  2.  State     — IDLE · CHARGING · DISCHARGING  (auto-detected from INA219 current)
 *  3.  Live data — Voltage · Current · Power printed every second
 *  4.  Charger   — Optional charger-detect pin (GPIO 7 through voltage divider)
 *
 *  Wiring (see circuit diagram)
 *  ----------------------------
 *  ESP32 GPIO 8  (SDA) ─── INA219 SDA
 *  ESP32 GPIO 9  (SCL) ─── INA219 SCL
 *  ESP32 3.3 V          ─── INA219 VCC
 *  ESP32 GND            ─── INA219 GND
 *  Battery +             ─── INA219 VIN+
 *  INA219 VIN-           ─── Load (Motor 1 + Motor 2 in parallel)
 *  Motor GND             ─── Battery –  ─── ESP32 GND  (common ground)
 *  (Optional) Charger +  ─── 100 K ─┬─ 33 K ─── GND
 *                                    └─── ESP32 GPIO 7  (charger detect)
 *
 *  Libraries required (install from Arduino Library Manager)
 *  ---------------------------------------------------------
 *  • Adafruit INA219  (by Adafruit)
 *  • Wire             (built-in)
 * =============================================================================
 */

#include <Wire.h>
#include <Adafruit_INA219.h>

// ─── Pin Definitions ─────────────────────────────────────────────────────────
#define I2C_SDA           8       // ESP32-S3 GPIO for I2C SDA
#define I2C_SCL           9       // ESP32-S3 GPIO for I2C SCL
#define CHARGER_DETECT_PIN 7      // GPIO to detect charger presence (via voltage divider)

// ─── Battery Constants (3S1P 18650 Li-ion) ───────────────────────────────────
#define BATTERY_CAPACITY_MAH   3000.0   // Rated capacity in mAh
#define VOLTAGE_FULL           12.60    // 3S fully charged  (4.20 V × 3)
#define VOLTAGE_NOMINAL        11.10    // 3S nominal        (3.70 V × 3)
#define VOLTAGE_CUTOFF          9.00    // 3S low cutoff     (3.00 V × 3)

// ─── Thresholds ──────────────────────────────────────────────────────────────
#define CURRENT_IDLE_THRESHOLD   50.0   // |current| < 50 mA → IDLE
#define CHARGING_CURRENT_SIGN   -1      // INA219 reads NEGATIVE when current flows INTO battery
                                        // (battery is on high side, charger pushes current backward)

// ─── Timing ──────────────────────────────────────────────────────────────────
#define SAMPLE_INTERVAL_MS     1000     // Read sensor every 1 second
#define SOC_CORRECTION_INTERVAL 30000   // Re-anchor SOC to voltage every 30 seconds (only when IDLE)

// ─── State Enumeration ───────────────────────────────────────────────────────
enum BatteryState {
  STATE_IDLE,
  STATE_CHARGING,
  STATE_DISCHARGING
};

// ─── Global Objects ──────────────────────────────────────────────────────────
Adafruit_INA219 ina219(0x40);

// ─── Runtime Variables ───────────────────────────────────────────────────────
float soc_mAh           = 0.0;      // Remaining capacity via Coulomb counting (mAh)
float soc_percent       = 0.0;      // SOC as 0-100 %
BatteryState battState  = STATE_IDLE;

unsigned long lastSampleTime      = 0;
unsigned long lastCorrectionTime  = 0;
bool ina219Found = false;

// =============================================================================
//  3S Li-ion Voltage → SOC Lookup Table  (OCV at rest, 20–25 °C)
//  Derived from typical 18650 discharge curves scaled to 3S.
// =============================================================================
struct VoltageSocPoint {
  float voltage;
  float soc;        // 0.0 – 100.0
};

// 16-point OCV table for a 3S pack
const VoltageSocPoint ocvTable[] = {
  { 12.60, 100.0 },
  { 12.45,  95.0 },
  { 12.33,  90.0 },
  { 12.18,  85.0 },
  { 12.06,  80.0 },
  { 11.94,  75.0 },
  { 11.82,  70.0 },
  { 11.70,  65.0 },
  { 11.58,  60.0 },
  { 11.46,  55.0 },
  { 11.34,  50.0 },
  { 11.22,  45.0 },
  { 11.10,  40.0 },
  { 10.80,  30.0 },
  { 10.50,  20.0 },
  { 10.20,  15.0 },
  {  9.90,  10.0 },
  {  9.60,   5.0 },
  {  9.30,   2.0 },
  {  9.00,   0.0 },
};
const int ocvTableSize = sizeof(ocvTable) / sizeof(ocvTable[0]);

// =============================================================================
//  Utility: Interpolate SOC from voltage using the OCV table
// =============================================================================
float voltageToSOC(float voltage) {
  // Clamp to table bounds
  if (voltage >= ocvTable[0].voltage)            return 100.0;
  if (voltage <= ocvTable[ocvTableSize - 1].voltage) return 0.0;

  // Linear interpolation between two nearest points
  for (int i = 0; i < ocvTableSize - 1; i++) {
    if (voltage >= ocvTable[i + 1].voltage && voltage <= ocvTable[i].voltage) {
      float vRange   = ocvTable[i].voltage - ocvTable[i + 1].voltage;
      float socRange = ocvTable[i].soc     - ocvTable[i + 1].soc;
      float ratio    = (voltage - ocvTable[i + 1].voltage) / vRange;
      return ocvTable[i + 1].soc + (ratio * socRange);
    }
  }
  return 0.0;
}

// =============================================================================
//  Pretty-print helpers
// =============================================================================
const char* stateToString(BatteryState s) {
  switch (s) {
    case STATE_IDLE:        return "IDLE";
    case STATE_CHARGING:    return "CHARGING ⚡";
    case STATE_DISCHARGING: return "DISCHARGING 🔋";
    default:                return "UNKNOWN";
  }
}

void printSeparator() {
  Serial.println("────────────────────────────────────────────────");
}

void printHeader() {
  Serial.println();
  printSeparator();
  Serial.println("  🔋  BATTERY MONITOR  —  VVM601 + INA219");
  printSeparator();
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  printHeader();
  Serial.println("  Initialising...");

  // ── Charger detect pin ─────────────────────────────────────────────────────
  pinMode(CHARGER_DETECT_PIN, INPUT);

  // ── I2C on custom pins ─────────────────────────────────────────────────────
  Wire.begin(I2C_SDA, I2C_SCL);

  // ── INA219 init ────────────────────────────────────────────────────────────
  if (!ina219.begin()) {
    Serial.println("  ❌  INA219 NOT FOUND at 0x40 !");
    Serial.println("  Check wiring: SDA→GPIO8, SCL→GPIO9, VCC→3.3V, GND→GND");
    ina219Found = false;
  } else {
    ina219Found = true;
    Serial.println("  ✅  INA219 initialised at 0x40");

    // ── Calibrate for 3.2 A max (default 0.1 Ω shunt) ───────────────────────
    // The default setCalibration_32V_2A is fine for up to ~3.2 A actual
    // (the INA219 with the 0.1 Ω shunt can measure ±3.2 A regardless of
    //  the "2A" naming; 32V range is also fine since our pack < 13 V).
    ina219.setCalibration_32V_2A();

    // ── Bootstrap SOC from voltage ───────────────────────────────────────────
    float busV = ina219.getBusVoltage_V();
    float shuntV = ina219.getShuntVoltage_mV() / 1000.0;
    float loadVoltage = busV + shuntV;

    soc_percent = voltageToSOC(loadVoltage);
    soc_mAh     = (soc_percent / 100.0) * BATTERY_CAPACITY_MAH;

    Serial.print("  Initial voltage : ");
    Serial.print(loadVoltage, 2);
    Serial.println(" V");
    Serial.print("  Initial SOC     : ");
    Serial.print(soc_percent, 1);
    Serial.println(" %");
  }

  printSeparator();
  Serial.println();

  lastSampleTime     = millis();
  lastCorrectionTime = millis();
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  if (!ina219Found) {
    Serial.println("❌ INA219 not found — halted. Check wiring and reset.");
    delay(5000);
    return;
  }

  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;

  float dt_hours = (now - lastSampleTime) / 3600000.0;  // elapsed time in hours
  lastSampleTime = now;

  // ── Read INA219 ────────────────────────────────────────────────────────────
  float shuntVoltage_mV = ina219.getShuntVoltage_mV();
  float busVoltage_V    = ina219.getBusVoltage_V();
  float current_mA      = ina219.getCurrent_mA();
  float loadVoltage_V   = busVoltage_V + (shuntVoltage_mV / 1000.0);
  float power_mW        = loadVoltage_V * current_mA;
  float power_W         = power_mW / 1000.0;

  // ── Read charger detect GPIO ───────────────────────────────────────────────
  bool chargerDetected = digitalRead(CHARGER_DETECT_PIN) == HIGH;

  // ── Determine battery state ────────────────────────────────────────────────
  //    Positive current = discharging (battery supplies load)
  //    Negative current = charging    (charger pushes current into battery)
  //    Near-zero        = idle
  if (abs(current_mA) < CURRENT_IDLE_THRESHOLD) {
    battState = STATE_IDLE;
  } else if (current_mA < -CURRENT_IDLE_THRESHOLD || chargerDetected) {
    battState = STATE_CHARGING;
  } else {
    battState = STATE_DISCHARGING;
  }

  // ── Coulomb counting ───────────────────────────────────────────────────────
  //    When discharging, current is positive → subtract from remaining mAh
  //    When charging, current is negative   → add to remaining mAh
  //    We use the raw INA219 current sign convention:
  //      +  = current flowing from battery to load (discharge)
  //      -  = current flowing into battery (charge)
  //    So:  soc_mAh -= current_mA * dt_hours;  (positive current reduces SOC)
  soc_mAh -= current_mA * dt_hours;

  // ── Clamp SOC to valid range ───────────────────────────────────────────────
  if (soc_mAh > BATTERY_CAPACITY_MAH) soc_mAh = BATTERY_CAPACITY_MAH;
  if (soc_mAh < 0.0)                  soc_mAh = 0.0;
  soc_percent = (soc_mAh / BATTERY_CAPACITY_MAH) * 100.0;

  // ── Periodic voltage-based correction (only when IDLE) ─────────────────────
  //    The OCV table is only accurate at rest (no load). When IDLE and voltage
  //    is stable, we slowly blend the voltage-derived SOC to correct drift.
  if (battState == STATE_IDLE && (now - lastCorrectionTime > SOC_CORRECTION_INTERVAL)) {
    float voltageSOC = voltageToSOC(loadVoltage_V);
    // Weighted blend: 70% coulomb counting, 30% voltage map
    soc_percent = (soc_percent * 0.7) + (voltageSOC * 0.3);
    soc_mAh     = (soc_percent / 100.0) * BATTERY_CAPACITY_MAH;
    lastCorrectionTime = now;
  }

  // ── Build SOC bar ──────────────────────────────────────────────────────────
  int barLen = 20;
  int filled = (int)((soc_percent / 100.0) * barLen);
  char bar[32];
  for (int i = 0; i < barLen; i++) {
    bar[i] = (i < filled) ? '#' : '-';
  }
  bar[barLen] = '\0';

  // ── Print to Serial ────────────────────────────────────────────────────────
  printSeparator();
  Serial.printf("  STATE      : %s\n", stateToString(battState));
  Serial.printf("  SOC        : %.1f %%  [%s]\n", soc_percent, bar);
  Serial.printf("  Remaining  : %.0f / %.0f mAh\n", soc_mAh, BATTERY_CAPACITY_MAH);
  printSeparator();
  Serial.printf("  Voltage    : %.2f V   (bus: %.2f V  shunt: %.2f mV)\n",
                loadVoltage_V, busVoltage_V, shuntVoltage_mV);
  Serial.printf("  Current    : %.1f mA  (%.3f A)\n", current_mA, current_mA / 1000.0);
  Serial.printf("  Power      : %.2f W   (%.0f mW)\n", power_W, power_mW);
  printSeparator();
  Serial.printf("  Charger    : %s\n", chargerDetected ? "CONNECTED" : "NOT CONNECTED");
  printSeparator();

  // ── Low-battery warning ────────────────────────────────────────────────────
  if (soc_percent <= 10.0) {
    Serial.println("  ⚠️  LOW BATTERY — consider recharging!");
  }
  if (loadVoltage_V <= VOLTAGE_CUTOFF) {
    Serial.println("  🛑  CRITICAL — voltage at/below cutoff! BMS should disconnect.");
  }
  Serial.println();
}
