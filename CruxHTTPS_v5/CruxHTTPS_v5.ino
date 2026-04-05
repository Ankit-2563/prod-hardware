// ═══════════════════════════════════════════════════════════════════════
//  CruxHTTPS_v5.ino — Battery Monitor  (Serial-only, no server)
// ─────────────────────────────────────────────────────────────────────
//  Board     : VVM601 (ESP32-S3)  — same pin-out as v4
//  Sensors   : 4 x DHT (temp/humidity), INA219 (current + voltage)
//  Battery   : 11.1 V / 8 Ah  18650 3S  w/ BMS
//              Full: 12.6 V  |  Cutoff: 9.0 V
//  Supply    : 12 V DC  (charging source)
//
//  Purpose   : Print real-time SOC and charging status to Serial Monitor.
//              +ve of both LOAD and BATTERY go through the INA219 high-side
//              shunt so we can measure current direction:
//                 positive current -> charging
//                 negative current -> discharging
//
//  No modem, no Wi-Fi, no HTTP — purely serial diagnostics.
// ═══════════════════════════════════════════════════════════════════════

#include "config.h"

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// --- Sensor objects -------------------------------------------------------
Adafruit_INA219 ina219;

DHT dhtSensors[TEMP_SENSOR_COUNT] = {
    DHT(DHT0_PIN, DHT0_TYPE),
    DHT(DHT1_PIN, DHT1_TYPE),
    DHT(DHT2_PIN, DHT2_TYPE),
    DHT(DHT3_PIN, DHT3_TYPE),
};

const uint8_t dhtPins[TEMP_SENSOR_COUNT]  = {DHT0_PIN, DHT1_PIN, DHT2_PIN, DHT3_PIN};
const uint8_t dhtTypes[TEMP_SENSOR_COUNT] = {DHT0_TYPE, DHT1_TYPE, DHT2_TYPE, DHT3_TYPE};

// --- State ----------------------------------------------------------------
enum ChargeState { CHARGING, DISCHARGING, IDLE_FLOAT };

float sensorTemp     = 0.0;
float sensorHumid    = 0.0;
float sensorCurrent  = 0.0;   // Amps  (signed: +charge, -discharge)
float sensorVoltage  = 0.0;   // Volts (load voltage = bus + shunt)
float sensorPower    = 0.0;   // Watts
float sensorSOC      = 0.0;   // %

ChargeState chargeState = IDLE_FLOAT;

// Coulomb-counting state
float    socEstimate   = -1.0;
uint32_t lastCoulombMs = 0;

// Track last valid voltage for glitch rejection
float lastValidVoltage = 0.0;

uint32_t lastReadMs    = 0;
uint32_t readCount     = 0;
uint32_t bootTime      = 0;

// ═════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println();
    LOG("===========================================================");
    LOG("  Crux Battery Monitor - v5  (Serial-only)");
    LOG("  Battery  : 11.1V 8000mAh  18650 3S  (BMS)");
    LOG("  Full     : 12.6 V");
    LOG("  Cutoff   : 9.0 V");
    LOG("  Supply   : 12 V DC");
    LOG("  Sensor   : INA219 (I2C) + 4x DHT");
    LOG("===========================================================");
    Serial.println();

    initSensors();

    bootTime = millis();
    LOG("[BOOT] Setup complete -- entering monitor loop\n");
}

// ═════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════

void loop()
{
    uint32_t now = millis();

    if (now - lastReadMs >= READ_INTERVAL_MS)
    {
        lastReadMs = now;
        readCount++;

        readAllSensors();
        printDashboard();
    }

    delay(50);  // yield
}

// ═════════════════════════════════════════════════════════════════════
//  SENSOR INIT
// ═════════════════════════════════════════════════════════════════════

void initSensors()
{
    LOG("[SENSOR] Initializing...");

    // DHT sensors
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
        dhtSensors[i].begin();
        LOGF("[SENSOR]   DHT%d on GPIO %d  (sensor #%d)\n",
             dhtTypes[i], dhtPins[i], i);
    }

    // INA219 on I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // ── I2C bus scan (diagnostic) ──
    LOG("[I2C] Scanning bus...");
    int devicesFound = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            LOGF("[I2C]   Device found at 0x%02X\n", addr);
            devicesFound++;
        }
    }
    LOGF("[I2C]   Total devices: %d\n", devicesFound);

    if (!ina219.begin())
    {
        LOG("[SENSOR]   WARNING: INA219 NOT found -- check wiring!");
        LOG("[SENSOR]   SDA=" + String(I2C_SDA_PIN) +
            "  SCL=" + String(I2C_SCL_PIN));
    }
    else
    {
        // Set calibration for 32V / 2A range (default)
        // This covers the 12.6V battery and typical charge/discharge currents
        ina219.setCalibration_32V_2A();
        LOG("[SENSOR]   INA219 ready  (32V / 2A range)");

        // ── Raw INA219 diagnostic dump ──
        delay(100);  // let first conversion complete
        float shunt_mV = ina219.getShuntVoltage_mV();
        float bus_V    = ina219.getBusVoltage_V();
        float cur_mA   = ina219.getCurrent_mA();
        float pwr_mW   = ina219.getPower_mW();
        LOG("[DIAG] ── INA219 Raw Readings ──");
        LOGF("[DIAG]   Shunt Voltage : %+.4f mV\n", shunt_mV);
        LOGF("[DIAG]   Bus Voltage   : %.4f V\n",   bus_V);
        LOGF("[DIAG]   Current       : %+.3f mA\n", cur_mA);
        LOGF("[DIAG]   Power         : %.3f mW\n",  pwr_mW);
        LOGF("[DIAG]   Load Voltage  : %.4f V  (bus + shunt)\n", bus_V + shunt_mV / 1000.0);
        if (bus_V < 1.0)
            LOG("[DIAG]   ⚠ Bus voltage near 0 — VIN-/GND wiring suspect");
        if (fabs(cur_mA) > 3100.0)
            LOG("[DIAG]   ⚠ Current at overflow — shunt resistor / VIN+/VIN- wiring suspect");
        LOG("[DIAG] ────────────────────────");
    }

    LOG("[SENSOR] Init complete\n");
}

// ═════════════════════════════════════════════════════════════════════
//  SENSOR READING
// ═════════════════════════════════════════════════════════════════════

void readAllSensors()
{
    // -- Temperature -------------------------------------------------------
    float tempSum = 0.0;
    int   tempCount = 0;

    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
        float t = dhtSensors[i].readTemperature();
        if (!isnan(t))
        {
            tempSum += t;
            tempCount++;
        }
    }

    if (tempCount > 0)
        sensorTemp = tempSum / tempCount;

    // -- Humidity (from first DHT) -----------------------------------------
    float h = dhtSensors[0].readHumidity();
    if (!isnan(h))
        sensorHumid = h;

    // -- INA219 -- Current / Voltage / Power -------------------------------
    float shuntVoltage_mV = ina219.getShuntVoltage_mV();
    float busVoltage_V    = ina219.getBusVoltage_V();
    float current_mA      = ina219.getCurrent_mA();

    // Load voltage = bus voltage + shunt drop
    float rawVoltage = busVoltage_V + (shuntVoltage_mV / 1000.0);

    // ---- Voltage validation ----
    // If voltage is below 3V, it's a garbage reading (loose GND wire, noise).
    // Ignore it and keep the last valid voltage.
    if (rawVoltage >= 3.0)
    {
        sensorVoltage    = rawVoltage;
        lastValidVoltage = rawVoltage;
    }
    else if (lastValidVoltage > 0)
    {
        // Keep last valid reading, don't update
        sensorVoltage = lastValidVoltage;
    }
    else
    {
        sensorVoltage = rawVoltage;  // no valid reading yet, use what we have
    }

    // Signed current:  positive = charging, negative = discharging
    float signedCurrent_A = (current_mA / 1000.0) * CHARGE_DIRECTION;
    sensorCurrent = signedCurrent_A;

    // Power
    sensorPower = ina219.getPower_mW() / 1000.0;

    // -- Charge State ------------------------------------------------------
    if (signedCurrent_A > CHARGE_CURRENT_THRESH_A)
        chargeState = CHARGING;
    else if (signedCurrent_A < -DISCHARGE_CURRENT_THRESH_A)
        chargeState = DISCHARGING;
    else
        chargeState = IDLE_FLOAT;

    // -- SOC (Coulomb counting + voltage lookup) ---------------------------
    sensorSOC = calculateSOC(sensorVoltage, signedCurrent_A, sensorTemp);
}

// ═════════════════════════════════════════════════════════════════════
//  SOC CALCULATION  -- adapted for 3S 18650 (9.0 - 12.6 V)
// ═════════════════════════════════════════════════════════════════════

// Voltage-to-SOC lookup for 3S Li-ion (18650)
// Based on typical Li-ion discharge curve x 3 cells in series.
//   Per-cell: 3.00 -> 3.30 -> 3.50 -> 3.60 -> 3.70 -> 3.75 -> 3.80 -> 3.85 -> 3.95 -> 4.10 -> 4.20
//   x 3    :  9.00 -> 9.90 -> 10.50-> 10.80-> 11.10-> 11.25-> 11.40-> 11.55-> 11.85-> 12.30-> 12.60
float voltageToSOC(float v)
{
    static const float LUT_V[] = {
         9.00,   // 0%   -- BMS cutoff
         9.90,   // 10%
        10.50,   // 20%
        10.80,   // 30%
        11.10,   // 40%
        11.25,   // 50%
        11.40,   // 60%
        11.55,   // 70%
        11.85,   // 80%
        12.30,   // 90%
        12.60    // 100% -- fully charged
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

// Temperature compensation for Li-ion (per cell ~3 mV per degree C, x 3 cells)
float temperatureCompensate(float v, float tempC)
{
    const float CELLS             = (float)CELL_COUNT;
    const float MV_PER_CELL_PER_C = 0.003;  // 3 mV per degree C typical
    return v + (25.0 - tempC) * CELLS * MV_PER_CELL_PER_C;
}

float calculateSOC(float rawVoltage, float signedCurrentA, float tempC)
{
    float compV      = temperatureCompensate(rawVoltage, tempC);
    float voltageSoc = voltageToSOC(compV);

    // First reading -- seed from voltage
    if (socEstimate < 0.0)
    {
        socEstimate   = voltageSoc;
        lastCoulombMs = millis();
        LOGF("[SOC] Initialized from voltage: %.1f%%\n", socEstimate);
        return socEstimate;
    }

    // Coulomb counting: dSOC = (I x dt) / Capacity x 100
    uint32_t nowMs        = millis();
    float    elapsedHours = (nowMs - lastCoulombMs) / 3600000.0;
    lastCoulombMs = nowMs;

    float deltaSOC  = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0;
    socEstimate    += deltaSOC;
    socEstimate     = constrain(socEstimate, 0.0, 100.0);

    // Idle drift correction -- VERY gentle blend toward voltage lookup
    // Only when current is essentially zero (idle battery).
    // Using 2% weight (was 20%) to prevent SOC from drifting on noise.
    if (fabs(signedCurrentA) < CHARGE_CURRENT_THRESH_A)
    {
        socEstimate = 0.98 * socEstimate + 0.02 * voltageSoc;
    }

    return socEstimate;
}

// ═════════════════════════════════════════════════════════════════════
//  SERIAL DASHBOARD
// ═════════════════════════════════════════════════════════════════════

const char *chargeStateStr(ChargeState s)
{
    switch (s)
    {
        case CHARGING:     return "[CHARGING]";
        case DISCHARGING:  return "[DISCHARGING]";
        case IDLE_FLOAT:   return "[IDLE / FLOAT]";
    }
    return "[UNKNOWN]";
}

void printDashboard()
{
    uint32_t uptimeSec = (millis() - bootTime) / 1000;

    // Clear screen - keeps only latest reading visible in Serial Monitor
    // Works with PuTTY, screen, minicom. Arduino IDE will just show latest at bottom.
    Serial.print("\033[2J\033[H");

    Serial.println("+----------------------------------------------+");
    Serial.println("|          CRUX BATTERY MONITOR  v5            |");
    Serial.println("+----------------------------------------------+");

    LOGF("|  Reading #%-6lu       Uptime: %02lu:%02lu:%02lu    |\n",
         readCount,
         uptimeSec / 3600, (uptimeSec / 60) % 60, uptimeSec % 60);

    Serial.println("+----------------------------------------------+");

    // -- Charge status -------------------------------------------------
    LOGF("|  Status  : %-33s|\n", chargeStateStr(chargeState));

    // -- SOC bar -------------------------------------------------------
    int barLen   = 20;
    int filled   = (int)(sensorSOC / 100.0 * barLen);
    if (filled > barLen) filled = barLen;

    char bar[22];
    for (int i = 0; i < barLen; i++)
        bar[i] = (i < filled) ? '#' : '-';
    bar[barLen] = '\0';

    LOGF("|  SOC     : %5.1f %%  [%s]  |\n", sensorSOC, bar);

    Serial.println("+----------------------------------------------+");

    // -- Electrical readings -------------------------------------------
    LOGF("|  Voltage : %6.2f V                          |\n", sensorVoltage);
    LOGF("|  Current : %+7.3f A                         |\n", sensorCurrent);
    LOGF("|  Power   : %6.2f W                          |\n", sensorPower);

    // Per-cell voltage estimate
    float perCell = sensorVoltage / CELL_COUNT;
    LOGF("|  Per-Cell: %5.2f V  (avg of %dS)              |\n", perCell, CELL_COUNT);

    Serial.println("+----------------------------------------------+");

    // -- Environment ---------------------------------------------------
    LOGF("|  Temp    : %5.1f C                            |\n", sensorTemp);
    LOGF("|  Humid   : %5.1f %%                            |\n", sensorHumid);

    Serial.println("+----------------------------------------------+");

    // -- Diagnostics ---------------------------------------------------
    if (sensorVoltage > BATTERY_FULL_V + 0.1)
    {
        Serial.println("|  WARNING: VOLTAGE ABOVE FULL CHARGE         |");
    }
    if (sensorVoltage < BATTERY_EMPTY_V)
    {
        Serial.println("|  WARNING: VOLTAGE BELOW CUTOFF               |");
    }
    if (chargeState == CHARGING && sensorSOC >= 99.5)
    {
        Serial.println("|  FULL: Battery fully charged (float)         |");
    }
    if (chargeState == DISCHARGING && sensorSOC < 10.0)
    {
        Serial.println("|  LOW BATTERY: Connect charger                |");
    }

    Serial.println("+----------------------------------------------+");
}
