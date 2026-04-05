// ═══════════════════════════════════════════════════════════════════
//  config.h — CruxHTTPS v5 · Battery Monitor (Serial-only, no server)
// ═══════════════════════════════════════════════════════════════════
//
//  Hardware pins identical to v4 (VVM601 ESP32-S3 board).
//  This build strips all modem / server code.
//  Purpose: real-time SOC + charge-status diagnostics on Serial Monitor.
//
//  Battery: 11.1 V  8000 mAh  18650 3S  (BMS onboard)
//           Full charge  : 12.6 V
//           Nominal      : 11.1 V
//           Cutoff       : 9.0 V
//
//  Power supply: 12 V DC (used for charging)
//

#ifndef CONFIG_H
#define CONFIG_H

// ─────────────────────────────────────────────────────────────────
//  DHT TEMPERATURE + HUMIDITY SENSORS  (same GPIOs as v4)
// ─────────────────────────────────────────────────────────────────
#define TEMP_SENSOR_COUNT 4

#define DHT0_PIN  4
#define DHT0_TYPE 22

#define DHT1_PIN  5
#define DHT1_TYPE 22

#define DHT2_PIN  16
#define DHT2_TYPE 22

#define DHT3_PIN  17
#define DHT3_TYPE 11

// ─────────────────────────────────────────────────────────────────
//  I2C  (INA219 current / voltage sensor)
// ─────────────────────────────────────────────────────────────────
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// ─────────────────────────────────────────────────────────────────
//  BATTERY PARAMETERS  — 3S 18650 pack (from label)
// ─────────────────────────────────────────────────────────────────
#define BATTERY_CAPACITY_AH       8.0     // 8000 mAh
#define BATTERY_FULL_V           12.6     // 3 × 4.20 V
#define BATTERY_NOMINAL_V        11.1     // 3 × 3.70 V
#define BATTERY_EMPTY_V           9.0     // 3 × 3.00 V (BMS cutoff)
#define CELL_COUNT                3       // 3S configuration

// Current direction: +1 → charging current is positive from INA219
//                    -1 → charging current is negative (reverse wiring)
#define CHARGE_DIRECTION          1

// Voltage offset to match multimeters due to tiny wire resistance or ADC tolerances 
#define VOLTAGE_CALIBRATION_OFFSET 0.17 

// ─────────────────────────────────────────────────────────────────
//  CHARGING SUPPLY
// ─────────────────────────────────────────────────────────────────
#define SUPPLY_VOLTAGE           12.0     // external PSU voltage

// ─────────────────────────────────────────────────────────────────
//  THRESHOLDS  — charge-state detection
// ─────────────────────────────────────────────────────────────────
// If current   > +CHARGE_CURRENT_THRESH_A  → CHARGING
// If current   < -DISCHARGE_CURRENT_THRESH_A → DISCHARGING
// Otherwise    → IDLE / FLOAT
#define CHARGE_CURRENT_THRESH_A    0.05   // 50 mA
#define DISCHARGE_CURRENT_THRESH_A 0.05   // 50 mA

// ─────────────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────────────
#define READ_INTERVAL_MS  2000            // sensor sample every 2 s
#define SERIAL_BAUD       115200

// ─────────────────────────────────────────────────────────────────
//  DEBUG
// ─────────────────────────────────────────────────────────────────
#define ENABLE_DEBUG true

#if ENABLE_DEBUG
  #define LOG(x)     Serial.println(x)
  #define LOGF(...)  Serial.printf(__VA_ARGS__)
#else
  #define LOG(x)     ((void)0)
  #define LOGF(...)  ((void)0)
#endif

#endif
