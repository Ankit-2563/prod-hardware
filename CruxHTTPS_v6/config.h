// ═══════════════════════════════════════════════════════════════════
//  config.h — CruxHTTPS v6
// ═══════════════════════════════════════════════════════════════════
//
//  Combines:
//    • v4 modem, server, timing, watchdog, HTTP buffer settings
//    • v5 battery parameters (3S 18650 Li-ion), SOC thresholds,
//      INA219 charge-direction, cell count
//
//  Sensors: 3 × DHT22 (DHT11 removed)
//  Battery: 11.1V / 8 Ah  3S 18650 with BMS
//           Full: 12.6V  |  Cutoff: 9.0V
//

#ifndef CONFIG_H
#define CONFIG_H

#include "secrets.h"

// ─────────────────────────────────────────────────────────────────
//  EC200U 4G MODEM  (VVM601 — pins fixed on PCB)
// ─────────────────────────────────────────────────────────────────
#define MODEM_RX    40
#define MODEM_TX    41
#define MODEM_POWER 42
#define MODEM_BAUD  115200

#define GPRS_APN  ""
#define GPRS_USER ""
#define GPRS_PASS ""

// ─────────────────────────────────────────────────────────────────
//  DHT22 TEMPERATURE + HUMIDITY SENSORS  (3 sensors, no DHT11)
// ─────────────────────────────────────────────────────────────────
#define TEMP_SENSOR_COUNT 3

#define DHT0_PIN  4
#define DHT0_TYPE 22

#define DHT1_PIN  5
#define DHT1_TYPE 22

#define DHT2_PIN  16
#define DHT2_TYPE 22

// ─────────────────────────────────────────────────────────────────
//  I2C  (INA219 current / voltage sensor)
// ─────────────────────────────────────────────────────────────────
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// ─────────────────────────────────────────────────────────────────
//  BATTERY PARAMETERS  — 3S 18650 Li-ion pack
// ─────────────────────────────────────────────────────────────────
#define BATTERY_CAPACITY_AH  8.0    // 8000 mAh
#define BATTERY_FULL_V      12.6    // 3 × 4.20 V
#define BATTERY_NOMINAL_V   11.1    // 3 × 3.70 V
#define BATTERY_EMPTY_V      9.0    // 3 × 3.00 V  (BMS cutoff)
#define CELL_COUNT           3      // 3S configuration

// Current direction:
//   +1  → charging current reads positive from INA219
//   -1  → charging current reads negative (reverse shunt wiring)
#define CHARGE_DIRECTION    -1

// ─────────────────────────────────────────────────────────────────
//  CHARGE-STATE DETECTION THRESHOLDS
// ─────────────────────────────────────────────────────────────────
//  |I| > CHARGE_CURRENT_THRESH_A    → CHARGING
//  |I| > DISCHARGE_CURRENT_THRESH_A → DISCHARGING
//  Otherwise                        → IDLE / FLOAT
#define CHARGE_CURRENT_THRESH_A     0.05f   // 50 mA
#define DISCHARGE_CURRENT_THRESH_A  0.05f   // 50 mA

// ─────────────────────────────────────────────────────────────────
//  TIMING & LOOP
// ─────────────────────────────────────────────────────────────────
// Sensor read + dashboard update interval (fast, local only)
#define READ_INTERVAL_MS    2000UL

// Server POST interval — exactly 1 per minute
#define SEND_INTERVAL_MS   60000UL

// Loop housekeeping (adaptive delay trims this toward next POST)
#define LOOP_DELAY_MS      1000
#define ADAPTIVE_LOOP_DELAY true
#define LOOP_DELAY_MIN_MS  50

// ─────────────────────────────────────────────────────────────────
//  NETWORK & HTTP
// ─────────────────────────────────────────────────────────────────
#define REGISTER_RETRY_MS   10000
#define NETWORK_TIMEOUT_MS  180000
#define HTTP_TIMEOUT_MS     15000
#define MAX_RETRIES         3
#define RETRY_BACKOFF_MS    5000

// ─────────────────────────────────────────────────────────────────
//  WATCHDOG
//  Must exceed worst-case modem boot + TLS handshake + HTTP round-trip
// ─────────────────────────────────────────────────────────────────
#define WDT_TIMEOUT_SECONDS 120

// ─────────────────────────────────────────────────────────────────
//  JSON / HTTP BUFFER SIZES
// ─────────────────────────────────────────────────────────────────
// Data payload has 7 fields (temp, humid, voltage, current, power,
// soc, chargeState) — 256 bytes is comfortable.
#define JSON_DOC_CAPACITY    256
#define JSON_SERIAL_BUFFER   320

// Request line + headers (no body); 512 bytes covers long paths + auth
#define HTTP_HEADER_BUFFER   512

// Response body bytes captured for debug logging
#define HTTP_RESP_BODY_MAX   256

// ─────────────────────────────────────────────────────────────────
//  DEBUG
// ─────────────────────────────────────────────────────────────────
#define ENABLE_DEBUG  true
#define SERIAL_BAUD   115200
// #define DUMP_AT_COMMANDS   // uncomment to mirror raw AT traffic

#if ENABLE_DEBUG
  #define LOG(x)    Serial.println(x)
  #define LOGF(...) Serial.printf(__VA_ARGS__)
#else
  #define LOG(x)    ((void)0)
  #define LOGF(...) ((void)0)
#endif

#endif // CONFIG_H
