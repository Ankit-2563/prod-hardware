// ═══════════════════════════════════════════════════════════════════
//  config.h — CruxHTTPS v8
// ═══════════════════════════════════════════════════════════════════

#ifndef CONFIG_H
#define CONFIG_H

#include "secrets.h"

// ─────────────────────────────────────────────────────────────────
//  MODEM
// ─────────────────────────────────────────────────────────────────
#define MODEM_RX    40
#define MODEM_TX    41
#define MODEM_POWER 42
#define MODEM_BAUD  115200

#define GPRS_APN  ""
#define GPRS_USER ""
#define GPRS_PASS ""

// ─────────────────────────────────────────────────────────────────
//  DHT SENSORS
// ─────────────────────────────────────────────────────────────────
#define TEMP_SENSOR_COUNT 3

#define DHT0_PIN  4
#define DHT0_TYPE 22

#define DHT1_PIN  5
#define DHT1_TYPE 22

#define DHT2_PIN  16
#define DHT2_TYPE 22

// ─────────────────────────────────────────────────────────────────
//  I2C  (INA219 @ 0x40, DS3231 @ 0x68 — shared bus)
// ─────────────────────────────────────────────────────────────────
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// ─────────────────────────────────────────────────────────────────
//  BATTERY
// ─────────────────────────────────────────────────────────────────
#define BATTERY_CAPACITY_AH      100.0
#define BATTERY_FULL_V            14.4
#define BATTERY_EMPTY_V           10.5

// +1 = positive INA219 current means charging
// -1 = negative INA219 current means charging (reverse wiring)
// Check with ENABLE_DEBUG: look at [SOC] I= while charging
#define ACS712_CHARGE_DIRECTION   1

// ─────────────────────────────────────────────────────────────────
//  SOC TUNING
// ─────────────────────────────────────────────────────────────────

// Current below this → battery at rest → idle drift correction applies
#define SOC_IDLE_CURRENT_THRESH_A   0.3f

// Weight of voltage-LUT SOC in idle blend (0.05 = gentle, 0.20 = aggressive)
// Lower = less jumpy but slower to self-correct. 0.05 is a safe default.
#define SOC_VOLTAGE_BLEND           0.05f

// On reboot, if saved SOC and live-voltage SOC differ more than this,
// discard saved SOC and re-seed from voltage instead.
// 25% is a generous tolerance — tighten to 15% if battery rarely rests.
#define SOC_REBOOT_MAX_DEVIATION    25.0f

// Maximum time the board can have been off and still trust the saved SOC.
// 24 hours = 86400 s. Beyond this, re-seed from voltage.
#define NVS_MAX_RESTORE_SECONDS     86400UL

// Self-discharge rate for lead-acid during power-off period.
// ~0.003% per hour = ~2.2% per month (conservative AGM estimate).
#define SELF_DISCHARGE_PCT_PER_HOUR 0.003f

// ─────────────────────────────────────────────────────────────────
//  TIMING & LOOP
// ─────────────────────────────────────────────────────────────────
#define LOOP_DELAY_MS      1000
#define ADAPTIVE_LOOP_DELAY true
#define LOOP_DELAY_MIN_MS    50

#define SEND_INTERVAL_MS   60000UL   // 1 minute

#define REGISTER_RETRY_MS  10000
#define NETWORK_TIMEOUT_MS 180000
#define HTTP_TIMEOUT_MS    15000
#define MAX_RETRIES            3
#define RETRY_BACKOFF_MS    5000

#define WDT_TIMEOUT_SECONDS 120

// ─────────────────────────────────────────────────────────────────
//  BUFFERS
// ─────────────────────────────────────────────────────────────────
#define JSON_DOC_CAPACITY   192
#define JSON_SERIAL_BUFFER  256
#define HTTP_HEADER_BUFFER  512
#define HTTP_RESP_BODY_MAX  256

// ─────────────────────────────────────────────────────────────────
//  DEBUG
// ─────────────────────────────────────────────────────────────────
#define ENABLE_DEBUG true
#define SERIAL_BAUD  115200
// #define DUMP_AT_COMMANDS

#if ENABLE_DEBUG
  #define LOG(x)    Serial.println(x)
  #define LOGF(...) Serial.printf(__VA_ARGS__)
#else
  #define LOG(x)    ((void)0)
  #define LOGF(...) ((void)0)
#endif

#endif
