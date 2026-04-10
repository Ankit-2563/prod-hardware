// ═══════════════════════════════════════════════════════════════════
//  config.h — CruxHTTPS v3 · Production configuration
// ═══════════════════════════════════════════════════════════════════
//
//  v3 goals: less heap churn (stack JSON + HTTP headers), tighter timing
//  before uploads, watchdog-friendly sensor path, sane production defaults.
//

#ifndef CONFIG_H
#define CONFIG_H

#include "secrets.h"

// ─────────────────────────────────────────────────────────────────
//  EC200U 4G MODEM (VVM601 — pins fixed on PCB)
// ─────────────────────────────────────────────────────────────────
#define MODEM_RX 40
#define MODEM_TX 41
#define MODEM_POWER 42
#define MODEM_BAUD 115200

#define GPRS_APN ""
#define GPRS_USER ""
#define GPRS_PASS ""

#define TEMP_SENSOR_COUNT 3

#define DHT0_PIN 4
#define DHT0_TYPE 22

#define DHT1_PIN 5
#define DHT1_TYPE 22

#define DHT2_PIN 16
#define DHT2_TYPE 22

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// ─────────────────────────────────────────────────────────────────
//  DS3231 RTC & AT24C32 EEPROM (Shared I2C Bus)
// ─────────────────────────────────────────────────────────────────
#define RTC_SDA_PIN I2C_SDA_PIN   // Same as INA219
#define RTC_SCL_PIN I2C_SCL_PIN   // Same as INA219
#define EEPROM_I2C_ADDR 0x57      // AT24C32 Default I2C Address
//  SENSOR CALIBRATION — see v2 config comments; values unchanged by default
// ─────────────────────────────────────────────────────────────────
// Note: INA219 handles its own calibration internally.

#define BATTERY_CAPACITY_AH 100.0
#define BATTERY_FULL_V 14.4
#define BATTERY_EMPTY_V 10.5
#define ACS712_CHARGE_DIRECTION 1

// ─────────────────────────────────────────────────────────────────
//  TIMING & LOOP
//
//  ADAPTIVE_LOOP_DELAY — wake sooner as the next POST approaches so you
//  do not overshoot SEND_INTERVAL_MS by a full LOOP_DELAY_MS tick.
//
//  WDT_TIMEOUT_SECONDS — must exceed worst-case network + HTTP handshake.
// ─────────────────────────────────────────────────────────────────
#define LOOP_DELAY_MS 1000
#define ADAPTIVE_LOOP_DELAY true
#define LOOP_DELAY_MIN_MS 50

#define SEND_INTERVAL_MS 60000UL

#define REGISTER_RETRY_MS 10000
#define NETWORK_TIMEOUT_MS 180000
#define HTTP_TIMEOUT_MS 15000
#define MAX_RETRIES 3
#define RETRY_BACKOFF_MS 5000

#define WDT_TIMEOUT_SECONDS 120

// JSON on stack: registration + data payloads stay under this in practice
#define JSON_DOC_CAPACITY 192
#define JSON_SERIAL_BUFFER 256

// Request line + headers (no JSON body); keep Host/path realistic
#define HTTP_HEADER_BUFFER 512

// Response body snippet logged when ENABLE_DEBUG
#define HTTP_RESP_BODY_MAX 256

// ─────────────────────────────────────────────────────────────────
#define ENABLE_DEBUG true
#define SERIAL_BAUD 115200
// #define DUMP_AT_COMMANDS

#if ENABLE_DEBUG
#define LOG(x) Serial.println(x)
#define LOGF(...) Serial.printf(__VA_ARGS__)
#else
#define LOG(x) ((void)0)
#define LOGF(...) ((void)0)
#endif

#endif
