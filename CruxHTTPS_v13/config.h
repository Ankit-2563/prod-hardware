// ═══════════════════════════════════════════════════════════════════
//  config.h — CruxHTTPS v11
//  SOC: ChargeSDLogger engine  |  Comms: v4 HTTPS nginx proxy style
//  Offline: SD CSV fallback + flush on reconnect
// ═══════════════════════════════════════════════════════════════════

#ifndef CONFIG_H
#define CONFIG_H

#include "secrets.h"

// ─────────────────────────────────────────────────────────────────
//  EC200U 4G MODEM  (VVM601 — pins fixed on PCB)
// ─────────────────────────────────────────────────────────────────
#define MODEM_RX 40
#define MODEM_TX 41
#define MODEM_POWER 42
#define MODEM_BAUD 115200

#define GPRS_APN ""
#define GPRS_USER ""
#define GPRS_PASS ""

// ─────────────────────────────────────────────────────────────────
//  DHT SENSORS  (3x DHT22, 1x DHT11)
// ─────────────────────────────────────────────────────────────────
#define TEMP_SENSOR_COUNT 4

#define DHT0_PIN 4
#define DHT0_TYPE 22

#define DHT1_PIN 5
#define DHT1_TYPE 22

#define DHT2_PIN 16
#define DHT2_TYPE 22

#define DHT3_PIN 17
#define DHT3_TYPE 11

// ─────────────────────────────────────────────────────────────────
//  I2C PINS — Two separate buses
//  Bus 1 (Wire)  : INA219 current/voltage sensor
//  Bus 2 (Wire1) : DS3231 RTC
// ─────────────────────────────────────────────────────────────────
#define I2C_INA_SDA_PIN 8
#define I2C_INA_SCL_PIN 9

#define I2C_RTC_SDA_PIN 14
#define I2C_RTC_SCL_PIN 21

// ─────────────────────────────────────────────────────────────────
//  SD CARD (SPI)
// ─────────────────────────────────────────────────────────────────
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13
#define SD_SCK_PIN 12
#define SD_CS_PIN 10

#define LOG_FILE "/chargelog.csv"

// ─────────────────────────────────────────────────────────────────
//  BATTERY  (3S Li-ion 8 Ah pack)
// ─────────────────────────────────────────────────────────────────
#define BATTERY_CAPACITY_AH 8.0f
#define BATTERY_FULL_V 12.6f
#define BATTERY_EMPTY_V 9.0f

// +1 → positive INA219 current means CHARGING  (VIN- = Battery+)
// -1 → positive INA219 current means DISCHARGING (reversed wiring)
#define ACS712_CHARGE_DIRECTION 1

// ─────────────────────────────────────────────────────────────────
//  SOC TUNING  (from ChargeSDLogger)
// ─────────────────────────────────────────────────────────────────
#define IDLE_THRESHOLD_A 0.020f            // ±20 mA → IDLE state
#define SOC_IDLE_CURRENT_THRESH_A 0.010f   // <10 mA → start rest timer
#define SOC_REBOOT_MAX_DEVIATION 25.0f     // % — discard NVS if off by more
#define NVS_MAX_RESTORE_SECONDS 86400UL    // 24 h — max off-time for NVS restore
#define SELF_DISCHARGE_PCT_PER_HOUR 0.003f // 0.3%/h self-discharge model

// ─────────────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────────────
#define LOOP_DELAY_MS 1000UL          // main loop cadence (also sensor read rate)
#define SEND_INTERVAL_MS 60000UL      // live data push every 60 s
#define LOCAL_LOG_INTERVAL_MS 60000UL // SD card log every 60 s

#define REGISTER_RETRY_MS 10000
#define NETWORK_TIMEOUT_MS 60000 // shorter than v10 — fail fast → offline mode
#define HTTP_TIMEOUT_MS 15000
#define MAX_RETRIES 3
#define RETRY_BACKOFF_MS 5000

#define WDT_TIMEOUT_SECONDS 120

// ─────────────────────────────────────────────────────────────────
//  BUFFERS
// ─────────────────────────────────────────────────────────────────
#define JSON_DOC_CAPACITY 320
#define JSON_SERIAL_BUFFER 640
#define HTTP_HEADER_BUFFER 512
#define HTTP_RESP_BODY_MAX 256

// ─────────────────────────────────────────────────────────────────
//  SERVER  (via nginx HTTPS reverse proxy — v4 style)
//  USE_HTTPS must match your nginx/cert setup.
//  SERVER_PORT 443 for HTTPS, 80 for plain HTTP behind proxy.
// ─────────────────────────────────────────────────────────────────
// These are defined in secrets.h:
//   SERVER_HOST, SERVER_PORT, REGISTER_PATH, DATA_PATH
//   DEVICE_ID, DEVICE_SECRET, DEVICE_NAME, FIRMWARE_VER

// ─────────────────────────────────────────────────────────────────
//  DEBUG
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

#endif // CONFIG_H