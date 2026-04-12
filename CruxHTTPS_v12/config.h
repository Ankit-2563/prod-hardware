// ═══════════════════════════════════════════════════════════════════
//  config.h — CruxHTTPS v12
//  Combined: v4 server logic + v9 SOC engine + rest-detection fix + SD buffer+cycles
// ═══════════════════════════════════════════════════════════════════

#ifndef CONFIG_H
#define CONFIG_H

#include "secrets.h"

// ─────────────────────────────────────────────────────────────────
//  EC200U 4G MODEM (VVM601 — pins fixed on PCB)
// ─────────────────────────────────────────────────────────────────
#define MODEM_RX    40
#define MODEM_TX    41
#define MODEM_POWER 42
#define MODEM_BAUD  115200

#define GPRS_APN  ""
#define GPRS_USER ""
#define GPRS_PASS ""

// ─────────────────────────────────────────────────────────────────
//  DHT SENSORS (3x DHT22, 1x DHT11)
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
//  I2C PINS — Two separate buses
//  Bus 1 (Wire) : INA219 current sensor
//  Bus 2 (Wire1): DS3231 RTC
// ─────────────────────────────────────────────────────────────────
#define I2C_INA_SDA_PIN  8
#define I2C_INA_SCL_PIN  9

#define I2C_RTC_SDA_PIN 14
#define I2C_RTC_SCL_PIN 21

// ─────────────────────────────────────────────────────────────────
//  BATTERY
// ─────────────────────────────────────────────────────────────────
#define BATTERY_CAPACITY_AH      8.0
#define BATTERY_FULL_V           12.6
#define BATTERY_EMPTY_V           9.0

// +1 = positive INA219 current means charging
// -1 = negative INA219 current means charging (reverse wiring)
#define ACS712_CHARGE_DIRECTION   1

// ─────────────────────────────────────────────────────────────────
//  SOC TUNING
// ─────────────────────────────────────────────────────────────────
#define SOC_IDLE_CURRENT_THRESH_A   0.010f  // 10mA — only true rest triggers correction
#define SOC_REBOOT_MAX_DEVIATION    25.0f
#define NVS_MAX_RESTORE_SECONDS     86400UL
#define SELF_DISCHARGE_PCT_PER_HOUR 0.003f

// ─────────────────────────────────────────────────────────────────
//  TIMING & LOOP
// ─────────────────────────────────────────────────────────────────
#define LOOP_DELAY_MS       1000
#define ADAPTIVE_LOOP_DELAY true
#define LOOP_DELAY_MIN_MS     50

#define SEND_INTERVAL_MS   60000UL   // 1 request per minute

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
//  SD CARD (SPI)
// ─────────────────────────────────────────────────────────────────
#define SD_MOSI_PIN   11
#define SD_MISO_PIN   13
#define SD_SCK_PIN    12
#define SD_CS_PIN     10

// ─────────────────────────────────────────────────────────────────
//  OFFLINE BUFFER
// ─────────────────────────────────────────────────────────────────
#define SD_DATA_FILE        "/crux_buffer.csv"
#define SD_CYCLE_FILE       "/crux_cycles.txt"
#define SD_FLUSH_BATCH_SIZE  20      // records to send per flush attempt
#define CHARGE_CYCLE_SOC_LOW  20.0f  // % — cycle starts below this
#define CHARGE_CYCLE_SOC_HIGH 80.0f  // % — cycle completes above this

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
