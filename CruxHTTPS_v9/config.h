// ═══════════════════════════════════════════════════════════════════
//  config.h — CruxHTTPS v9 (Standalone / Serial Monitor Only)
// ═══════════════════════════════════════════════════════════════════

#ifndef CONFIG_H
#define CONFIG_H

// ─────────────────────────────────────────────────────────────────
//  I2C PINS
// ─────────────────────────────────────────────────────────────────
#define I2C_INA_SDA_PIN 8
#define I2C_INA_SCL_PIN 9

#define I2C_RTC_SDA_PIN 14
#define I2C_RTC_SCL_PIN 21

// ─────────────────────────────────────────────────────────────────
//  BATTERY
// ─────────────────────────────────────────────────────────────────
#define BATTERY_CAPACITY_AH      8.0
#define BATTERY_FULL_V            12.6
#define BATTERY_EMPTY_V           9.0

// +1 = positive INA219 current means charging
// -1 = negative INA219 current means charging (reverse wiring)
#define ACS712_CHARGE_DIRECTION   1

// ─────────────────────────────────────────────────────────────────
//  SOC TUNING
// ─────────────────────────────────────────────────────────────────
#define SOC_IDLE_CURRENT_THRESH_A   0.010f // Only correct when |I| < 10 mA (true rest)
#define SOC_VOLTAGE_BLEND           0.005f // 0.5%/s blend — 10× slower (was 0.05)
#define SOC_REBOOT_MAX_DEVIATION    25.0f
#define NVS_MAX_RESTORE_SECONDS     86400UL
#define SELF_DISCHARGE_PCT_PER_HOUR 0.003f

// ─────────────────────────────────────────────────────────────────
//  TIMING & LOOP
// ─────────────────────────────────────────────────────────────────
#define LOOP_DELAY_MS        1000
#define WDT_TIMEOUT_SECONDS  120

// ─────────────────────────────────────────────────────────────────
//  DEBUG
// ─────────────────────────────────────────────────────────────────
#define SERIAL_BAUD  115200

#define LOG(x)    Serial.println(x)
#define LOGF(...) Serial.printf(__VA_ARGS__)

#endif
