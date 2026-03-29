// ═══════════════════════════════════════════════════════════════════
//  config.h — Crux IoT Platform · Production Configuration
// ═══════════════════════════════════════════════════════════════════

#ifndef CONFIG_H
#define CONFIG_H

// Device ID, secrets, server host, TLS cert, API paths
#include "secrets.h"

// ─────────────────────────────────────────────────────────────────
//  EC200U 4G MODEM (VVM601 — pins fixed on PCB)
// ─────────────────────────────────────────────────────────────────
#define MODEM_RX 40
#define MODEM_TX 41
#define MODEM_POWER 42
#define MODEM_BAUD 115200

//  APN — empty = auto-detect (works for Vi / Vodafone Idea 4G)
#define GPRS_APN ""
#define GPRS_USER ""
#define GPRS_PASS ""

// Temperature sensors — 1× DHT11 + 3× DHT22
// Average of all valid readings is sent to the server.
#define TEMP_SENSOR_COUNT 4

//  Index 0 — DHT11 on GPIO 4  (confirmed)
#define DHT0_PIN 4
#define DHT0_TYPE 22 // DHT11

//  Index 1 — DHT22 on GPIO 5
#define DHT1_PIN 5
#define DHT1_TYPE 22 // DHT22

//  Index 2 — DHT22 on GPIO 16
#define DHT2_PIN 16
#define DHT2_TYPE 22 // DHT22

//  Index 3 — DHT22 on GPIO 39
#define DHT3_PIN 17
#define DHT3_TYPE 11 // DHT22

#define ACS712_PIN 2
#define ZMPT101B_PIN 3

// ─────────────────────────────────────────────────────────────────
//  SENSOR CALIBRATION (12 V DC supply / battery, motor load)
//
//  WHY CALIBRATE?
//  The ESP32 ADC and analog front ends are not factory-calibrated to "real world"
//  amps and volts. Allegro ACS712 modules use a Hall sensor; output offset and
//  gain vary slightly per chip. Any resistor divider for voltage scales the ADC
//  reading — one multiplier ties ADC volts to battery volts.
//
//  ACS712 — CURRENT (see Allegro ACS712 datasheet: sensitivity by variant)
//  Formula in firmware: I = (V_adc - ACS712_ZERO_POINT) / ACS712_SENSITIVITY
//    • 5A  module: typically 185 mV/A  → 0.185
//    • 20A module: typically 100 mV/A → 0.100
//    • 30A module: typically 66 mV/A  → 0.066
//  Ten 12 V motors can sum to many amps — if total continuous current can exceed
//  ~4 A, prefer ACS712-20A or -30A and set ACS712_SENSITIVITY to match the chip.
//
//  ZERO (ACS712_ZERO_POINT): With NO current through the sensor, measure the
//  voltage at the ESP32 ADC pin (after any divider) with a multimeter. Set
//  ACS712_ZERO_POINT to that value. If the Hall IC is powered at 5 V, nominal
//  zero is 2.5 V at the chip; a divider to 3.3 V logic scales that — always use
//  the measured voltage at the pin.
//
//  GAIN (ACS712_SENSITIVITY): With a KNOWN DC current (multimeter in series,
//  or bench supply CC + known load), read Serial debug V or compute from raw ADC.
//  sensitivity = (V_adc_at_known_I - ACS712_ZERO_POINT) / I_known_amperes.
//
//  ZMPT101B vs 12 V DC (important):
//  The ZMPT101B is an AC voltage transformer module — it measures mains AC (e.g.
//  230 V AC), not steady 12 V DC from a battery. For 12 V DC bus voltage use a
//  resistor divider (e.g. 39 kΩ + 10 kΩ from 12 V to GND, tap to ADC) or a DC
//  INA219/ADS1115 + shunt. If you still use the ZMPT101B analog input *pin* for
//  a DC-divided signal, treat ZMPT101B_* macros as "DC bus scale": set
//  ZMPT101B_ZERO_POINT to ADC voltage at 0 V battery (often ~0), and
//  ZMPT101B_CAL_FACTOR = V_multimeter / V_reading_from_firmware at one known
//  battery voltage (e.g. 12.60 V floated).
//
//  ─────────────────────────────────────────────────────────────────
#define ADC_RESOLUTION 4095.0
#define ADC_REF_VOLTAGE 3.3

// ACS712: match ACS712_SENSITIVITY to your chip (5A / 20A / 30A)
#define ACS712_SENSITIVITY 0.185 // V per A (5A: 0.185 | 20A: 0.100 | 30A: 0.066)
#define ACS712_ZERO_POINT 1.65   // V at ADC pin at 0 A — measure, do not guess
#define ACS712_SAMPLES 500       // averaging reduces noise from motors PWM

// Bus voltage: scale from ADC to "real" V (see ZMPT101B vs DC note above)
#define ZMPT101B_CAL_FACTOR 1.0  // V_true / (V_adc - ZERO) at calibration point
#define ZMPT101B_ZERO_POINT 1.65 // DC divider: often ~0; AC module: ~Vcc/2 at ADC
#define ZMPT101B_SAMPLES 500

// Battery SOC estimation
//
//  BATTERY_CAPACITY_AH — the Ah rating printed on your battery label.
//  Common sizes: 7Ah (small UPS), 26Ah (mid), 100Ah (large solar).
//  ⚠ Set this correctly — it directly scales the coulomb counting accuracy.
#define BATTERY_CAPACITY_AH 100.0 // ← change to your battery's Ah rating

#define BATTERY_FULL_V 14.4  // voltage when fully charged (bulk charge done)
#define BATTERY_EMPTY_V 10.5 // voltage when fully discharged (cut-off)

//  ACS712 current direction
//  The ACS712 output goes ABOVE 1.65V when current flows in one direction
//  and BELOW 1.65V in the other. Which direction is "charging" depends on
//  how you wired the sensor on your PCB.
//
//  HOW TO CHECK:
//    1. Flash with ENABLE_DEBUG true
//    2. Open Serial Monitor
//    3. While battery is clearly CHARGING, look at "[SOC] I=" in the log
//    4. If current shows as NEGATIVE during charging → set this to -1
//    5. If current shows as POSITIVE during charging → leave as +1  (default)
#define ACS712_CHARGE_DIRECTION 1 // +1 or -1

// ─────────────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────────────
#define SEND_INTERVAL_MS 600000 // 10 minutes -> 1 req per 10 min
#define REGISTER_RETRY_MS 10000
#define NETWORK_TIMEOUT_MS 180000 // 3 min max wait for network
#define HTTP_TIMEOUT_MS 15000
#define MAX_RETRIES 3
#define RETRY_BACKOFF_MS 5000

// ─────────────────────────────────────────────────────────────────
//  DEBUG — set false to silence Serial output in production
// ─────────────────────────────────────────────────────────────────
#define ENABLE_DEBUG true
#define SERIAL_BAUD 115200

// Uncomment to dump raw AT commands (very verbose):
// #define DUMP_AT_COMMANDS

#if ENABLE_DEBUG
#define LOG(x) Serial.println(x)
#define LOGF(...) Serial.printf(__VA_ARGS__)
#else
#define LOG(x)
#define LOGF(...)
#endif

#endif
