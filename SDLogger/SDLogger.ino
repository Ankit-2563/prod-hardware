// ═══════════════════════════════════════════════════════════════════════
//  SDLogger.ino — VVM601 ESP32-S3
// ─────────────────────────────────────────────────────────────────────
//  Logs a random value to /datalog.csv on the SD card every 60 seconds.
//  Uses DS3231 RTC for timestamps (falls back to millis if RTC absent).
//
//  SD Card Module (SPI)        ESP32-S3 Pin
//  ─────────────────────────── ────────────
//  VCC                         5V
//  GND                         GND
//  MOSI                        11
//  MISO                        13
//  SCK                         12
//  CS                          10
//
//  DS3231 RTC (I2C — Wire1)
//  SDA → 14   SCL → 21   VCC → 5V   GND → GND
// ═══════════════════════════════════════════════════════════════════════

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>

// ─────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────────────
#define SD_MOSI      11
#define SD_MISO      13
#define SD_SCK       12
#define SD_CS        10

#define RTC_SDA      14
#define RTC_SCL      21

// ─────────────────────────────────────────────────────────────────────
//  CONFIG
// ─────────────────────────────────────────────────────────────────────
#define LOG_FILE        "/datalog.csv"
#define LOG_INTERVAL_MS  60000UL    // 1 minute

// ─────────────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────────────
RTC_DS3231 rtc;
bool       rtcReady   = false;
bool       sdReady    = false;
uint32_t   lastLogMs  = 0;
uint32_t   rowCount   = 0;

// ─────────────────────────────────────────────────────────────────────
//  HELPERS
// ─────────────────────────────────────────────────────────────────────

// Returns a timestamp string.
// Format: "YYYY-MM-DD HH:MM:SS"  (RTC)  or  "T+NNNNNs"  (millis fallback)
String getTimestamp()
{
    if (rtcReady) {
        DateTime t = rtc.now();
        char buf[24];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                 t.year(), t.month(), t.day(),
                 t.hour(), t.minute(), t.second());
        return String(buf);
    }
    // Fallback: seconds since boot
    char buf[16];
    snprintf(buf, sizeof(buf), "T+%lus", millis() / 1000);
    return String(buf);
}

// Writes the CSV header if the file is brand-new (0 bytes)
void ensureHeader()
{
    // If file already has content, skip header
    File f = SD.open(LOG_FILE, FILE_READ);
    if (f && f.size() > 0) {
        f.close();
        return;
    }
    if (f) f.close();

    File w = SD.open(LOG_FILE, FILE_WRITE);
    if (w) {
        w.println("row,timestamp,value");
        w.close();
        Serial.println("[SD] Header written");
    }
}

// Appends one row to the CSV
void logRow(uint32_t row, const String& ts, int value)
{
    File f = SD.open(LOG_FILE, FILE_APPEND);
    if (!f) {
        Serial.println("[SD] ERROR: Could not open file for append");
        return;
    }
    f.print(row);
    f.print(",");
    f.print(ts);
    f.print(",");
    f.println(value);
    f.close();
}

// ─────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println("═══════════════════════════════════════════");
    Serial.println("  SD Card CSV Logger — VVM601 ESP32-S3");
    Serial.println("═══════════════════════════════════════════");

    // ── DS3231 RTC ───────────────────────────────────────────────────
    Wire1.begin(RTC_SDA, RTC_SCL);
    if (!rtc.begin(&Wire1)) {
        Serial.println("[RTC] DS3231 not found — using millis() for timestamps");
        rtcReady = false;
    } else {
        if (rtc.lostPower()) {
            Serial.println("[RTC] Lost power — setting to compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        rtcReady = true;
        DateTime t = rtc.now();
        Serial.printf("[RTC] Ready — %04d-%02d-%02d %02d:%02d:%02d\n",
                      t.year(), t.month(), t.day(),
                      t.hour(), t.minute(), t.second());
    }

    // ── SD Card ──────────────────────────────────────────────────────
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    if (!SD.begin(SD_CS)) {
        Serial.println("[SD] ERROR: Card not found or init failed");
        Serial.println("     Check wiring: MOSI=11 MISO=13 SCK=12 CS=10");
        sdReady = false;
    } else {
        sdReady = true;
        uint64_t cardMB = SD.cardSize() / (1024 * 1024);
        Serial.printf("[SD] Ready — Card size: %llu MB\n", cardMB);
        ensureHeader();

        // Count existing rows so we continue numbering correctly
        File f = SD.open(LOG_FILE, FILE_READ);
        if (f) {
            while (f.available()) {
                if (f.read() == '\n') rowCount++;
            }
            f.close();
            if (rowCount > 0) rowCount--;   // subtract header line
            Serial.printf("[SD] Existing rows in file: %lu\n", rowCount);
        }
    }

    randomSeed(esp_random());   // True hardware RNG seed on ESP32-S3

    Serial.println();
    Serial.println("Logging every 60 seconds.");
    Serial.println("Row   Timestamp              Value");
    Serial.println("───── ────────────────────── ─────");

    // Log the first reading immediately on boot — don't wait a full minute
    lastLogMs = millis() - LOG_INTERVAL_MS;
}

// ─────────────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────────────
void loop()
{
    if (millis() - lastLogMs < LOG_INTERVAL_MS) return;
    lastLogMs = millis();

    int    value = random(0, 1001);     // Random 0–1000
    String ts    = getTimestamp();
    rowCount++;

    // Print to Serial
    Serial.printf("%-5lu %s   %d\n", rowCount, ts.c_str(), value);

    // Write to SD card
    if (sdReady) {
        logRow(rowCount, ts, value);
    } else {
        Serial.println("      [SD not ready — row NOT saved]");
    }
}
