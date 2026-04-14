// ═══════════════════════════════════════════════════════════════════════
//  FinalCruxFirmware.ino — Crux IoT · final firmware
// ─────────────────────────────────────────────────────────────────────
//  Combined firmware:
//    - Temperature sensors from v4 (3x DHT22, 1x DHT11)
//    - Server registration + data sending from v4 (1 req/min)
//    - SOC engine from v9 (RTC Coulomb counting + NVS + rest-detection)
//    - Two separate I2C buses (INA219 on Wire, DS3231 on Wire1)
// ═══════════════════════════════════════════════════════════════════════

#include "config.h"
#include <esp_task_wdt.h>

// ── TinyGSM must be configured BEFORE the include ────────────────────
#define TINY_GSM_MODEM_BG96
#define TINY_GSM_USE_GPRS true
#define SerialAT Serial1
#define SerialMon Serial

#include <TinyGsmClient.h>
#include <ArduinoJson.h>

#if !defined(ARDUINOJSON_VERSION_MAJOR) || (ARDUINOJSON_VERSION_MAJOR < 7)
#define CRUX_USE_JSONDOC_V6 1
#else
#define CRUX_USE_JSONDOC_V6 0
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#if USE_HTTPS
TinyGsmClientSecure netClient(modem);
#else
TinyGsmClient netClient(modem);
#endif

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <Preferences.h>

Adafruit_INA219 ina219;
RTC_DS3231 rtc;
Preferences prefs;

DHT dhtSensors[TEMP_SENSOR_COUNT] = {
    DHT(DHT0_PIN, DHT0_TYPE),
    DHT(DHT1_PIN, DHT1_TYPE),
    DHT(DHT2_PIN, DHT2_TYPE),
    DHT(DHT3_PIN, DHT3_TYPE),
};

const uint8_t dhtPins[TEMP_SENSOR_COUNT] = {DHT0_PIN, DHT1_PIN, DHT2_PIN, DHT3_PIN};
const uint8_t dhtTypes[TEMP_SENSOR_COUNT] = {DHT0_TYPE, DHT1_TYPE, DHT2_TYPE, DHT3_TYPE};

bool registered = false;
uint32_t lastSendMs = 0;
uint32_t bootTime = 0;
uint32_t successCount = 0;
uint32_t failCount = 0;

float sensorTemp = 0.0;
float sensorHumid = 0.0;
float sensorCurrent = 0.0;
float sensorVoltage = 0.0;
float sensorPower = 0.0;
float sensorSOC = 0.0;

float socEstimate = -1.0;
uint32_t lastCoulombUnix = 0;
uint32_t lastCoulombMs = 0;
float lastValidVoltage = 0.0;

bool rtcAvailable = false;
bool rtcTimeValid = false;

uint32_t restStartMs = 0;
bool isResting = false;
bool anchorDone = false;

static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret);
static inline void safeWdtReset()
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (esp_task_wdt_status(NULL) == ESP_OK)
        esp_task_wdt_reset();
#else
    esp_task_wdt_reset();
#endif
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(200);

    LOG("═══════════════════════════════════════════════");
    LOG("  Crux IoT · Firmware v" FIRMWARE_VER);
    LOG("  Device : " DEVICE_NAME);
    LOG("  ID     : " DEVICE_ID);
    LOG("  Server : " SERVER_HOST);
    LOGF("  Mode   : %s (port %d)\n", USE_HTTPS ? "HTTPS" : "HTTP", SERVER_PORT);
    LOG("═══════════════════════════════════════════════\n");

    initSensors();
    restoreSOCFromNVS();

    powerOnModem();
    initModem();
    waitForNetwork();
    connectGPRS();
    syncRTCFromNetwork();
    registerWithRetry();

    bootTime = millis();
    LOG("\n[BOOT] Setup complete — entering main loop\n");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true};
    esp_task_wdt_init(&wdt_cfg);
    esp_task_wdt_add(NULL);
#else
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    esp_task_wdt_add(NULL);
#endif
    LOG("[BOOT] Watchdog active");
}

void loop()
{
    ensureConnected();
    if (!registered)
        registerWithRetry();

    uint32_t now = millis();
    if (registered && (now - lastSendMs >= SEND_INTERVAL_MS))
    {
        lastSendMs = now;
        readAllSensors();
        saveSOCToNVS();
        bool ok = sendSensorData();
        if (ok) successCount++;
        else failCount++;
        uint32_t upSec = (millis() - bootTime) / 1000;
        LOGF("[STATS] Uptime: %lum %lus | Sent: %lu ok, %lu fail\n",
             upSec / 60, upSec % 60, successCount, failCount);
    }
    safeWdtReset();
    delay(10);
}

void initSensors()
{
    LOG("[SENSOR] Initializing...");
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
        dhtSensors[i].begin();
        LOGF("[SENSOR]   DHT%d on GPIO %d  (sensor #%d)\n", dhtTypes[i], dhtPins[i], i);
    }
    Wire.begin(I2C_INA_SDA_PIN, I2C_INA_SCL_PIN);
    if (!ina219.begin(&Wire)) LOG("[SENSOR] WARNING: INA219 not found on SDA 8 / SCL 9");
    else { ina219.setCalibration_32V_2A(); LOG("[SENSOR] INA219 ready"); }

    Wire1.begin(I2C_RTC_SDA_PIN, I2C_RTC_SCL_PIN);
    if (!rtc.begin(&Wire1))
    {
        LOG("[RTC] DS3231 not found on SDA 14 / SCL 21 — dt uses millis()");
        rtcAvailable = false;
    }
    else
    {
        rtcAvailable = true;
        if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        DateTime t = rtc.now();
        rtcTimeValid = (t.year() >= 2024);
    }
    LOG("[SENSOR] Ready\n");
}

void syncRTCFromNetwork()
{
    if (!rtcAvailable) return;
    LOG("[RTC] Syncing from network time...");
    modem.sendAT("+CTZU=1");
    modem.waitResponse(1000);
    delay(1000);
    String dt = modem.getGSMDateTime(DATE_FULL);
    LOG("[RTC] Modem time string: " + dt);
    if (dt.length() < 17) return;

    int yr = 0, mo = 0, dy = 0, hr = 0, mi = 0, sc = 0;
    if (dt.length() >= 19 && dt.charAt(4) == '/')
    {
        yr = dt.substring(0, 4).toInt();
        mo = dt.substring(5, 7).toInt();
        dy = dt.substring(8, 10).toInt();
        hr = dt.substring(11, 13).toInt();
        mi = dt.substring(14, 16).toInt();
        sc = dt.substring(17, 19).toInt();
    }
    else
    {
        yr = dt.substring(0, 2).toInt() + 2000;
        mo = dt.substring(3, 5).toInt();
        dy = dt.substring(6, 8).toInt();
        hr = dt.substring(9, 11).toInt();
        mi = dt.substring(12, 14).toInt();
        sc = dt.substring(15, 17).toInt();
    }
    if (yr < 2024 || yr > 2099) return;
    rtc.adjust(DateTime(yr, mo, dy, hr, mi, sc));
    rtcTimeValid = true;
}

float voltageToSOC(float v, float currentA)
{
    const float INTERNAL_RESISTANCE_OHM = 0.25f;
    float restingV = v + fabs(currentA) * INTERNAL_RESISTANCE_OHM;
    static const float LUT_V[] = {9.00, 9.90, 10.65, 10.95, 11.10, 11.25, 11.40, 11.55, 11.70, 12.00, 12.60};
    static const float LUT_SOC[] = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0};
    const int N = 11;
    if (restingV <= LUT_V[0]) return 0.0;
    if (restingV >= LUT_V[N - 1]) return 100.0;
    for (int i = 1; i < N; i++)
        if (restingV <= LUT_V[i])
            return LUT_SOC[i - 1] + ((restingV - LUT_V[i - 1]) / (LUT_V[i] - LUT_V[i - 1])) * (LUT_SOC[i] - LUT_SOC[i - 1]);
    return 100.0;
}

float calculateSOC(float rawVoltage, float signedCurrentA)
{
    float voltageSoc = voltageToSOC(rawVoltage, signedCurrentA);
    if (socEstimate < 0.0)
    {
        if (rawVoltage < 3.0)
        {
            LOG("[SOC] Waiting for valid battery voltage...");
            return 0.0;
        }
        socEstimate = voltageSoc;
        if (rtcAvailable && rtcTimeValid)
            lastCoulombUnix = (uint32_t)rtc.now().unixtime();
        lastCoulombMs = millis();
        LOGF("[SOC] Seeded from voltage LUT: %.1f%% (V=%.2f)\n", socEstimate, rawVoltage);
        return socEstimate;
    }

    float elapsedHours = 0.0;
    if (rtcAvailable && rtcTimeValid && lastCoulombUnix > 0)
    {
        uint32_t nowUnix = (uint32_t)rtc.now().unixtime();
        uint32_t deltaSec = (nowUnix > lastCoulombUnix) ? (nowUnix - lastCoulombUnix) : 0;
        if (deltaSec > 180) deltaSec = 180;
        elapsedHours = deltaSec / 3600.0;
        lastCoulombUnix = nowUnix;
        LOGF("[SOC] dt RTC: %lus (%.5fh)\n", deltaSec, elapsedHours);
    }
    else
    {
        uint32_t nowMs = millis();
        uint32_t deltaMs = nowMs - lastCoulombMs;
        if (deltaMs > 180000) deltaMs = 180000;
        elapsedHours = deltaMs / 3600000.0;
        lastCoulombMs = nowMs;
        LOGF("[SOC] dt millis: %.5fh\n", elapsedHours);
    }
    float deltaSOC = (signedCurrentA * elapsedHours / BATTERY_CAPACITY_AH) * 100.0;
    socEstimate += deltaSOC;
    socEstimate = constrain(socEstimate, 0.0, 100.0);

    // v9 rest-anchor: after 5 minutes near-idle, gently align to voltage SOC.
    uint32_t nowMs = millis();
    if (fabs(signedCurrentA) < SOC_IDLE_CURRENT_THRESH_A)
    {
        if (!isResting)
        {
            isResting = true;
            restStartMs = nowMs;
            anchorDone = false;
            LOG("[SOC] Rest timer started");
        }
        else if (!anchorDone && (nowMs - restStartMs >= 300000UL))
        {
            float before = socEstimate;
            socEstimate = (0.90f * socEstimate) + (0.10f * voltageSoc);
            anchorDone = true;
            LOGF("[SOC] 5min REST ANCHOR: %.1f%% -> %.1f%%\n", before, socEstimate);
        }
    }
    else
    {
        if (isResting)
        {
            isResting = false;
            anchorDone = false;
            LOG("[SOC] Load detected - rest cancelled");
        }
    }

    LOGF("[SOC] V=%.2f I=%+.3fA dSOC=%+.3f%% => %.1f%%\n",
         rawVoltage, signedCurrentA, deltaSOC, socEstimate);
    return socEstimate;
}

void readAllSensors()
{
    float tempSum = 0.0; int tempCount = 0;
    for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
    {
        float t = dhtSensors[i].readTemperature();
        if (!isnan(t)) { tempSum += t; tempCount++; }
        if ((i & 1) == 1) safeWdtReset();
    }
    if (tempCount > 0) sensorTemp = tempSum / tempCount;
    float h = dhtSensors[0].readHumidity();
    if (!isnan(h)) sensorHumid = h;
    safeWdtReset();

    float shunt_mV = ina219.getShuntVoltage_mV();
    float bus_V = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float rawVoltage = bus_V + (shunt_mV / 1000.0);
    if (rawVoltage >= 3.0) { sensorVoltage = rawVoltage; lastValidVoltage = rawVoltage; }
    else if (lastValidVoltage > 0.0) sensorVoltage = lastValidVoltage;
    else sensorVoltage = rawVoltage;
    float signedCurrent_A = (current_mA / 1000.0) * (float)ACS712_CHARGE_DIRECTION;
    sensorCurrent = fabs(signedCurrent_A);
    sensorPower = ina219.getPower_mW() / 1000.0;
    sensorSOC = calculateSOC(sensorVoltage, signedCurrent_A);
}

void saveSOCToNVS()
{
    if (socEstimate < 0.0) return;
    prefs.begin("crux", false);
    prefs.putFloat("soc", socEstimate);
    if (rtcAvailable && rtcTimeValid) prefs.putUInt("ts", (uint32_t)rtc.now().unixtime());
    prefs.end();
}

void restoreSOCFromNVS()
{
    prefs.begin("crux", true);
    float savedSOC = prefs.getFloat("soc", -1.0);
    uint32_t savedTS = prefs.getUInt("ts", 0);
    prefs.end();

    if (savedSOC < 0.0 || savedSOC > 100.0)
    {
        LOG("[NVS] No saved SOC; seed from voltage");
        return;
    }

    // v9-style restore guard against unrealistic jumps after reboot.
    float shunt_mV = ina219.getShuntVoltage_mV();
    float bus_V = ina219.getBusVoltage_V();
    float liveVoltage = bus_V + (shunt_mV / 1000.0f);
    if (liveVoltage >= 3.0f)
    {
        float liveSOC = voltageToSOC(liveVoltage, 0.0f);
        float diff = fabs(savedSOC - liveSOC);
        LOGF("[NVS] Saved=%.1f%% Live=%.1f%% Diff=%.1f%%\n", savedSOC, liveSOC, diff);
        if (diff > SOC_REBOOT_MAX_DEVIATION)
        {
            LOG("[NVS] Saved SOC discarded (deviation too high)");
            return;
        }
    }

    socEstimate = savedSOC;
    lastCoulombUnix = savedTS;
    lastCoulombMs = millis();
    LOGF("[NVS] Restored SOC: %.1f%%\n", socEstimate);
}

void powerOnModem()
{
    pinMode(MODEM_POWER, OUTPUT);
    digitalWrite(MODEM_POWER, LOW); delay(100);
    digitalWrite(MODEM_POWER, HIGH); delay(1000);
    digitalWrite(MODEM_POWER, LOW); delay(5000);
}

void initModem()
{
    SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    LOG("[MODEM] Initializing...");
    if (!modem.init()) LOG("[MODEM] init() failed");
    if (!modem.restart()) LOG("[MODEM] restart() failed");
    String name = modem.getModemName();
    String info = modem.getModemInfo();
    LOG("[MODEM] Name: " + name);
    LOG("[MODEM] Info: " + info);
    LOG("[MODEM] Ready");
}

void waitForNetwork()
{
    LOG("[NET] Scanning for 4G network...");
    if (!modem.waitForNetwork(NETWORK_TIMEOUT_MS, true))
    {
        LOG("[NET] No network - rebooting");
        delay(30000);
        ESP.restart();
    }
    LOGF("[NET] Registered (signal: %d/31)\n", modem.getSignalQuality());
}

void connectGPRS()
{
    LOG("[NET] Connecting GPRS...");
    if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS))
    {
        delay(10000);
        if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS))
        {
            LOG("[NET] GPRS failed - rebooting");
            delay(5000);
            ESP.restart();
        }
    }
    LOG("[NET] GPRS connected — IP: " + modem.getLocalIP());
}

void ensureConnected()
{
    if (!modem.isNetworkConnected()) { waitForNetwork(); connectGPRS(); }
    if (!modem.isGprsConnected()) connectGPRS();
}

void registerWithRetry()
{
    LOG("[REG] Registering device...");
#if CRUX_USE_JSONDOC_V6
    StaticJsonDocument<JSON_DOC_CAPACITY> doc;
#else
    JsonDocument doc;
#endif
    doc["deviceId"] = DEVICE_ID;
    doc["deviceSecret"] = DEVICE_SECRET;
    doc["deviceName"] = DEVICE_NAME;
    doc["firmwareVersion"] = FIRMWARE_VER;
    char jsonBuf[JSON_SERIAL_BUFFER];
    size_t jsonLen = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (jsonLen == 0 || jsonLen >= sizeof(jsonBuf))
    {
        LOG("[REG] JSON serialize error or buffer full");
        return;
    }
    for (int i = 1; i <= 5; i++)
    {
        int code = httpPost(REGISTER_PATH, jsonBuf, jsonLen, nullptr, nullptr);
        if (code == 200 || code == 201)
        {
            registered = true;
            LOG("[REG] Registered\n");
            return;
        }
        LOGF("[REG] Attempt %d failed (HTTP %d)\n", i, code);
        delay(REGISTER_RETRY_MS);
    }
    LOG("[REG] Will retry in main loop\n");
}

bool sendSensorData()
{
    LOG("[DATA] Sending...");
#if CRUX_USE_JSONDOC_V6
    StaticJsonDocument<JSON_DOC_CAPACITY> doc;
#else
    JsonDocument doc;
#endif
    doc["temperature"] = round1(sensorTemp);
    doc["voltage"] = round2(sensorVoltage);
    doc["power"] = round2(sensorPower);
    doc["current"] = round3(sensorCurrent);
    doc["soc"] = round1(sensorSOC);
    char jsonBuf[JSON_SERIAL_BUFFER];
    size_t jsonLen = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (jsonLen == 0 || jsonLen >= sizeof(jsonBuf))
    {
        LOG("[DATA] JSON serialize error");
        return false;
    }
    LOGF("[DATA] Payload: %s\n", jsonBuf);
    if (rtcAvailable && rtcTimeValid)
    {
        DateTime t = rtc.now();
        LOGF("[RTC] Local: %04d-%02d-%02d %02d:%02d:%02d (server recordedAt is UTC)\n",
             t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
    }
    int code = httpPost(DATA_PATH, jsonBuf, jsonLen, DEVICE_ID, DEVICE_SECRET);
    if (code == 201)
    {
        LOG("[DATA] 201\n");
        return true;
    }
    LOGF("[DATA] HTTP %d\n", code);
    if (code == 404)
        registered = false;
    return false;
}

static int httpPost(const char *path, const char *body, size_t bodyLen,
                    const char *devId, const char *devSecret)
{
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++)
    {
        if (attempt > 1)
        {
            LOGF("[HTTP] Retry %d/%d\n", attempt, MAX_RETRIES);
            delay(RETRY_BACKOFF_MS * attempt);
        }
        LOGF("[HTTP] Connect %s:%d\n", SERVER_HOST, SERVER_PORT);
        if (!netClient.connect(SERVER_HOST, SERVER_PORT))
        {
            LOG("[HTTP] TCP failed");
            netClient.stop();
            continue;
        }
        char hdrs[HTTP_HEADER_BUFFER];
        int h = snprintf(
            hdrs, sizeof(hdrs),
            "POST %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %u\r\n"
            "Connection: close\r\n",
            path, SERVER_HOST, (unsigned)bodyLen);
        if (h <= 0 || (size_t)h >= sizeof(hdrs) - 96)
        {
            LOG("[HTTP] header base truncated");
            netClient.stop();
            continue;
        }
        int pos = h;
        if (devId && devSecret)
        {
            int a = snprintf(hdrs + pos, sizeof(hdrs) - (size_t)pos,
                             "x-device-id: %s\r\nx-device-secret: %s\r\n",
                             devId, devSecret);
            if (a <= 0 || pos + a >= (int)sizeof(hdrs) - 8)
            {
                LOG("[HTTP] auth header truncated");
                netClient.stop();
                continue;
            }
            pos += a;
        }
        int fin = snprintf(hdrs + pos, sizeof(hdrs) - (size_t)pos, "\r\n");
        if (fin < 2)
        {
            netClient.stop();
            continue;
        }
        pos += fin;
        netClient.write((const uint8_t *)hdrs, (size_t)pos);
        netClient.write((const uint8_t *)body, bodyLen);

        uint32_t t0 = millis();
        while (!netClient.available() && millis() - t0 < HTTP_TIMEOUT_MS) delay(50);
        if (!netClient.available())
        {
            LOG("[HTTP] timeout");
            netClient.stop();
            continue;
        }
        String statusLine = netClient.readStringUntil('\n');
        int statusCode = -1;
        if (statusLine.length() > 12)
            statusCode = statusLine.substring(9, 12).toInt();
        LOGF("[HTTP] <- %d\n", statusCode);
        while (netClient.available())
        {
            String line = netClient.readStringUntil('\n');
            if (line.length() <= 1) break;
        }
        char respBuf[HTTP_RESP_BODY_MAX];
        size_t respN = 0;
        uint32_t readStart = millis();
        while (netClient.available() && millis() - readStart < 3000 && respN < sizeof(respBuf) - 1)
            respBuf[respN++] = (char)netClient.read();
        respBuf[respN] = '\0';
        if (respN > 0)
            LOGF("[HTTP] Body: %s\n", respBuf);
        netClient.stop();
        if (statusCode >= 200 && statusCode < 300) return statusCode;
        if (statusCode >= 400 && statusCode < 500 && statusCode != 408 && statusCode != 429) return statusCode;
    }
    LOG("[HTTP] all retries failed");
    return -1;
}

float round1(float v) { return ((int)(v * 10 + 0.5f)) / 10.0f; }
float round2(float v) { return ((int)(v * 100 + 0.5f)) / 100.0f; }
float round3(float v) { return ((int)(v * 1000 + 0.5f)) / 1000.0f; }
