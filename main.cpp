// ============================================================
//  SENDER — ESP32 #1
//  รับคำสั่งจาก Firebase (Web UI) → คำนวณ fanSpeed → ส่ง ESP-NOW
//
//  Firebase Path: /control/
//    mode        : "auto" | "manual"
//    power       : 0 | 1
//    speed       : 0–3  (manual mode: 0=0%, 1=50%, 2=75%, 3=100%)
//    temperature : float  (min temp threshold — auto mode, ต่ำกว่านี้ = off)
//    humidity    : int    (min humidity threshold — auto mode, ต่ำกว่านี้ = off)
//    fanSelect   : int    (1–10 เลือก fan ตัวที่จะส่ง)
//    timer/time_str     : "HH:MM"
//    timer/total_seconds: int (seconds)
//    direction   : "left" | "right"  (push-button trigger)
//    loopMode    : 0 | 1  (servo sweep on/off)
//    syncTime    : "HH:MM:SS"  (web → RTC sync)
//
//  ESP-NOW Packet (struct_message):
//    fanSpeed    : float  (0–100 %)
//    temperature : float  (sensor reading)
//    humidity    : float  (sensor reading)
//    mode        : char[8] ("auto" | "manual")
//    power       : int    (0 | 1)
//    dirTrigger  : int    (0=none, 1=left, 2=right)  — pulse then reset
//    currentTime : char[6] ("HH:MM")
//    fanSelect   : int    (1–10)
//    loopMode    : int    (0=stop sweep, 1=sweep)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <FirebaseESP32.h>
#include "Adafruit_SHT31.h"
#include <Fuzzy.h>
#include <LCD_I2C.h>
#include "RTClib.h"

// ---------------------------------------------------
// 1. Wi-Fi & Firebase
// ---------------------------------------------------
#define WIFI_SSID     "netGu"
#define WIFI_PASSWORD "12345678"
#define FIREBASE_HOST "global-html-test1-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "eVrDZYRfVIuHDMeGZPSru0qEGu4hRg9y7BgdMr9R"

// ---------------------------------------------------
// 2. ESP-NOW — MAC ของ Receiver (Fan 1–10)
// ---------------------------------------------------
#define FAN_COUNT 10
uint8_t fanMAC[FAN_COUNT][6] = {
    {0xCC, 0x7B, 0x5C, 0x28, 0x36, 0xC0}, // Fan 1
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02}, // Fan 2
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03}, // Fan 3
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x04}, // Fan 4
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x05}, // Fan 5
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x06}, // Fan 6
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x07}, // Fan 7
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x08}, // Fan 8
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x09}, // Fan 9
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x0A}, // Fan 10
};

// ---------------------------------------------------
// 3. Hardware Objects
// ---------------------------------------------------
LCD_I2C          lcd(0x26, 16, 2);
Adafruit_SHT31   sht30 = Adafruit_SHT31();
RTC_DS1307       rtc;
Fuzzy           *fuzzy = new Fuzzy();

// ---------------------------------------------------
// 4. Firebase Objects
// ---------------------------------------------------
FirebaseData fbdo;   // stream
FirebaseData fbGet;  // one-time get
FirebaseAuth auth;
FirebaseConfig config;

// ---------------------------------------------------
// 5. ESP-NOW Struct
// ---------------------------------------------------
typedef struct struct_message {
    float  fanSpeed;          // 0–100 %
    float  temperature;       // °C  (sensor)
    float  humidity;          // %   (sensor)
    char   mode[8];           // "auto" | "manual"
    int    power;             // 0 | 1
    int    dirTrigger;        // 0=none, 1=left, 2=right
    char   currentTime[6];    // "HH:MM"
    int    fanSelect;          // 1–10 fan ที่เลือก
    int    loopMode;           // 0=stop sweep, 1=sweep
    int    underLimit;         // 1 = sensor ต่ำกว่า threshold (fan off แต่ power ยังเปิด)
} struct_message;

struct_message      txData;

// ---------------------------------------------------
// 6. State Variables (mirroring Firebase)
// ---------------------------------------------------
String  g_mode       = "auto";
int     g_power      = 0;
int     g_speed      = 0;
float   g_minTemp    = -999.0f;
int     g_minHum     = -1;
int     g_fanSelect  = 1;
String  g_timeStr    = "";
int     g_totalSec   = -1;
String  g_direction  = "";

// Timer countdown
unsigned long g_timerStartMs  = 0;
bool          g_timerRunning  = false;

// Direction pulse (send once then reset)
int           g_dirTrigger    = 0;
String        g_lastSentDir   = "";

String espStatus = "Wait";
int     g_loopMode   = 1;
String  g_syncTime   = "";

// ---------------------------------------------------
// 7. Manual speed index → % conversion
// ---------------------------------------------------
float speedIndexToPercent(int idx) {
    switch (idx) {
        case 1: return 50.0f;
        case 2: return 75.0f;
        case 3: return 100.0f;
        default: return 0.0f;
    }
}

// ---------------------------------------------------
// 8. ESP-NOW Send Callback
// ---------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    espStatus = (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "ERR";
    Serial.printf("[ESP-NOW] Send: %s\n", espStatus.c_str());
}

// ---------------------------------------------------
// 9. Firebase Initial Fetch
// ---------------------------------------------------
void fetchAllValues() {
    Serial.println("[INIT] อ่านค่าเริ่มต้นจาก Firebase...");

    if (Firebase.getString(fbGet, "/control/mode"))
        g_mode = fbGet.stringData();

    if (Firebase.getInt(fbGet, "/control/power"))
        g_power = fbGet.intData();

    if (Firebase.getInt(fbGet, "/control/speed"))
        g_speed = fbGet.intData();

    if (Firebase.getFloat(fbGet, "/control/temperature"))
        g_minTemp = fbGet.floatData();
    else
        g_minTemp = -999.0f;

    if (Firebase.getInt(fbGet, "/control/humidity"))
        g_minHum = fbGet.intData();
    else
        g_minHum = -1;

    if (Firebase.getInt(fbGet, "/control/fanSelect"))
        g_fanSelect = constrain(fbGet.intData(), 1, 10);
    else
        g_fanSelect = 1;

    if (Firebase.getString(fbGet, "/control/timer/time_str")) {
        g_timeStr = fbGet.stringData();
        g_timerRunning = (g_timeStr.length() == 5);
        if (g_timerRunning)
            Serial.printf("[INIT] timer scheduled at %s\n", g_timeStr.c_str());
    }

    if (Firebase.getString(fbGet, "/control/direction"))
        g_direction = fbGet.stringData();

    if (Firebase.getInt(fbGet, "/control/loopMode"))
        g_loopMode = fbGet.intData();

    if (Firebase.getString(fbGet, "/control/syncTime")) {
        g_syncTime = fbGet.stringData();
        if (g_syncTime.length() >= 5) {
            int sh = g_syncTime.substring(0,2).toInt();
            int sm = g_syncTime.substring(3,5).toInt();
            int ss = (g_syncTime.length() >= 8) ? g_syncTime.substring(6,8).toInt() : 0;
            DateTime now = rtc.now();
            rtc.adjust(DateTime(now.year(), now.month(), now.day(), sh, sm, ss));
            Serial.printf("[RTC] Synced to %02d:%02d:%02d\n", sh, sm, ss);
            Firebase.setString(fbGet, "/control/syncTime", "");
        }
    }

    Serial.printf("[INIT] mode=%s power=%d speed=%d minT=%.1f minH=%d timer=%ds fan=%d loop=%d\n",
                  g_mode.c_str(), g_power, g_speed, g_minTemp, g_minHum, g_totalSec, g_fanSelect, g_loopMode);
}

// ---------------------------------------------------
// 10. Firebase Stream Handler
// ---------------------------------------------------
void handleStream() {
    if (!Firebase.readStream(fbdo)) return;
    if (!fbdo.streamAvailable())   return;

    String dpath = fbdo.dataPath();
    String dtype = fbdo.dataType();

    Serial.printf("[STREAM] %s  (%s)\n", dpath.c_str(), dtype.c_str());

    if (dpath == "/mode" && dtype == "string") {
        g_mode = fbdo.stringData();
        Serial.printf("  → mode = %s\n", g_mode.c_str());
    }
    else if (dpath == "/power" && dtype == "int") {
        g_power = fbdo.intData();
        Serial.printf("  → power = %d\n", g_power);
    }
    else if (dpath == "/speed" && dtype == "int") {
        int v = fbdo.intData();
        g_speed = constrain(v, 0, 3);
        Serial.printf("  → speed = %d (%.0f%%)\n", g_speed, speedIndexToPercent(g_speed));
    }
    else if (dpath == "/temperature") {
        if (dtype == "null") { g_minTemp = -999.0f; }
        else if (dtype == "float") g_minTemp = fbdo.floatData();
        else if (dtype == "int")   g_minTemp = (float)fbdo.intData();
        Serial.printf("  → minTemp = %.1f\n", g_minTemp);
    }
    else if (dpath == "/humidity") {
        if (dtype == "null")      { g_minHum = -1; }
        else if (dtype == "int")  { g_minHum = constrain(fbdo.intData(), 0, 100); }
        Serial.printf("  → minHum = %d\n", g_minHum);
    }
    else if (dpath == "/fanSelect" && dtype == "int") {
        g_fanSelect = constrain(fbdo.intData(), 1, 10);
        Serial.printf("  → fanSelect = %d\n", g_fanSelect);
    }
    else if (dpath == "/timer" && dtype == "json") {
        // UI เขียน timer ทั้งก้อน { time_str, total_seconds } → parse time_str จาก JSON
        String raw = fbdo.jsonString();
        int idx = raw.indexOf("\"time_str\":\"");
        if (idx >= 0) {
            String ts = raw.substring(idx + 12, idx + 17); // "HH:MM"
            if (ts.length() == 5 && ts[2] == ':') {
                g_timeStr = ts;
                g_timerRunning = true;
                Serial.printf("  → timer scheduled at %s (from json)\n", g_timeStr.c_str());
            }
        }
    }
    else if (dpath == "/timer/time_str" && dtype == "string") {
        g_timeStr = fbdo.stringData();
        g_timerRunning = (g_timeStr.length() == 5);
        if (g_timerRunning)
            Serial.printf("  → timer scheduled at %s\n", g_timeStr.c_str());
        else
            Serial.println("  → timer cleared");
    }
    else if (dpath == "/timer/total_seconds") {
        // ไม่ใช้ total_seconds แล้ว — timer ใช้ time_str เทียบกับ RTC
    }
    else if (dpath == "/timer" && dtype == "null") {
        g_timeStr = ""; g_totalSec = -1; g_timerRunning = false;
    }
    else if (dpath == "/direction" && dtype == "string") {
        String newDir = fbdo.stringData();
        if (newDir == "") {
            g_lastSentDir = "";
        } else if (newDir == "left" || newDir == "right") {
            g_dirTrigger = (newDir == "left") ? 1 : 2;
            g_direction  = newDir;
            Serial.printf("  → direction trigger: %s\n", newDir.c_str());
        }
    }
    else if (dpath == "/loopMode" && dtype == "int") {
        g_loopMode = fbdo.intData();
        Serial.printf("  → loopMode = %d\n", g_loopMode);
    }
    else if (dpath == "/syncTime" && dtype == "string") {
        String st = fbdo.stringData();
        if (st.length() >= 5) {
            int sh = st.substring(0,2).toInt();
            int sm = st.substring(3,5).toInt();
            int ss = (st.length() >= 8) ? st.substring(6,8).toInt() : 0;
            DateTime now = rtc.now();
            rtc.adjust(DateTime(now.year(), now.month(), now.day(), sh, sm, ss));
            Serial.printf("[RTC] Synced to %02d:%02d:%02d\n", sh, sm, ss);
            Firebase.setString(fbGet, "/control/syncTime", "");
        }
    }
    else if (dpath == "/" && dtype == "json") {
        fetchAllValues();
    }
}

// ---------------------------------------------------
// 11. Fuzzy Logic Setup
// ---------------------------------------------------
// ตารางกฎ (จากสเปค):
//   T < 22                → ทุก H          → Level 0 (OFF)
//   22–25 , H 40–60       →                → Level 1
//   22–25 , H > 65        →                → Level 2
//   25–28 , H 40–60       →                → Level 2
//   T > 28                → ทุก H          → Level 3
//   ทุก T , H > 80        →                → Level 3
//
// Temperature sets:
//   tLow     : 0–22        (< 22)
//   tMidLow  : 22–25       (22–25)
//   tMidHigh : 25–28       (25–28)
//   tHigh    : > 28
//
// Humidity sets:
//   hLow     : < 40        (ต่ำ)
//   hMid     : 40–65       (ปานกลาง)
//   hHigh    : 65–80       (สูง)
//   hVHigh   : > 80        (สูงมาก)
//
// Output (% PWM ตรงกับ Level 0/1/2/3):
//   fOff  =   0%   (Level 0)
//   fLow  =  50%   (Level 1)
//   fMid  =  75%   (Level 2)
//   fHigh = 100%   (Level 3)
// ---------------------------------------------------
void setupFuzzy() {
    // --- Input 1: Temperature ---
    FuzzyInput *temp = new FuzzyInput(1);
    //                          a    b    c    d
    FuzzySet *tLow     = new FuzzySet(  0,   0,  20,  22); // < 22°C
    FuzzySet *tMidLow  = new FuzzySet( 20,  22,  25,  27); // 22–25°C
    FuzzySet *tMidHigh = new FuzzySet( 25,  27,  28,  30); // 25–28°C
    FuzzySet *tHigh    = new FuzzySet( 28,  30, 100, 100); // > 28°C
    temp->addFuzzySet(tLow);
    temp->addFuzzySet(tMidLow);
    temp->addFuzzySet(tMidHigh);
    temp->addFuzzySet(tHigh);
    fuzzy->addFuzzyInput(temp);

    // --- Input 2: Humidity ---
    FuzzyInput *hum = new FuzzyInput(2);
    FuzzySet *hLow   = new FuzzySet(  0,   0,  38,  42); // < 40%
    FuzzySet *hMid   = new FuzzySet( 38,  42,  63,  67); // 40–65%
    FuzzySet *hHigh  = new FuzzySet( 63,  67,  78,  82); // 65–80%
    FuzzySet *hVHigh = new FuzzySet( 78,  82, 100, 100); // > 80%
    hum->addFuzzySet(hLow);
    hum->addFuzzySet(hMid);
    hum->addFuzzySet(hHigh);
    hum->addFuzzySet(hVHigh);
    fuzzy->addFuzzyInput(hum);

    // --- Output: Fan Level ---
    FuzzyOutput *fan = new FuzzyOutput(1);
    FuzzySet *fOff  = new FuzzySet(  0,   0,   0,   0); // Level 0 = OFF
    FuzzySet *fLow  = new FuzzySet( 45,  50,  50,  55); // Level 1 = 50%
    FuzzySet *fMid  = new FuzzySet( 70,  75,  75,  80); // Level 2 = 75%
    FuzzySet *fHigh = new FuzzySet( 95, 100, 100, 100); // Level 3 = 100%
    fan->addFuzzySet(fOff);
    fan->addFuzzySet(fLow);
    fan->addFuzzySet(fMid);
    fan->addFuzzySet(fHigh);
    fuzzy->addFuzzyOutput(fan);

    // --- Rules ---
    auto addRule = [&](int id, FuzzySet *t, FuzzySet *h, FuzzySet *f) {
        FuzzyRuleAntecedent *ant = new FuzzyRuleAntecedent();
        ant->joinWithAND(t, h);
        FuzzyRuleConsequent *con = new FuzzyRuleConsequent();
        con->addOutput(f);
        fuzzy->addFuzzyRule(new FuzzyRule(id, ant, con));
    };

    // T < 22 → Level 0 (ทุก humidity)
    addRule(1, tLow, hLow,   fOff);
    addRule(2, tLow, hMid,   fOff);
    addRule(3, tLow, hHigh,  fOff);
    addRule(4, tLow, hVHigh, fOff);

    // 22–25, H 40–65 → Level 1
    addRule(5, tMidLow, hLow,  fOff);  // H < 40 ยังถือว่า off
    addRule(6, tMidLow, hMid,  fLow);  // H 40–65 → Level 1
    addRule(7, tMidLow, hHigh, fMid);  // H 65–80 → Level 2 (> 65)
    addRule(8, tMidLow, hVHigh, fHigh);// H > 80  → Level 3

    // 25–28, H 40–65 → Level 2
    addRule(9,  tMidHigh, hLow,   fOff);  // H < 40 ยังถือว่า off
    addRule(10, tMidHigh, hMid,   fMid);  // H 40–65 → Level 2
    addRule(11, tMidHigh, hHigh,  fMid);  // H 65–80 → Level 2
    addRule(12, tMidHigh, hVHigh, fHigh); // H > 80  → Level 3

    // T > 28 → Level 3 (ทุก humidity)
    addRule(13, tHigh, hLow,   fHigh);
    addRule(14, tHigh, hMid,   fHigh);
    addRule(15, tHigh, hHigh,  fHigh);
    addRule(16, tHigh, hVHigh, fHigh);
}

// ---------------------------------------------------
// 12. Setup
// ---------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);

    lcd.begin();
    lcd.backlight();
    lcd.print("System Init...");

    if (!rtc.begin()) {
        Serial.println("[WARN] RTC not found");
    } else {
        // >>> ตั้งเวลาใหม่ 1 ครั้ง แล้ว comment บรรทัด SET_RTC_TIME ออก แล้วอัปโหลดใหม่ <<<
        // SET_RTC_TIME:
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // <<< comment ถึงตรงนี้ >>>
        Serial.printf("[RTC] OK — %s\n", rtc.now().timestamp().c_str());
    }

    if (!sht30.begin(0x44)) Serial.println("[WARN] SHT30 not found");

    setupFuzzy();

    WiFi.mode(WIFI_STA);
    Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.printf("\n[WiFi] Connected IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] Channel: %d  <-- ใส่เลขนี้ใน ESPNOW_CHANNEL ของ receiver\n", WiFi.channel());
    Serial.printf("[WiFi] Channel: %d  <-- ใส่ตัวเลขนี้ใน ESPNOW_CHANNEL ของ Receiver\n", WiFi.channel());
    Serial.printf("[WiFi] *** Channel: %d *** (ใส่เลขนี้ใน ESPNOW_CHANNEL ของ Receiver)\n", WiFi.channel());

    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    fetchAllValues();
    if (!Firebase.beginStream(fbdo, "/control"))
        Serial.printf("[Error] Stream: %s\n", fbdo.errorReason().c_str());
    else
        Serial.println("[Firebase] Stream ready at /control");

    if (esp_now_init() != ESP_OK) {
        Serial.println("[Error] ESP-NOW init failed");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    for (int i = 0; i < FAN_COUNT; i++) {
        esp_now_peer_info_t peer;
        memset(&peer, 0, sizeof(peer));
        memcpy(peer.peer_addr, fanMAC[i], 6);
        peer.channel = WiFi.channel();
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) == ESP_OK)
            Serial.printf("[ESP-NOW] Fan %d peer added\n", i + 1);
        else
            Serial.printf("[ESP-NOW] Fan %d peer FAILED\n", i + 1);
    }

    lcd.clear();
    Serial.println("[OK] Setup complete");
}

// ---------------------------------------------------
// 13. Loop
// ---------------------------------------------------
void loop() {
    // 13-A: อ่าน Firebase stream
    handleStream();

    // 13-B: ตรวจสอบ Timer (Schedule — ปิดเมื่อถึงหรือเลยเวลาที่กำหนด เช่น "19:20")
    if (g_timerRunning && g_timeStr.length() == 5) {
        DateTime nowT = rtc.now();
        int tgtH  = g_timeStr.substring(0, 2).toInt();
        int tgtM  = g_timeStr.substring(3, 5).toInt();
        int nowMin = nowT.hour() * 60 + nowT.minute();
        int tgtMin = tgtH * 60 + tgtM;
        // ใช้ >= เพื่อไม่พลาดถ้า loop ช้าและข้ามนาทีนั้นไป
        if (nowMin >= tgtMin) {
            Serial.printf("[TIMER] ถึงเวลา %s (now %02d:%02d) → Power OFF\n",
                          g_timeStr.c_str(), nowT.hour(), nowT.minute());
            g_power        = 0;
            g_timerRunning = false;
            g_timeStr      = "";
            Firebase.setInt(fbGet, "/control/power", 0);
            Firebase.setString(fbGet, "/control/timer/time_str", "");
        }
    }

    // 13-C: อ่าน Sensor
    float t = sht30.readTemperature();
    float h = sht30.readHumidity();
    if (isnan(t)) t = 0.0f;
    if (isnan(h)) h = 0.0f;

    // 13-D: คำนวณ fanSpeed
    float fanSpeed = 0.0f;
    String statusStr = "OFF";

    bool isUnderLimit = false;  // tracking สำหรับส่งไป receiver

    if (g_power == 0) {
        fanSpeed  = 0.0f;
        statusStr = "PWR-OFF";
    } else {
        // ตรวจ underLimit ก่อน — ใช้ได้ทั้ง auto และ manual
        bool underLimit = false;
        if (g_minTemp > -998.0f && t < g_minTemp) underLimit = true;
        if (g_minHum  >= 0      && (int)h < g_minHum) underLimit = true;

        if (underLimit) {
            fanSpeed     = 0.0f;
            statusStr    = "UND-LIM";
            isUnderLimit = true;
        } else if (g_mode == "auto") {
            float tInput = (g_minTemp > -998.0f) ? max(t, g_minTemp) : t;
            float hInput = (g_minHum  >= 0)      ? max(h, (float)g_minHum) : h;

            fuzzy->setInput(1, tInput);
            fuzzy->setInput(2, hInput);
            fuzzy->fuzzify();
            fanSpeed  = fuzzy->defuzzify(1);
            statusStr = (fanSpeed > 75) ? "AUTO-H" :
                        (fanSpeed > 50) ? "AUTO-M" : "AUTO-L";
        } else {
            // manual mode
            fanSpeed  = speedIndexToPercent(g_speed);
            statusStr = (g_speed == 0) ? "MAN-0" :
                        (g_speed == 1) ? "MAN-L" :
                        (g_speed == 2) ? "MAN-M" : "MAN-H";
        }
    }

    // 13-E: รับเวลาปัจจุบันจาก RTC
    DateTime now = rtc.now();
    char timeStr[6];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d", now.hour(), now.minute());

    static unsigned long lastRTCWrite = 0;
    if (millis() - lastRTCWrite > 30000UL) {
        lastRTCWrite = millis();
        char fullTime[9];
        snprintf(fullTime, sizeof(fullTime), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        Firebase.setString(fbGet, "/status/rtcTime", fullTime);
    }

    static unsigned long lastSensorWrite = 0;
    if (millis() - lastSensorWrite > 10000UL) {
        lastSensorWrite = millis();
        Firebase.setFloat(fbGet, "/status/temperature", t);
        Firebase.setFloat(fbGet, "/status/humidity",    h);
    }

    // 13-F: เตรียม Packet
    static unsigned long lastFanSpeedSend = 0;
    static float         lastSentFanSpeed = -1.0f;
    static unsigned long lastSend         = 0;

    txData.temperature = t;
    txData.humidity    = h;
    strncpy(txData.mode, g_mode.c_str(), sizeof(txData.mode) - 1);
    txData.mode[sizeof(txData.mode)-1] = '\0';
    txData.power       = g_power;
    txData.dirTrigger  = g_dirTrigger;
    strncpy(txData.currentTime, timeStr, sizeof(txData.currentTime));
    txData.fanSelect   = g_fanSelect;
    txData.loopMode    = g_loopMode;
    txData.underLimit  = isUnderLimit ? 1 : 0;  // ← ใหม่

    // ✅ FIX S1: เพิ่ม fanSpeedChanged เข้าเงื่อนไข — ส่งทันทีเมื่อ speed เปลี่ยน
    bool fanSpeedChanged = (fanSpeed != lastSentFanSpeed);
    bool fanSpeedDue     = (millis() - lastFanSpeedSend >= 5000UL);
    bool forceSend       = (g_power == 0 && lastSentFanSpeed != 0.0f);

    // ✅ FIX S3: ถ้า sensor อ่านไม่ได้ (t==0 && h==0) และ lastSentFanSpeed < 0
    //    → ป้องกันส่ง fanSpeed=0 รอบแรก โดย skip ถ้า sensor ยังไม่พร้อม
    bool sensorReady = !(t == 0.0f && h == 0.0f && g_mode == "auto");

    if ((fanSpeedDue || forceSend || fanSpeedChanged || lastSentFanSpeed < 0) && sensorReady) {
        txData.fanSpeed   = fanSpeed;
        lastSentFanSpeed  = fanSpeed;
        lastFanSpeedSend  = millis();
    } else if (lastSentFanSpeed < 0) {
        // sensor ยังไม่พร้อมแต่รอบแรก — ใช้ค่า 0 ไปก่อน (ปลอดภัยกว่า garbage)
        txData.fanSpeed = 0.0f;
    }

    // ส่ง packet ทุก 500ms
    int fanIdx = constrain(g_fanSelect - 1, 0, FAN_COUNT - 1);
    if (millis() - lastSend >= 500UL) {
        lastSend = millis();
        esp_now_send(fanMAC[fanIdx], (uint8_t *)&txData, sizeof(txData));

        // รีเซ็ต direction trigger หลังส่งไปแล้ว 1 รอบ
        if (g_dirTrigger != 0) {
            g_lastSentDir = g_direction;
            g_dirTrigger  = 0;
        }
    }

    // 13-G: แสดงผล LCD
    lcd.setCursor(0, 0);
    lcd.print(timeStr);
    lcd.setCursor(7, 0);
    lcd.print("ESP:");
    lcd.print(espStatus);
    if (espStatus == "OK") lcd.print(" ");

    lcd.setCursor(0, 1);
    lcd.print((int)t); lcd.print("C ");
    lcd.print((int)h); lcd.print("% ");
    lcd.print("F:"); lcd.print((int)fanSpeed); lcd.print("% ");
    lcd.print(statusStr.substring(0, 3));

    // 13-H: Serial Monitor
    Serial.printf("[%s] mode=%s pwr=%d fan=%.0f%% T=%.1f H=%.0f dir=%d fanSel=%d loop=%d ESP=%s\n",
                  timeStr, g_mode.c_str(), g_power,
                  fanSpeed, t, h,
                  txData.dirTrigger, g_fanSelect, g_loopMode, espStatus.c_str());

    delay(50);
}