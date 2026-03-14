#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include "esp_wifi.h"

// ============================================================
//  การตั้งค่าขา I/O
// ============================================================
#define FAN_PIN         27
#define SERVO_PIN       22
#define PIR_PIN         23

// ============================================================
//  PWM พัดลม
// ============================================================
#define FAN_FREQ        200
#define FAN_RES         8

// ============================================================
//  Servo
// ============================================================
#define SERVO_MIN_DEG    0
#define SERVO_MAX_DEG    180
#define SERVO_STEP_DEG   15
#define SERVO_SWEEP_MS   20
#define SERVO_MANUAL_MS  80

// ============================================================
//  Fan ramp
// ============================================================
#define RAMP_STEP        1.0f
#define RAMP_INTERVAL    20

// ============================================================
//  Timeouts
// ============================================================
#define NO_PERSON_TIMEOUT_MS  (5UL * 60UL * 1000UL) // 5 นาที
#define NO_PACKET_TIMEOUT_MS  (5UL * 60UL * 1000UL) // 5 นาที

// ============================================================
//  ตัวแปรเก็บ Channel ที่สแกนเจอ
// ============================================================
int currentChannel = 0;

// ============================================================
//  Struct — ต้องตรงกับ Sender
// ============================================================
typedef struct struct_message {
    float fanSpeed;
    float temperature;
    float humidity;
    char  mode[8];
    int   power;
    int   dirTrigger;
    char  currentTime[6];
    int   fanSelect;
    int   loopMode;
    int   underLimit;   // 1 = sensor ต่ำกว่า threshold (fan off แต่ power ยังเปิด)
} struct_message;

struct_message rxData;
volatile bool  dataReceived = false;

// ============================================================
//  State
// ============================================================
Servo servo;
ESP32PWM fanPWM;

int   servoAngle   = 90;
bool  sweepForward = true;
bool  manualMoving = false;
int   manualTarget = 90;

unsigned long lastManualStep = 0;
unsigned long lastSweepStep  = 0;

// ============================================================
//  PIR & Sleep variables (แก้ไขเป็น volatile สำหรับ Interrupt)
// ============================================================
volatile bool  personFound    = false;
volatile unsigned long lastPersonTime   = 0;

bool  isAsleep    = false;
bool  loopEnabled = false;
String activeMode = "none";
int   lastDirTrigger = 0;
bool  packetEverReceived = false;

float targetFanPWM  = 0.0f;
float currentFanPWM = 0.0f;
unsigned long lastRamp = 0;

bool underLimitActive = false;  // ← ใหม่: true เมื่อ sensor ต่ำกว่า threshold

// packet stats
unsigned long packetCount   = 0;
unsigned long lastPacketMs  = 0;

// ============================================================
//  ฟังก์ชันสแกนหา Channel ของ Hotspot (netGu)
// ============================================================
int getWiFiChannel(const char *ssid) {
    if (int32_t n = WiFi.scanNetworks()) {
        for (int i = 0; i < n; i++) {
            if (WiFi.SSID(i) == ssid) {
                return WiFi.channel(i);
            }
        }
    }
    return 0;
}

// ============================================================
//  ESP-NOW callback
// ============================================================
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    dataReceived = true;
    packetCount++;
    lastPacketMs = millis();
}

// ============================================================
//  Interrupt Service Routine สำหรับ PIR Sensor
// ============================================================
void IRAM_ATTR pirInterruptHandler() {
    if (digitalRead(PIR_PIN) == HIGH) {
        personFound = true;
        lastPersonTime = millis();
    } else {
        personFound = false;
    }
}

// ============================================================
//  Fan ramp
// ============================================================
void setFanTarget(float pct) {
    targetFanPWM = constrain(pct / 100.0f * 255.0f, 0.0f, 255.0f);
}

void updateFanRamp() {
    if (millis() - lastRamp < RAMP_INTERVAL) return;
    lastRamp = millis();

    if (currentFanPWM < targetFanPWM)
        currentFanPWM = min(currentFanPWM + RAMP_STEP, targetFanPWM);
    else if (currentFanPWM > targetFanPWM)
        currentFanPWM = max(currentFanPWM - RAMP_STEP, targetFanPWM);
        
    fanPWM.write((int)currentFanPWM);
}

// ============================================================
//  Servo manual move
// ============================================================
void startManualMove(int targetDeg) {
    manualTarget   = constrain(targetDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    manualMoving   = true;
    lastManualStep = millis();
    Serial.printf("[MANUAL] target=%d from=%d\n", manualTarget, servoAngle);
}

void updateManualMove() {
    if (!manualMoving) return;
    if (millis() - lastManualStep < SERVO_MANUAL_MS) return;
    lastManualStep = millis();

    if      (servoAngle < manualTarget) servoAngle++;
    else if (servoAngle > manualTarget) servoAngle--;
    servo.write(servoAngle);

    if (servoAngle == manualTarget) {
        manualMoving = false;
        Serial.printf("[MANUAL] arrived=%d\n", servoAngle);
    }
}

// ============================================================
//  Sweep
// ============================================================
void updateSweep() {
    if (!loopEnabled) return;
    if (millis() - lastSweepStep < SERVO_SWEEP_MS) return;
    lastSweepStep = millis();
    
    if (sweepForward) { 
        servoAngle++; 
        if (servoAngle >= SERVO_MAX_DEG) { servoAngle = SERVO_MAX_DEG; sweepForward = false; } 
    } else { 
        servoAngle--;
        if (servoAngle <= SERVO_MIN_DEG) { servoAngle = SERVO_MIN_DEG; sweepForward = true; } 
    }
    servo.write(servoAngle);
}

void updateAutoSweep() {
    if (personFound) return; // ถ้าเจอคนให้หยุดหมุน
    if (millis() - lastSweepStep < SERVO_SWEEP_MS) return;
    lastSweepStep = millis();
    
    if (sweepForward) { 
        servoAngle++; 
        if (servoAngle >= SERVO_MAX_DEG) { servoAngle = SERVO_MAX_DEG; sweepForward = false; } 
    } else { 
        servoAngle--;
        if (servoAngle <= SERVO_MIN_DEG) { servoAngle = SERVO_MIN_DEG; sweepForward = true; } 
    }
    servo.write(servoAngle);
}

// ============================================================
//  Sleep
// ============================================================
void enterSleep() {
    if (isAsleep) return;
    isAsleep = true;
    targetFanPWM = currentFanPWM = 0;
    
    fanPWM.write(0);
    
    manualMoving = false;
    if (servo.attached()) servo.detach();
    WiFi.setSleep(true);
    Serial.println("[SLEEP] Entered sleep");
}

void exitSleep() {
    if (!isAsleep) return;
    isAsleep = false;
    WiFi.setSleep(false);
    if (!servo.attached()) servo.attach(SERVO_PIN);
    lastPersonTime = millis();
    Serial.println("[WAKE] Woke up");
}

// ============================================================
//  Status Dashboard
// ============================================================
unsigned long lastStatusPrint = 0;
void printStatus() {
    if (millis() - lastStatusPrint < 3000) return;
    lastStatusPrint = millis();

    float fanPct    = currentFanPWM / 255.0f * 100.0f;
    float targetPct = targetFanPWM  / 255.0f * 100.0f;
    unsigned long secSincePacket = lastPacketMs ? (millis() - lastPacketMs) / 1000UL : 9999;

    const char* pwrStr = isAsleep ? "OFF (Sleep)" : packetEverReceived ? "ON" : "ON (no packet yet)";

    char modeStr[36];
    if (!packetEverReceived)        snprintf(modeStr, sizeof(modeStr), "WAITING FOR PACKET...");
    else if (activeMode == "manual") snprintf(modeStr, sizeof(modeStr), "MANUAL | Loop %s", loopEnabled ? "ON" : "OFF");
    else if (activeMode == "auto")   snprintf(modeStr, sizeof(modeStr), "AUTO (PIR)");
    else                             snprintf(modeStr, sizeof(modeStr), "%s", activeMode.c_str());

    char servoStr[36];
    if (manualMoving)
        snprintf(servoStr, sizeof(servoStr), "%d -> %d deg (moving)", servoAngle, manualTarget);
    else if (activeMode == "auto" && personFound)
        snprintf(servoStr, sizeof(servoStr), "%d deg [HOLD - person]", servoAngle);
    else if ((activeMode == "auto" && !personFound) || (activeMode == "manual" && loopEnabled))
        snprintf(servoStr, sizeof(servoStr), "%d deg [sweep %s]", servoAngle, sweepForward ? "->" : "<-");
    else
        snprintf(servoStr, sizeof(servoStr), "%d deg [idle]", servoAngle);

    Serial.println("\n+----------------------------------------+");
    Serial.printf( "| MAC : %-32s |\n", WiFi.macAddress().c_str());
    Serial.printf( "| CH  : %-2d   Pkts: %-5lu   ago: %4lus    |\n", currentChannel, packetCount, secSincePacket);
    Serial.println("+----------------------------------------+");
    Serial.printf( "| PWR  : %-31s |\n", pwrStr);
    Serial.printf( "| MODE : %-31s |\n", modeStr);
    Serial.println("+----------------------------------------+");
    Serial.printf( "| FAN  : %5.1f%%  (PWM %3d/255)           |\n", fanPct, (int)currentFanPWM);
    if (fabsf(fanPct - targetPct) > 1.0f)
        Serial.printf("| RAMP : -> %.1f%%                          |\n", targetPct);
    Serial.printf( "| SERVO: %-31s |\n", servoStr);

    if (packetEverReceived) {
        Serial.println("+----------------------------------------+");
        Serial.printf