#pragma once
#include <Arduino.h>

// จำนวน Encoder สูงสุดที่สามารถลงทะเบียนใช้งานพร้อมกันแบบ Dynamic
constexpr uint8_t MAX_DYNAMIC_ENCODERS = 16;

class MotorEncoder
{
private:
    // =======================================================
    // 1. พารามิเตอร์ตั้งค่าภายในของแต่ละตัว (กำหนดผ่าน Constructor หรือ init ได้)
    // =======================================================
    uint8_t EncoderIN_A = 0;      // ขาสัญญาณ Encoder A (Interrupt)
    uint8_t EncoderIN_B = 0;      // ขาสัญญาณ Encoder B (เช็คทิศทาง)
    uint16_t ppr;                 // Pulse Per Revolution (PPR)
    uint8_t MaxCounter = 30;      // จำนวนรอบ (ms) ที่ไม่มี Pulse ก่อนตัด RPM เป็น 0
    uint32_t MinPulseUs = 30;     // เวลาขั้นต่ำระหว่าง Pulse (us) สำหรับกรอง Noise ใน ISR
    bool InvertDirection = false; // กลับทิศทาง (+ / -) โดยไม่ต้องสลับสาย

    // =======================================================
    // 2. ตัวแปรสถานะภายในสำหรับ ISR และการคำนวณ
    // =======================================================
    volatile uint32_t DeltaTime = 0;    // คาบเวลาระหว่าง Pulse (us)
    volatile uint32_t PrevTime = 0;     // เวลาของ Pulse ก่อนหน้า (us)
    volatile int8_t MotorDirection = 0; // ทิศทางการหมุน (+1 หรือ -1)
    volatile int32_t EncoderCount = 0;  // จำนวน Pulse สะสมทั้งหมด (ค่าดิบ)
    volatile int32_t PulseOffset = 0;   // ค่าชดเชยสำหรับเซ็ต Pulse ปัจจุบันให้เป็น 0 (หรือค่าที่ต้องการ)
    volatile uint32_t IsrCount = 0;     // จำนวนครั้งที่เข้า Interrupt ทั้งหมด (ก่อนกรอง Noise)
    volatile uint32_t FilteredCount = 0;// จำนวนครั้งที่ถูกตัดทิ้งเพราะ dt < MinPulseUs

    int32_t PrevEncoderCount = 0;
    uint8_t Counter = 0;
    int8_t slotIndex = -1; // ช่องตาราง Dynamic ISR ที่ลงทะเบียนไว้ (-1 = ยังไม่ได้ผูก)
    elapsedMillis debugtime;

public:
    volatile int16_t CurrentRPM = 0;
    volatile float CurrentAngle = 0.0f; // มุมปัจจุบัน (-180 ถึง +180 องศา)
    volatile float TotalAngle = 0.0f;   // มุมสะสมต่อเนื่อง (องศา)

    // ตารางเก็บ Instance สำหรับ Dynamic ISR และ updateAllRPM()
    static MotorEncoder *slots[MAX_DYNAMIC_ENCODERS];

    // Constructor: สร้างเปล่าๆ หรือกำหนดค่าเริ่มต้นตั้งแต่ตอนประกาศ Object ได้เลย
    MotorEncoder() = default;
    MotorEncoder(uint8_t encA,
                 uint8_t encB,
                 uint16_t motorPPR = 0,
                 uint8_t maxCounter = 30,
                 uint32_t minPulseUs = 30,
                 bool invertDirection = false);

    ~MotorEncoder();

    // เริ่มต้นใช้งานด้วยค่าที่ตั้งไว้ใน Constructor (ตั้ง pinMode + ผูก Interrupt + เริ่ม Timer อัตโนมัติ)
    bool init();

    // กำหนดค่าตัวแปรภายในทั้งหมดพร้อมเริ่มต้นใช้งานในคำสั่งเดียว
    bool init(uint8_t encA,
              uint8_t encB,
              uint16_t motorPPR = 370,
              uint8_t maxCounter = 30,
              uint32_t minPulseUs = 30,
              bool invertDirection = false);

    // ถอด Encoder ตัวนี้ออกจากระบบ Interrupt และ Timer แบบ Dynamic
    void detach();

    // ฟังก์ชันปรับแก้ตัวแปรภายในรายตัวขณะรัน
    void setConfig(uint16_t motorPPR, uint8_t maxCounter = 30, uint32_t minPulseUs = 30, bool invertDirection = false);
    void setPPR(uint16_t motorPPR);
    void setMaxCounter(uint8_t maxCounter);
    void setMinPulseUs(uint32_t minPulseUs);
    void setInvertDirection(bool invert);

    // รองรับโค้ดเดิมที่เรียก setPin()
    void setPin(uint8_t encA, uint8_t encB, uint16_t motorPPR = 370);

    void ISREncoder();
    void ReadRPM();

    int16_t getRPM() const { return CurrentRPM; }
    int32_t getCount() const { return EncoderCount - PulseOffset; }
    int32_t getRawCount() const { return EncoderCount; }
    float getAngle() const { return CurrentAngle; }
    float getTotalAngle() const { return TotalAngle; }
    void setOffset(int32_t target_pulse = 0);
    void reset();
    void debug(const char *name = nullptr, uint32_t interval_ms = 50);
};

// Object มาตรฐาน 3 ล้อ (สามารถประกาศเพิ่ม/ลด Object ใหม่ที่ไฟล์ไหนก็ได้)
extern MotorEncoder EncoderFL, EncoderFR, EncoderB;

// เริ่มต้น IntervalTimer สำหรับอัปเดต Encoder ทุกตัวที่ลงทะเบียนไว้ (ค่าเริ่มต้นทุก 1000 us = 1 ms)
void startEncoderTimer(uint32_t interval_us = 1000);
void updateAllRPM();
