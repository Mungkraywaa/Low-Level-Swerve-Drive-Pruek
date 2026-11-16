#pragma once

#include <Arduino.h>

// อัตราทดเกียร์มาตรฐานของ M3508 = 3591 / 187 (~19.2032:1)
// หากถอดหัวเกียร์ออก (1:1) หรือมีอัตราทดอื่น สามารถส่งค่า gear_ratio เข้า updateC620Feedback() ได้
constexpr float C620_DEFAULT_GEAR_RATIO = 3591.0f / 187.0f;

struct C620Feedback_t
{
    uint16_t raw_angle = 0;       // มุมดิบของโรเตอร์ (0 .. 8191)
    uint16_t last_raw_angle = 0;  // มุมดิบรอบก่อนหน้า (ใช้เช็คการข้ามรอบ 0 <-> 8191)
    uint16_t offset_angle = 0;    // มุมเริ่มต้นตอนบูต (ใช้เป็นจุดอ้างอิง 0 องศา)
    int32_t  round_cnt = 0;       // จำนวนรอบสะสมของโรเตอร์
    int32_t  total_ecd = 0;       // ค่า Encoder สะสมรวมหลายรอบ (หน่วย: counts)

    int16_t  rotor_rpm = 0;       // ความเร็วรอบโรเตอร์ (RPM)
    float    shaft_rpm = 0.0f;    // ความเร็วรอบเพลาออกหลังผ่านเกียร์ (RPM)
    float    rotor_angle = 0.0f;  // มุมโรเตอร์ (-180.0 .. +180.0 deg)
    float    shaft_angle = 0.0f;  // มุมเพลาออกหลังผ่านเกียร์ (-180.0 .. +180.0 deg)
    float    total_shaft_angle = 0.0f; // มุมเพลาออกสะสมต่อเนื่องหลายรอบ (deg)

    int16_t  torque_current = 0;  // กระแสจริง (-16384 .. 16384 = -20A .. +20A)
    uint8_t  temperature = 0;     // อุณหภูมิไดรฟ์ (°C)
    uint32_t last_rx_ms = 0;      // เวลาล่าสุดที่ได้รับเฟรม CAN จากไดรฟ์ตัวนี้ (millis)
    bool     initialized = false; // รับเฟรมแรกเพื่อตั้งค่า offset_angle แล้วหรือยัง
};

void initCAN();
void setC620Current(uint8_t id, int16_t current);

// อ่านเฟรมจาก CAN RX Buffer และอัปเดตค่า RPM / Angle (เรียกใน loop() ได้ตลอด)
void updateC620Feedback(float gear_ratio = C620_DEFAULT_GEAR_RATIO);

// ฟังก์ชันดึงค่า RPM, Angle และข้อมูล Feedback ตามหมายเลข ID (1 - 8)
bool    isC620Online(uint8_t id, uint32_t timeout_ms = 50);
int16_t getC620RotorRPM(uint8_t id);
float   getC620ShaftRPM(uint8_t id);
float   getC620RotorAngle(uint8_t id);
float   getC620ShaftAngle(uint8_t id);
int16_t getC620Current(uint8_t id);
uint8_t getC620Temp(uint8_t id);
void    resetC620Angle(uint8_t id);
const C620Feedback_t &getC620Feedback(uint8_t id);
