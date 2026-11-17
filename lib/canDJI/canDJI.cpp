#include "robot.h"
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// เก็บค่ากระแสล่าสุดของมอเตอร์ ID 1 - 8 (Index 0 = ID 1, ..., Index 7 = ID 8)
int16_t c620_current_buffer[8] = {0};

// เก็บข้อมูล Feedback ของมอเตอร์ ID 1 - 8
C620Feedback_t c620_fb[8];

void initCAN()
{
    can1.begin();
    can1.setBaudRate(1000000);
}

/**
 * @brief สั่งค่ากระแสให้ C620 ตามหมายเลข ID (1 - 8)
 * @param id หมายเลข ID ของ C620 (1 ถึง 8)
 * @param current ค่ากระแสเป้าหมาย (-16384 ถึง 16384 เทียบเท่า -20A ถึง +20A)
 */
void setC620Current(uint8_t id, int16_t current)
{
    // 1. ตรวจสอบขอบเขต ID และจำกัดค่ากระแสไม่ให้เกินสเปก (-16384 ~ 16384)
    if (id < 1 || id > 8)
        return;
    current = constrain(current, -16384, 16384);

    // 2. อัปเดตค่าลงใน Buffer ของตัวที่สั่ง
    uint8_t idx = id - 1;
    c620_current_buffer[idx] = current;

    // 3. เตรียมเฟรม CAN (Standard Frame, 8 Bytes)
    CAN_message_t msg = {};
    msg.len = 8;

    // เลือก CAN ID และตำแหน่งเริ่มต้นใน Array ตามกลุ่ม ID
    uint8_t start_idx;
    if (id <= 4)
    {
        msg.id = 0x200; // ควบคุม ID 1 - 4
        start_idx = 0;
    }
    else
    {
        msg.id = 0x1FF; // ควบคุม ID 5 - 8
        start_idx = 4;
    }

    // 4. แพ็กข้อมูลแบบ Big-Endian (High Byte มาก่อน Low Byte)
    for (int i = 0; i < 4; i++)
    {
        int16_t val = c620_current_buffer[start_idx + i];
        msg.buf[i * 2] = (uint8_t)((val >> 8) & 0xFF); // High Byte
        msg.buf[i * 2 + 1] = (uint8_t)(val & 0xFF);    // Low Byte
    }

    // 5. ส่งออกทาง CAN Bus
    can1.write(msg);
}

/**
 * @brief อ่านเฟรม Feedback (0x201 - 0x208) จาก CAN RX Buffer และคำนวณ RPM / Angle
 * @param gear_ratio อัตราทดเกียร์ (ค่าเริ่มต้น M3508 = 3591.0f / 187.0f)
 */
void updateC620Feedback(float gear_ratio)
{
    if (gear_ratio <= 0.0f)
        gear_ratio = 1.0f;

    CAN_message_t rx;
    while (can1.read(rx))
    {
        // เฟรม Feedback ของ C620 คือ Standard ID 0x201 ถึง 0x208
        if (rx.flags.extended || rx.id < 0x201 || rx.id > 0x208)
            continue;

        uint8_t idx = rx.id - 0x201; // 0 .. 7 (สำหรับ ID 1 .. 8)
        C620Feedback_t &fb = c620_fb[idx];

        uint16_t new_raw_angle = (uint16_t)((rx.buf[0] << 8) | rx.buf[1]);
        fb.rotor_rpm           = (int16_t)((rx.buf[2] << 8) | rx.buf[3]);
        fb.torque_current      = (int16_t)((rx.buf[4] << 8) | rx.buf[5]);
        fb.temperature         = rx.buf[6];
        fb.last_rx_ms          = millis();

        // ตั้งค่ามุมเริ่มต้นในเฟรมแรกที่ได้รับ เพื่อใช้เป็นจุดอ้างอิง 0 องศา
        if (!fb.initialized)
        {
            fb.offset_angle   = new_raw_angle;
            fb.last_raw_angle = new_raw_angle;
            fb.raw_angle      = new_raw_angle;
            fb.round_cnt      = 0;
            fb.initialized    = true;
        }
        else
        {
            // เช็คการหมุนข้ามรอยต่อ 0 <-> 8191 (ครึ่งรอบคือ 4096)
            int32_t diff = (int32_t)new_raw_angle - (int32_t)fb.last_raw_angle;
            if (diff > 4096)
                fb.round_cnt--; // หมุนย้อนข้าม 0 ไปฝั่ง 8191
            else if (diff < -4096)
                fb.round_cnt++; // หมุนเดินหน้าข้าม 8191 ไปฝั่ง 0

            fb.last_raw_angle = new_raw_angle;
            fb.raw_angle      = new_raw_angle;
        }

        // คำนวณมุมสะสมและจำกัดขอบเขตมุมให้อยู่ในช่วง -180 ถึง +180 องศาเสมอ (O(1) ด้วย fmodf)
        fb.total_ecd         = (fb.round_cnt * 8192) + (int32_t)fb.raw_angle - (int32_t)fb.offset_angle;
        fb.total_shaft_angle = (fb.total_ecd * (360.0f / 8192.0f)) / gear_ratio;

        float raw_rotor_deg = fb.raw_angle * (360.0f / 8192.0f);
        fb.rotor_angle      = wrapAngle180(raw_rotor_deg);

        float wrapped_shaft = fmodf(fb.total_shaft_angle + 180.0f, 360.0f);
        if (wrapped_shaft < 0.0f)
            wrapped_shaft += 360.0f;
        fb.shaft_angle = wrapped_shaft - 180.0f;

        fb.shaft_rpm = fb.rotor_rpm / gear_ratio;
    }
}

bool isC620Online(uint8_t id, uint32_t timeout_ms)
{
    if (id < 1 || id > 8)
        return false;
    const C620Feedback_t &fb = c620_fb[id - 1];
    if (!fb.initialized)
        return false;
    return (millis() - fb.last_rx_ms) <= timeout_ms;
}

int16_t getC620RotorRPM(uint8_t id)
{
    if (id < 1 || id > 8)
        return 0;
    return c620_fb[id - 1].rotor_rpm;
}

float getC620ShaftRPM(uint8_t id)
{
    if (id < 1 || id > 8)
        return 0.0f;
    return c620_fb[id - 1].shaft_rpm;
}

float getC620RotorAngle(uint8_t id)
{
    if (id < 1 || id > 8)
        return 0.0f;
    return c620_fb[id - 1].rotor_angle;
}

float getC620ShaftAngle(uint8_t id)
{
    if (id < 1 || id > 8)
        return 0.0f;
    return c620_fb[id - 1].shaft_angle;
}

int16_t getC620Current(uint8_t id)
{
    if (id < 1 || id > 8)
        return 0;
    return c620_fb[id - 1].torque_current;
}

uint8_t getC620Temp(uint8_t id)
{
    if (id < 1 || id > 8)
        return 0;
    return c620_fb[id - 1].temperature;
}

void resetC620Angle(uint8_t id)
{
    if (id < 1 || id > 8)
        return;
    C620Feedback_t &fb = c620_fb[id - 1];
    fb.offset_angle      = fb.raw_angle;
    fb.round_cnt         = 0;
    fb.total_ecd         = 0;
    fb.shaft_angle       = 0.0f;
    fb.total_shaft_angle = 0.0f;
}

const C620Feedback_t &getC620Feedback(uint8_t id)
{
    static const C620Feedback_t empty_fb = {};
    if (id < 1 || id > 8)
        return empty_fb;
    return c620_fb[id - 1];
}
