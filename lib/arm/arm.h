#pragma once
#include <Arduino.h>

class arm
{
private:
    uint8_t motor_id = 1;

    // ตัวแปรเป้าหมายและขอบเขต
    float setpoint = 0.0f;      // เป้าหมายความเร็ว Rotor RPM (สำหรับ SpeedControl)
    float target_angle = 0.0f;  // เป้าหมายมุม Shaft Angle -180..180 deg (สำหรับ PositionControl)
    float max_current = 3000.0f;
    float accept_error = 0.5f;  // ระยะยอมรับความคลาดเคลื่อนของมุม (deg)
    float i_zone = 15.0f;       // สะสม I เฉพาะเมื่อ Error มุมน้อยกว่า 15 องศา

    // ค่า Gain PID (ใช้ร่วมกันหรือตั้งผ่าน setPID ได้)
    float Kp = 3.5f, Ki = 0.1f, Kd = 0.35f;
    float pos_Kp = 35.0f, pos_Ki = 0.2f, pos_Kd = 80.0f;

    float error = 0.0f, sumerror = 0.0f, preverror = 0.0f;
    int16_t pid_output = 0;

    elapsedMillis lockloop;
    elapsedMicros lastTx;
    elapsedMillis debugtime;

public:
    arm(uint8_t id = 1) : motor_id(id) {}

    void setID(uint8_t id);
    void setPID(float kp, float ki, float kd);
    void setPositionPID(float kp, float ki, float kd);

    void setSpeed(float target_rpm = 0.0f, float max_cur = 3000.0f);
    void setPosition(float angle_deg = 0.0f, float max_cur = 3000.0f, float accept_err = 0.5f);

    void reset();
    int16_t SpeedControl();
    int16_t PositionControl();
    void debug();
};

extern arm Arm1, Arm2;
