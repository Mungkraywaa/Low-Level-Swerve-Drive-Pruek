#include "robot.h"

// สร้าง Object สำหรับมอเตอร์แขน 2 ตัว (เปลี่ยนหมายเลข CAN ID เริ่มต้นได้ หรือใช้ setID() ใน setup)
arm Arm1(1);
arm Arm2(3);

void arm::setID(uint8_t id)
{
    motor_id = id;
}

void arm::setPID(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void arm::setPositionPID(float kp, float ki, float kd)
{
    pos_Kp = kp;
    pos_Ki = ki;
    pos_Kd = kd;
}

void arm::setSpeed(float target_rpm, float max_cur)
{
    setpoint = target_rpm;
    max_current = max_cur;
}

void arm::setPosition(float angle_deg, float max_cur, float accept_err)
{
    target_angle = wrapAngle180(angle_deg);
    max_current = max_cur;
    accept_error = accept_err;
}

void arm::reset()
{
    error = 0.0f;
    sumerror = 0.0f;
    preverror = 0.0f;
    pid_output = 0;
    lockloop = 0;
}

int16_t arm::SpeedControl()
{
    // 1. คำนวณ Speed PID ทุกๆ 10 ms (100 Hz)
    if (lockloop >= 10)
    {
        // ถ้าไดรฟ์ C620 ไม่มีไฟเลี้ยง / ขาดการติดต่อทาง CAN เกิน 50 ms หรือไม่ได้เรียกใช้นานเกิน 100 ms
        // ให้ล้างค่าสะสม sumerror ทันที เพื่อไม่ให้กระชากตอนสับสวิตช์จ่ายไฟเข้าบอร์ด
        if (!isC620Online(motor_id) || lockloop > 100)
        {
            sumerror = 0.0f;
            error = 0.0f;
            preverror = 0.0f;
            pid_output = 0;
            lockloop = 0;
        }
        else
        {
            lockloop = 0;

            error = setpoint - getC620RotorRPM(motor_id);

            // ถ้าสั่งหยุด (setpoint == 0) ไม่ให้สะสม I เพิ่ม
            if (fabsf(setpoint) < 1.0f)
            {
                sumerror = 0.0f;
            }
            else
            {
                sumerror += error;
                sumerror = constrain(sumerror, -5000.0f, 5000.0f); // กัน Integral Windup
            }

            float out = (Kp * error) + (Ki * sumerror) + (Kd * (error - preverror));
            preverror = error;

            // จำกัดกระแสสูงสุดไว้ที่ ±max_current
            pid_output = (int16_t)constrain(out, -max_current, max_current);
        }
    }

    // 2. ส่งคำสั่งกระแสจาก PID ให้ C620 ทุกๆ 1 ms (1000 us = 1 kHz)
    if (lastTx >= 1000)
    {
        lastTx = 0;
        setC620Current(motor_id, pid_output);
    }

    return pid_output;
}

int16_t arm::PositionControl()
{
    // 1. คำนวณ Position PID ทุกๆ 10 ms (100 Hz)
    if (lockloop >= 10)
    {
        // ถ้าไดรฟ์ C620 ไม่มีไฟเลี้ยง / ขาดการติดต่อทาง CAN เกิน 50 ms หรือไม่ได้เรียกใช้นานเกิน 100 ms
        // ให้ล้างค่าสะสม sumerror และ pid_output ทันที เพื่อป้องกันมอเตอร์กระชากตอนเปิดไฟเข้าบอร์ด
        if (!isC620Online(motor_id) || lockloop > 100)
        {
            sumerror = 0.0f;
            error = 0.0f;
            preverror = 0.0f;
            pid_output = 0;
            lockloop = 0;
        }
        else
        {
            lockloop = 0;

            // หาความคลาดเคลื่อนของมุมเพลาออก (-180 ถึง +180 องศา)
            error = wrapAngle180(target_angle - getC620ShaftAngle(motor_id));

            // สะสม I เฉพาะเมื่อ Error อยู่ในระยะ i_zone (< 15 องศา) เพื่อกัน Overshoot
            if (fabsf(error) < i_zone)
            {
                sumerror += error;
                sumerror = constrain(sumerror, -1000.0f, 1000.0f);
            }
            else
            {
                sumerror = 0.0f;
            }

            float out = (pos_Kp * error) + (pos_Ki * sumerror) + (pos_Kd * (error - preverror));

            // Deadband: ถ้าใกล้เป้าหมายมาก (< accept_error) ให้หยุดสั่งกระแสและล้าง I
            if (fabsf(error) < accept_error)
            {
                sumerror = 0.0f;
                out = 0.0f;
            }

            preverror = error;

            // จำกัดกระแสสูงสุดไว้ที่ ±max_current
            pid_output = (int16_t)constrain(out, -max_current, max_current);
        }
    }

    // 2. ส่งคำสั่งกระแสจาก PID ให้ C620 ทุกๆ 1 ms (1000 us = 1 kHz)
    if (lastTx >= 1000)
    {
        lastTx = 0;
        setC620Current(motor_id, pid_output);
    }

    return pid_output;
}

void arm::debug()
{
    if (debugtime < 10)
        return;
    debugtime = 0;

    Serial.printf(">ID%u_RotorRPM: %d\n", motor_id, getC620RotorRPM(motor_id));
    Serial.printf(">ID%u_ShaftRPM: %.2f\n", motor_id, getC620ShaftRPM(motor_id));
    Serial.printf(">ID%u_RotorAngle: %.2f\n", motor_id, getC620RotorAngle(motor_id));
    Serial.printf(">ID%u_ShaftAngle: %.2f\n", motor_id, getC620ShaftAngle(motor_id));
    Serial.printf(">ID%u_TargetAngle: %.2f\n", motor_id, target_angle);
    Serial.printf(">ID%u_Error: %.2f\n", motor_id, error);
    Serial.printf(">ID%u_Current: %d\n", motor_id, getC620Current(motor_id));
    Serial.printf(">ID%u_setpoint: %.2f\n", motor_id, setpoint);
    Serial.printf(">ID%u_pid_output: %d\n", motor_id, pid_output);
    Serial.printf(">ID%u_Max: %.2f\n", motor_id, setpoint + 30.0f);
    Serial.printf(">ID%u_Min: %.2f\n\n", motor_id, setpoint - 30.0f);
}
