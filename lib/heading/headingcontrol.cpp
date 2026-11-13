#include "robot.h"

heading Heading;

void heading::setPID(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void heading::reset()
{
    Error = 0.0f;
    SumError = 0.0f;
    PrevError = 0.0f;
    Output = 0.0f;
    lockloop = 0;
}

float heading::compute(float Setpoint)
{
    if (lockloop >= 10)
    {
        Error = wrapAngle180(RobotYaw - Setpoint);

        // 2. ถ้าหยุดเรียก compute() ไปนานเกิน 100 ms (เช่น หุ่นเพิ่งออกตัว) ให้ล้างค่าสะสมเก่าทิ้ง
        if (lockloop > 100)
        {
            SumError = 0.0f;
            PrevError = Error;
        }
        lockloop = 0;

        // 3. สะสม SumError เฉพาะเมื่ออยู่ใน IZone (< 15 องศา) เพื่อกัน I-term Overshoot
        if (fabsf(Error) < IZone)
        {
            SumError += Error;
            SumError = constrain(SumError, -100.0f, 100.0f);
        }
        else
        {
            SumError = 0.0f;
        }

        // คำนวณ Output นอก else ตามปกติ
        Output = (Kp * Error) + (Ki * SumError) + (Kd * (Error - PrevError));

        // Deadband: ถ้าใกล้เป้าหมายมาก (< 0.25 องศา) ให้หยุดสั่งหมุนและล้าง I
        if (fabsf(Error) < 0.25f)
        {
            SumError = 0.0f;
            Output = 0.0f;
        }

        PrevError = Error;
    }
    return Output;
}
