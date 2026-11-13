#include "robot.h"

Adafruit_BNO055 bno(55, 0x29, &Wire);
float RobotYaw = 0.0f;

static bool imuReady = false;
static elapsedMillis imuPollTimer;

// ฟังก์ชันช่วยคุมขอบเขตมุมให้อยู่ในช่วง -180 ถึง 180 องศาเสมอ
float wrapAngle180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

bool initIMU()
{
    Wire.begin();

    if (!bno.begin())
    {
        imuReady = false;
        return false;
    }

    // เปิดใช้ External Crystal เฉพาะบอร์ดที่มี 32.768 kHz crystal บนตัว (เช่น Adafruit BNO055 แท้)
    bno.setExtCrystalUse(true);

    imuReady = true;
    imuPollTimer = 0;
    return true;
}

void updateIMU()
{
    // อ่านใน loop() ที่ ~100 Hz เพื่อไม่ให้ Blocking I2C และ Clock Stretching ไปรบกวน ISR
    if (!imuReady || imuPollTimer < 10)
        return;

    imuPollTimer = 0;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    RobotYaw = wrapAngle180(static_cast<float>(euler.x()));
}
