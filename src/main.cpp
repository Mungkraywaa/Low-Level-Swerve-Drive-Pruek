#include "robot.h"

void setup()
{
    Serial.begin(115200);
    PinModeMotor();
}

void loop()
{
    // สั่งจ่าย PWM 1023 (จาก 1023) ให้ MotorFL หมุน
    MotorFL.drive(1023);

    // แสดงผลค่า RPM ของ MotorFL ผ่าน Serial Monitor
    Serial.print("MotorFL RPM: ");
    Serial.println(MotorFL.CurrentRPM);

    delay(10);
}
