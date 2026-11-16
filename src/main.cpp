#include "robot.h"

void setup()
{
    Serial.begin(115200);
    // PinModeMotor();
    initCAN();

    // ตั้งค่า CAN ID และมุมเริ่มต้น (PositionControl: -180 ถึง 180 องศา)
    // Arm1.setID(1);
    // Arm1.setPosition(0.0f, 3000.0f, 0.5f);

    Arm2.setID(3);
    Arm2.setPosition(0.0f, 3000.0f, 0.5f);

    // if (!initIMU())
    // {
    //     Serial.println("BNO055 not detected (check wiring or I2C address 0x28/0x29)");
    // }
}

void readSerialPosition()
{
    static char buf[32];
    static uint8_t idx = 0;
    static elapsedMillis lastCharTime;

    while (Serial.available())
    {
        char c = (char)Serial.read();
        lastCharTime = 0;

        if (c == '\n' || c == '\r')
        {
            if (idx > 0)
            {
                buf[idx] = '\0';
                float angle = strtof(buf, nullptr);
                idx = 0;
                Arm2.setPosition(angle, 3000.0f, 0.5f);
            }
        }
        else if (idx < sizeof(buf) - 1)
        {
            buf[idx++] = c;
        }
    }

    // กรณี Serial Monitor ไม่ได้ตั้งส่ง \n หรือ \r (No line ending): ถ้ารับตัวอักษรครบแล้วนิ่งเกิน 30 ms ให้สั่งงานทันที
    if (idx > 0 && lastCharTime >= 30)
    {
        buf[idx] = '\0';
        float angle = strtof(buf, nullptr);
        idx = 0;
        Arm2.setPosition(angle, 3000.0f, 0.5f);
    }
}

void loop()
{
    // updateIMU();

    // 1. รับค่ามุมเป้าหมายจาก Serial Monitor แบบ Non-blocking (พิมพ์องศา เช่น 90, -45, 0 แล้วกด Enter)
    readSerialPosition();

    // 2. อ่านค่า Feedback (RPM, Angle, Current, Temp) จาก C620 ทุกตัว
    updateC620Feedback();

    // 3. ควบคุมตำแหน่งมุมมอเตอร์ด้วย Position PID (ส่ง CAN ทุก 1 ms)
    Arm1.SpeedControl();
    Arm2.SpeedControl();

    // 4. แสดงผลกราฟ Teleplot ทาง Serial Monitor
    // Arm1.debug();
    // Arm2.debug();
}
