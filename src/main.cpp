#include "robot.h"
elapsedMillis timmers;

#define LimitArm1 8
#define LimitArm2 5

void setup()
{
    pinMode(LimitArm1, INPUT_PULLUP);
    pinMode(LimitArm2, INPUT_PULLUP);

    Serial.begin(115200);
    // PinModeMotor();
    initCAN();
    joy.init();
    // ตั้งค่า CAN ID และมุมเริ่มต้น (PositionControl: -180 ถึง 180 องศา)
    // Arm1.setID(1);
    // Arm1.setPosition(0.0f, 3000.0f, 0.5f);

    Arm1.setID(2);
    Arm2.setID(3);

    Arm1.setPID(3.0f, 0.8f, 0.0f);
    Arm2.setPID(3.0f, 0.8f, 0.0f);
    Arm2.setPositionPID(20.0f, 0.3f, 80.0f);

    // Arm2.setPosition(0.0f, 3000.0f, 0.5f);

    // if (!initIMU())
    // {
    //     Serial.println("BNO055 not detected (check wiring or I2C address 0x28/0x29)");
    // }
}

float setpoint = 0.0f;

void readSerialSetpoint()
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
                setpoint = strtof(buf, nullptr);
                idx = 0;
            }
        }
        else if (idx < sizeof(buf) - 1)
        {
            buf[idx++] = c;
        }
    }

    // กรณีไม่ได้ตั้งส่ง \n หรือ \r (No line ending): ถ้ารับตัวอักษรครบแล้วนิ่งเกิน 30 ms ให้อัปเดต setpoint ทันที
    if (idx > 0 && lastCharTime >= 30)
    {
        buf[idx] = '\0';
        setpoint = strtof(buf, nullptr);
        idx = 0;
    }
}

void loop()
{
    readSerialSetpoint();
    updateC620Feedback();
    Arm2.setPosition(setpoint);
    Arm2.PositionControl();
    Arm2.debug();
}

void armJoy()
{
    // 1. อ่านค่าจอย PS4 และค่า Feedback จาก C620 ทุกๆ รอบ loop
    joy.update();
    updateC620Feedback();

    if (!joy.isConnected)
    {
        Arm1.setSpeed(0.0f);
        Arm1.SpeedControl();
        Arm2.setSpeed(0.0f);
        Arm2.SpeedControl();
        return;
    }

    // 2. อัปเดตค่าความเร็วเป้าหมายจากจอยทุกๆ 10 ms
    if (timmers >= 10)
    {
        timmers = 0;
        int speed1 = map(joy._Ly, -127, 127, -5000, 5000);
        int speed2 = map(joy._Ry, -127, 127, -5000, 5000);

        if (digitalRead(LimitArm1) == 0 && speed1 < 0)
        {
            speed1 = 0;
        }

        if (digitalRead(LimitArm2) == 0 && speed2 < 0)
        {
            speed2 = 0;
        }
        Arm1.setSpeed(speed1, 12000);
        Arm2.setSpeed(speed2, 12000);

        Serial.print(">speed1: ");
        Serial.println(speed1);
        Serial.print(">speed2: ");
        Serial.println(speed2);
        Serial.print(">cur1: ");
        Serial.println(getC620Current(2));
        Serial.print(">cur2: ");
        Serial.println(getC620Current(3));
        Serial.print(">LimitArm1: ");
        Serial.println(digitalRead(LimitArm1));
        Serial.print(">LimitArm2: ");
        Serial.println(digitalRead(LimitArm2));
    }

    // 3. คำนวณ PID (ทุก 10 ms) และส่งคำสั่ง CAN (ทุก 1 ms) ต้องอยู่นอก if (timmers >= 10)
    Arm1.SpeedControl();
    Arm2.SpeedControl();
}