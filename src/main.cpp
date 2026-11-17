#include "robot.h"
elapsedMillis timmers;

MotorEncoder Enc1, Enc2;

#define LimitArm1 8
#define LimitArm2 5

float setpoint = 0.0f;

void armJoy();
void setup()
{
    pinMode(LimitArm1, INPUT_PULLUP);
    pinMode(LimitArm2, INPUT_PULLUP);

    Serial.begin(115200);
    Enc1.init(4, 3, 2048, 30, 5, false); // ขา A=4, B=3, PPR=2048, minPulseUs=5 us
    Enc2.init(6, 7, 2048, 30, 5, false); // ขา A=4, B=3, PPR=2048, minPulseUs=5 us

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
    Arm2.setPositionPID(17.0f, 0.3f, 80.0f);

    // Arm2.setPosition(0.0f, 3000.0f, 0.5f);

    // if (!initIMU())
    // {
    //     Serial.println("BNO055 not detected (check wiring or I2C address 0x28/0x29)");
    // }
}

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
    armJoy();
    // Enc2.debug("Enc");
    Serial.println(digitalRead(6));

    // readSerialSetpoint();
    // updateC620Feedback();
    // Arm2.setPosition(setpoint);
    // Arm2.PositionControl();
    // Arm2.debug();
}

void armJoy()
{
    static float pos1 = 0.0f;
    static float pos2 = 0.0f;

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

    // 2. อัปเดตมุมเป้าหมายแบบสะสม (ปล่อยจอยแล้วค้างมุมเดิม ไม่ดีดกลับ 0) ทุกๆ 10 ms
    if (timmers >= 10)
    {
        timmers = 0;

        // อัตราเปลี่ยนมุมสูงสุด ~1.5 องศา ต่อ 10 ms (~150 องศา/วินาที เมื่อดันจอยสุด)
        float step1 = (joy._Ly / 127.0f) * 1.5f;
        float step2 = (joy._Ry / 127.0f) * 1.5f;

        if (digitalRead(LimitArm1) == 0 && step1 < 0.0f)
        {
            step1 = 0.0f;
        }

        if (digitalRead(LimitArm2) == 0 && step2 < 0.0f)
        {
            step2 = 0.0f;
        }

        pos1 = constrain(pos1 + step1, -180.0f, 180.0f);
        pos2 = constrain(pos2 + step2, -180.0f, 180.0f);

        Arm1.setPosition(pos1, 12000.0f, 0.5f);
        Arm2.setPosition(pos2, 12000.0f, 0.5f);
    }

    // 3. คำนวณ Position PID (ทุก 10 ms) และส่งคำสั่ง CAN (ทุก 1 ms)
    Arm1.PositionControl();
    Arm2.PositionControl();
}