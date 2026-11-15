#include "robot.h"

void setup()
{
    Serial.begin(115200);
    // PinModeMotor();
    initCAN();
    // if (!initIMU())
    // {
    //     Serial.println("BNO055 not detected (check wiring or I2C address 0x28/0x29)");
    // }
}

elapsedMillis lockloop;
float sumerror = 0.0f, preverror = 0.0f;
int16_t pid_output = 0;
float setpoint = 500.0f; // เป้าหมาย Rotor RPM

void loop()
{
    // updateIMU();

    // 1. อ่านค่า Feedback (RPM, Angle, Current, Temp) จาก C620
    updateC620Feedback();

    // 2. คำนวณ PID ทุกๆ 10 ms (100 Hz)
    if (lockloop >= 10)
    {
        lockloop = 0;

        float Kp = 3.5f;
        float Ki = 0.1f;
        float Kd = 0.35f;

        float error = setpoint - getC620RotorRPM(3);

        sumerror += error;
        sumerror = constrain(sumerror, -5000.0f, 5000.0f); // กัน Integral Windup

        float out = (Kp * error) + (Ki * sumerror) + (Kd * (error - preverror));
        preverror = error;

        // จำกัดกระแสสูงสุดไว้ที่ ±10000 (จากสเปก ±16384) เพื่อความปลอดภัย
        pid_output = (int16_t)constrain(out, -10000.0f, 10000.0f);
    }

    // 3. ส่งคำสั่งกระแสจาก PID ให้ C620 ทุกๆ 1 ms (1 kHz)
    static uint32_t lastTx = 0;
    if (micros() - lastTx >= 1000)
    {
        lastTx = micros();
        setC620Current(3, pid_output);
    }

    // 4. แสดงผลกราฟ Teleplot ทาง Serial Monitor ทุกๆ 10 ms
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 10)
    {
        lastPrint = millis();

        Serial.print(">RotorRPM: ");
        Serial.println(getC620RotorRPM(3));
        Serial.print(">ShaftRPM: ");
        Serial.println(getC620ShaftRPM(3));
        Serial.print(">RotorAngle: ");
        Serial.println(getC620RotorAngle(3));
        Serial.print(">ShaftAngle: ");
        Serial.println(getC620ShaftAngle(3));
        Serial.print(">Current: ");
        Serial.println(getC620Current(3));
        Serial.print(">setpoint: ");
        Serial.println(setpoint);
        Serial.print(">pid_output: ");
        Serial.println(pid_output);

        Serial.print(">Max: ");
        Serial.println(setpoint + 30);
        Serial.print(">Min: ");
        Serial.println(setpoint - 30);
        Serial.println();
    }
}
