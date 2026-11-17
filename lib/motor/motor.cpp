#include "robot.h"

// สร้าง Object ของมอเตอร์ทั้ง 3 ตัว (หน้าซ้าย, หน้าขวา, หลัง)
Motor MotorFL, MotorFR, MotorB;

// ฟังก์ชันตั้งค่าขาและสั่ง pinMode สำหรับขับมอเตอร์
void Motor::setPin(uint8_t inA, uint8_t inB, uint8_t pwm)
{
    MotorIN_A = inA;
    MotorIN_B = inB;
    MotorPWM = pwm;

    pinMode(MotorIN_A, OUTPUT);
    pinMode(MotorIN_B, OUTPUT);
    pinMode(MotorPWM, OUTPUT);
}

// ฟังก์ชันสั่งหมุนมอเตอร์ (-1023 ถึง 1023)
void Motor::drive(int16_t speed)
{
    speed = constrain(speed, -1023, 1023);
    if (speed > 0)
    {
        digitalWrite(MotorIN_A, HIGH);
        digitalWrite(MotorIN_B, LOW);
        analogWrite(MotorPWM, speed);
    }
    else if (speed < 0)
    {
        digitalWrite(MotorIN_A, LOW);
        digitalWrite(MotorIN_B, HIGH);
        analogWrite(MotorPWM, -speed);
    }
    else
    {
        digitalWrite(MotorIN_A, LOW);
        digitalWrite(MotorIN_B, LOW);
        analogWrite(MotorPWM, 0);
    }
}
