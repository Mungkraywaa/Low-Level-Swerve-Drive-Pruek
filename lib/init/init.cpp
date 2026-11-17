#include "robot.h"

///////////////////////// Pin Motor FL (Front Left)
#define MOTOR_FL_DIR_1 2
#define MOTOR_FL_DIR_2 1
#define MOTOR_FL_PWM 0

#define ENCODER_FL_IN_A 14
#define ENCODER_FL_IN_B 15

///////////////////////// Pin Motor FR (Front Right)
#define MOTOR_FR_DIR_1 4
#define MOTOR_FR_DIR_2 5
#define MOTOR_FR_PWM 3

#define ENCODER_FR_IN_A 17
#define ENCODER_FR_IN_B 16

///////////////////////// Pin Motor B (Back)
#define MOTOR_B_DIR_1 24
#define MOTOR_B_DIR_2 12
#define MOTOR_B_PWM 11

#define ENCODER_B_IN_A 23
#define ENCODER_B_IN_B 22

// PPR Motor
#define PPR 370

void PinModeMotor()
{
    // Settings analogWrite Resolution To 10 Bit
    analogWriteResolution(10);

    // 1. ตั้งค่าขาขับมอเตอร์ (Motor Driver)
    MotorFL.setPin(MOTOR_FL_DIR_1, MOTOR_FL_DIR_2, MOTOR_FL_PWM);
    MotorFR.setPin(MOTOR_FR_DIR_1, MOTOR_FR_DIR_2, MOTOR_FR_PWM);
    MotorB.setPin(MOTOR_B_DIR_1, MOTOR_B_DIR_2, MOTOR_B_PWM);

    // 2. กำหนดค่าและลงทะเบียน Encoder แบบ Dynamic (ผูก Interrupt + เริ่ม Timer 1 ms ให้อัตโนมัติ)
    // รูปแบบ: .init(pinA, pinB, ppr, maxCounter, minPulseUs, invertDirection)
    EncoderFL.init(ENCODER_FL_IN_A, ENCODER_FL_IN_B, PPR, 30, 30, false);
    EncoderFR.init(ENCODER_FR_IN_A, ENCODER_FR_IN_B, PPR, 30, 30, false);
    EncoderB.init(ENCODER_B_IN_A, ENCODER_B_IN_B, PPR, 30, 30, false);
}
