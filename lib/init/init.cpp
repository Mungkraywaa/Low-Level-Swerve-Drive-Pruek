#include "robot.h"

IntervalTimer encoderTimer;

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

    // SetPin & PinMode
    MotorFL.setPin(MOTOR_FL_DIR_1, MOTOR_FL_DIR_2, MOTOR_FL_PWM, ENCODER_FL_IN_A, ENCODER_FL_IN_B, PPR);
    MotorFR.setPin(MOTOR_FR_DIR_1, MOTOR_FR_DIR_2, MOTOR_FR_PWM, ENCODER_FR_IN_A, ENCODER_FR_IN_B, PPR);
    MotorB.setPin(MOTOR_B_DIR_1, MOTOR_B_DIR_2, MOTOR_B_PWM, ENCODER_B_IN_A, ENCODER_B_IN_B, PPR);
    
    // Attach Interrupt สำหรับอ่าน Encoder
    attachInterrupt(digitalPinToInterrupt(ENCODER_FL_IN_A), []()
                    { MotorFL.ISREncoder(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_FR_IN_A), []()
                    { MotorFR.ISREncoder(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_IN_A), []()
                    { MotorB.ISREncoder(); }, RISING);

    // ตั้งเวลาอ่านค่า RPM ทุกๆ 1000 us (1 ms)
    encoderTimer.begin(updateAllRPM, 1000);
}
