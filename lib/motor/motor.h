#pragma once
#include <Arduino.h>

class Motor
{
private:
    //=======================================================
    // Motor Pin
    //=======================================================
    uint8_t MotorIN_A = 0; // IN Motor A
    uint8_t MotorIN_B = 0; // IN Motor B
    uint8_t MotorPWM = 0;  // PWM

public:
    void setPin(uint8_t inA, uint8_t inB, uint8_t pwm);
    void drive(int16_t speed);
};

extern Motor MotorFL, MotorFR, MotorB;
