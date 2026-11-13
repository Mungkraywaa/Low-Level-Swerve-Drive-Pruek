#pragma once
#include <Arduino.h>

class heading
{
private:
    float Kp = 1.0f, Ki = 0.0f, Kd = 0.0f;
    float Error = 0.0f, SumError = 0.0f, PrevError = 0.0f;
    float Output = 0.0f;
    float IZone = 15.0f; // สะสม I เฉพาะเมื่อ Error น้อยกว่า 15 องศา
    elapsedMillis lockloop;

public:
    void setPID(float kp, float ki, float kd);
    void reset();
    float compute(float Setpoint);
};

extern heading Heading;
