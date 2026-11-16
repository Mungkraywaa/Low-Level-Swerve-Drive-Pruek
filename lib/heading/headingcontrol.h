#pragma once
#include <Arduino.h>

class heading
{
private:
    float Setpoint = 0.0f, MaxSpeed = 100.0f, AcceptError = 0.25f;
    float Kp = 1.0f, Ki = 0.0f, Kd = 0.0f;
    float Error = 0.0f, SumError = 0.0f, PrevError = 0.0f;
    float Output = 0.0f;
    float IZone = 15.0f; // สะสม I เฉพาะเมื่อ Error น้อยกว่า 15 องศา
    elapsedMillis lockloop;
    elapsedMillis debugtime;

public:
    void setPID(float kp, float ki, float kd);
    void set(float Setpoint = 0.0f, float MaxSpeed = 100.0f, float AcceptError = 0.25f);
    void reset();
    float compute();
    void debug();
};

extern heading Heading;
