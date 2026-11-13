#include "robot.h"

void setup()
{
    Serial.begin(115200);
    PinModeMotor();

    if (!initIMU())
    {
        Serial.println("BNO055 not detected (check wiring or I2C address 0x28/0x29)");
    }
}

void loop()
{
    updateIMU();
}
