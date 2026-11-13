#include "robot.h"

// สร้าง Object สำหรับเรียกใช้งานจากไฟล์อื่น
robotmove RobotMove;


void robotmove::debug()
{
    static uint32_t Prevtime;
    if (millis() - Prevtime >= 50)
    {
        }
}
void robotmove::KinematicsCompute(float vx, float vy, float omega)
{
    // คำนวน Vx Vy FL wheel
    float vx_FL = vx - (omega * WheelFL.trackwidth_m);
    float vy_FL = vy + (omega * WheelFL.wheelbase_m);

    // คำนวน Vx Vy FR wheel
    float vx_FR = vx - (omega * WheelFR.trackwidth_m);
    float vy_FR = vy + (omega * WheelFR.wheelbase_m);

    // คำนวน Vx Vy B wheel
    float vx_B = vx - (omega * WheelB.trackwidth_m);
    float vy_B = vy + (omega * WheelB.wheelbase_m);

    WheelFL.target_speed = sqrt((vx_FL * vx_FL) + (vy_FL * vy_FL));
    WheelFL.target_angle = wrapAngle180((atan2(vy_FL, vx_FL) * RAD_TO_DEG) + WheelFL.offset_angle - RobotYaw);

    WheelFR.target_speed = sqrt((vx_FR * vx_FR) + (vy_FR * vy_FR));
    WheelFR.target_angle = wrapAngle180((atan2(vy_FR, vx_FR) * RAD_TO_DEG) + WheelFR.offset_angle - RobotYaw);

    WheelB.target_speed = sqrt((vx_B * vx_B) + (vy_B * vy_B));
    WheelB.target_angle = wrapAngle180((atan2(vy_B, vx_B) * RAD_TO_DEG) + WheelB.offset_angle - RobotYaw);
}
