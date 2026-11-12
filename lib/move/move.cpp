#include "robot.h"

// สร้าง Object สำหรับเรียกใช้งานจากไฟล์อื่น
move RobotMove;

// ฟังก์ชันช่วยคุมขอบเขตมุมให้อยู่ในช่วง -180 ถึง 180 องศาเสมอ
static float wrapAngle180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

void move::KinematicsCompute(float vx, float vy, float omega)
{
    float yaw = 0.0f; // TODO: ดึงค่า Yaw (องศา) จาก IMU ในอนาคต

    float vx_FL = vx - (omega * WheelFL.trackwidth_m);
    float vy_FL = vy + (omega * WheelFL.wheelbase_m);

    float vx_FR = vx - (omega * WheelFR.trackwidth_m);
    float vy_FR = vy + (omega * WheelFR.wheelbase_m);

    float vx_B = vx - (omega * WheelB.trackwidth_m);
    float vy_B = vy + (omega * WheelB.wheelbase_m);

    WheelFL.target_speed = sqrt((vx_FL * vx_FL) + (vy_FL * vy_FL));
    WheelFL.target_angle = wrapAngle180((atan2(vy_FL, vx_FL) * RAD_TO_DEG) + WheelFL.offset_angle - yaw);

    WheelFR.target_speed = sqrt((vx_FR * vx_FR) + (vy_FR * vy_FR));
    WheelFR.target_angle = wrapAngle180((atan2(vy_FR, vx_FR) * RAD_TO_DEG) + WheelFR.offset_angle - yaw);

    WheelB.target_speed = sqrt((vx_B * vx_B) + (vy_B * vy_B));
    WheelB.target_angle = wrapAngle180((atan2(vy_B, vx_B) * RAD_TO_DEG) + WheelB.offset_angle - yaw);
}