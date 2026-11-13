#pragma once
#include <Arduino.h>

// องศาตำแหน่งติดตั้งของแต่ละล้อ (-180 ถึง 180 องศา)
#define Angle_Back 180.0f
#define Angle_Front_Left 60.0f
#define Angle_Front_Right -60.0f

// องศาชดเชย (Offset) ของแต่ละล้อให้ชี้ทิศ 0 องศาตรงกัน
#define Offset_Angle_Back 0.0f
#define Offset_Angle_Front_Left 0.0f
#define Offset_Angle_Front_Right 0.0f

// รัศมีจากจุดศูนย์กลางหุ่นยนต์ถึงล้อ (หน่วย: เมตร)
#define ROBOT_RADIUS 0.0f

class move
{
private:
    struct Swerve // แม่พิมพ์ตัวแปร Kinematics สำหรับล้อ 1 ล้อ
    {
        float mount_angle;  // องศาตำแหน่งติดตั้งของล้อ
        float offset_angle; // องศาชดเชยให้ล้อชี้ทิศ 0 องศาเสมอ
        float radius;       // รัศมีจากจุดศูนย์กลางหุ่นถึงล้อ (เมตร)

        float wheelbase_m;  // ระยะแกน X: radius * cos(angle)
        float trackwidth_m; // ระยะแกน Y: radius * sin(angle)

        float target_speed = 0.0f; // ความเร็วล้อที่คำนวณได้
        float target_angle = 0.0f; // องศาล้อเป้าหมายที่คำนวณได้

        // Constructor: รับค่า #define ข้างนอกเข้ามาคำนวณของใครของมันอัตโนมัติ
        Swerve(float angle_deg, float offset_deg, float r)
        {
            mount_angle = angle_deg;
            offset_angle = offset_deg;
            radius = r;
            wheelbase_m = r * cos(angle_deg * DEG_TO_RAD);
            trackwidth_m = r * sin(angle_deg * DEG_TO_RAD);
        }
    };

    Swerve WheelFL{Angle_Front_Left, Offset_Angle_Front_Left, ROBOT_RADIUS};   // ล้อหน้าซ้าย (Front Left)
    Swerve WheelFR{Angle_Front_Right, Offset_Angle_Front_Right, ROBOT_RADIUS}; // ล้อหน้าขวา (Front Right)
    Swerve WheelB{Angle_Back, Offset_Angle_Back, ROBOT_RADIUS};                // ล้อหลัง (Back)

public:
    void KinematicsCompute(float vx = 0.0f, float vy = 0.0f, float omega = 0.0f);
    void debug();
};

extern move RobotMove;