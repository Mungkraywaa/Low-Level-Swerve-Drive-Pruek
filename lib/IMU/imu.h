#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern Adafruit_BNO055 bno;

/**
 * @brief ฟังก์ชันช่วยคุมขอบเขตมุมให้อยู่ในช่วง -180 ถึง 180 องศาเสมอ
 */
float wrapAngle180(float angle);

/**
 * @brief เริ่มต้นการทำงานของ BNO055 ผ่าน I2C
 * @return true หากตรวจพบและเริ่มต้นเซนเซอร์สำเร็จ
 */
bool initIMU();

/**
 * @brief อัปเดตค่าจาก BNO055 แบบ Non-blocking ใน loop() (ทุก 10 ms / ~100 Hz)
 */
void updateIMU();
