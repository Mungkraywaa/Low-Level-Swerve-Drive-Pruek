#pragma once

// ตัวแปร global variable
extern float RobotYaw;

// ==========================================
// 1. System & Arduino Core Headers
// ==========================================
#include <Arduino.h>

// ==========================================
// 2. Project Module Headers
// ==========================================
#include "pin.h"
#include "encoder.h"
#include "motor.h"
#include "init.h"
#include "move.h"
#include "imu.h"
#include "headingcontrol.h"
#include "canDJI.h"
#include "arm.h"
#include "joy.h"
