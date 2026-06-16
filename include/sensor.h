#pragma once

#include "main.h"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"

// ============================================================
// 硬件对象声明（定义在 sensor.cpp）
// ============================================================

// 升降机构
extern pros::Motor    lift1;         // 左侧升降电机
extern pros::Motor    lift2;         // 右侧升降电机
extern pros::Rotation liftRotation;  // 升降编码器（V5 旋转传感器）

//底盘电机
extern pros::MotorGroup left_mg;    // 左侧电机组：正转端口 -10, 9；反转端口 -8
extern pros::MotorGroup right_mg;  // 右侧电机组：正转端口 1；反转端口 -2, 3

// 爪子电机和传感器
extern pros::Motor    Claw;  // 爪子电机
extern pros::Motor    Claw_Rot;         // 爪子旋转电机
extern pros::Rotation ClawRotation;  // 爪子编码器

// LemLib 定位硬件（IMU + 追踪轮编码器）— 端口留空，填入实际值
extern lemlib::V5InertialSensor  imu;
extern lemlib::V5RotationSensor  verticalEncoder;
extern lemlib::V5RotationSensor  horizontalEncoder; 