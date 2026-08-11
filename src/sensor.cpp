#include "lemlib/config.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "sensor.h"
#include "main.h"

// ============================================================
// 硬件对象定义（端口根据实际接线修改）
// ============================================================
pros::Motor    lift1(-12);         // 左侧升降电机
pros::Motor    lift2(11);         // 右侧升降电机
pros::Rotation liftRotation(9);   // 升降编码器（V5 旋转传感器）

//pros::MotorGroup left_mg({-10, 9, -8});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
//pros::MotorGroup right_mg({1 , -2, 3});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

lemlib::MotorGroup left_motors({-2,-1,-3}, 240_rpm);
// 右侧：正转端口 1；反转端口 -2, 3
lemlib::MotorGroup right_motors({4,20,5}, 240_rpm);

pros::Motor    Claw(-10);  // 爪子电机
pros::Motor    Claw_Rot(-21);         // 爪子俯仰轴旋转电机
pros::Motor    Claw_return(-21);         // 爪子滚转轴旋转电机
pros::Rotation ClawRotation(-21);  // 爪子编码器
pros::Motor    Claw_Left(8);         // 爪子左旋转电机
pros::Motor    Claw_Right(6);         // 爪子右旋转电机

pros::Motor    TugglePick(-21);  // tuggle拨片

//Intake
pros::Motor    IntakeFront(-21);  // 吸球电机前
pros::Motor    IntakeBack(-21);    // 吸球电机后



// LemLib 定位硬件 — 请填入实际端口号
lemlib::V5InertialSensor  imu(16);                  
lemlib::V5RotationSensor  verticalEncoder(17);      
lemlib::V5RotationSensor  horizontalEncoder(18);   



//===========================================================
//气动电磁阀对象
//===========================================================
pros::ADIDigitalOut Piston_tuggle('A');  // 翻筒气动
pros::ADIDigitalOut Piston_tuggle2('B');  // 翻筒气动
pros::ADIDigitalOut Piston_claw('H');  // 爪子气动