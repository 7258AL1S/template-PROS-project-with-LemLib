#include "lemlib/config.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "sensor.h"
#include "main.h"

// ============================================================
// 硬件对象定义（端口根据实际接线修改）
// ============================================================
pros::Motor    lift1(18);         // 左侧升降电机
pros::Motor    lift2(-19);         // 右侧升降电机
pros::Rotation liftRotation(20);   // 升降编码器（V5 旋转传感器）

//pros::MotorGroup left_mg({-10, 9, -8});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
//pros::MotorGroup right_mg({1 , -2, 3});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

lemlib::MotorGroup left_motors({-10, 9, -8}, 266_rpm);
// 右侧：正转端口 1；反转端口 -2, 3
lemlib::MotorGroup right_motors({1, -2, 3}, 266_rpm);

pros::Motor    Claw(15);  // 爪子电机
pros::Motor    Claw_Rot(-14);         // 爪子俯仰轴旋转电机
pros::Motor    Claw_return(17);         // 爪子滚转轴旋转电机
pros::Rotation ClawRotation(13);  // 爪子编码器


//Intake
pros::Motor    IntakeFront(-12);  // 吸球电机前
pros::Motor    IntakeBack(-16);    // 吸球电机后



// LemLib 定位硬件 — 请填入实际端口号
lemlib::V5InertialSensor  imu(4);                  // TODO: 改成 IMU 实际端口
lemlib::V5RotationSensor  verticalEncoder(5);      // TODO: 改成垂直定位轮编码器端口
lemlib::V5RotationSensor  horizontalEncoder(6);   // 反转：倒车时 X 减小 



//===========================================================
//气动电磁阀对象
//===========================================================
pros::ADIDigitalOut Piston_tuggle('A');  // 翻筒气动
pros::ADIDigitalOut Piston_claw('H');  // 爪子气动