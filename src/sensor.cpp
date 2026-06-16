#include "main.h"
#include "sensor.h"

// ============================================================
// 硬件对象定义（端口根据实际接线修改）
// ============================================================
pros::Motor    lift1(18);         // 左侧升降电机
pros::Motor    lift2(-19);         // 右侧升降电机
pros::Rotation liftRotation(20);   // 升降编码器（V5 旋转传感器）

pros::MotorGroup left_mg({-10, 9, -8});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({1 , -2, 3});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

pros::Motor    Claw(11);  // 爪子电机
pros::Motor    Claw_Rot(14);         // 爪子旋转电机
pros::Rotation ClawRotation(13);  // 爪子编码器 