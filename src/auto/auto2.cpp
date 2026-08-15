#include "auto.h"

// ============================================================
// 自动程序
// ============================================================
void auto2() {//开局左拐一个pin

    startAutoBackgroundTasks();

    Claw_Rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Claw_Rot.brake();
    Claw_return.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Claw_return.brake();

    TugglePick.move(127);
    pros::delay(500);
    TugglePick.move(0);
    

   // left_motors.setBrakeMode(lemlib::BrakeMode::COAST);
   // right_motors.setBrakeMode(lemlib::BrakeMode::COAST);
   // pros::delay(100000);
    // === 运动参数 ===
    LiftPID pid = {0.03f, 0.0f, 0.0f};
    LiftPID pidShort = {0.25f, 0.0f, 2.1f};
    lemlib::MoveToPoseParams Poseparams;
    lemlib::MoveToPoseSettings Posesettings;
    lemlib::MoveToPointParams params;
    //params.maxAngularSpeed = 0;
    lemlib::MoveToPointSettings settings;
    lemlib::TurnToParams turnParams;
    lemlib::TurnToSettings turnSettings;
   

    // GoForWard(1.0,-10.3,1000,pid);            // 旧版
    GoForWardCurve(0.9,5.5,1000,3.9f);        // 新版：Power, Target, FullTime, DecelDist



    liftCmd = {38, 325, 500};
    liftGo = true;
    //TurnCurve(1.0, 90.0, 800, 15.0);  
    turnSettings.angularPID = lemlib::PID(1.32, 0.0, 0.09);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    GoForWardCurve(1.0,14.7,1000,8.3f); 
    LiftUpDegree(-14, 358, 450);
    pros::delay(100);
    ClawOpen();
    pros::delay(300);
    left_motors.move(0);
	right_motors.move(0);
    GoForWardCurve(0.9,-4.6,1000,2.0f);

    return;

    turnSettings.angularPID = lemlib::PID(1.38, 0.0, 0.073);
    lemlib::turnTo(-45_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {-10, 359, 300};
    liftGo = true;
    ClawOpen();
    ClawIntake();
    GoForWardCurve(1.0,18.7,1000,9.0f);
    pros::delay(100);
    ClawClose();
    // 通知后台任务（autoSubsystems/debugTask）退出，避免进入手动阶段后仍在轮询/刷屏
    autoActive = false;
    pros::delay(100);
    return;


}
