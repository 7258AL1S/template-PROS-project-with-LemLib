#include "auto.h"


void auto2(int StopFlag) {

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
    GoForWardCurve(1.0,6.0,1000,3.9f);        // 新版：Power, Target, FullTime, DecelDist



    liftCmd = {38, 325, 500};
    liftGo = true;
    //TurnCurve(1.0, 90.0, 800, 15.0);  
    turnSettings.angularPID = lemlib::PID(1.3, 0.0, 0.086);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    GoForWardCurve(1.0,14.6,1000,9.0f); 
    left_motors.move(0.1);
	right_motors.move(0.1);
    LiftUpDegree(-14, 358, 450);
    
    pros::delay(200);
    lift1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift1.brake();
	lift2.brake();
    ClawOpen();
    pros::delay(500);
    /////
    left_motors.move(0);
	right_motors.move(0);
    GoForWardCurve(1.0,-5.7,1000,4.0f);

    if(StopFlag == 1){
        // 通知后台任务（autoSubsystems/debugTask）退出，避免进入手动阶段后仍在轮询/刷屏
        autoActive = false;
        pros::delay(100);
        return;
    }

    turnSettings.angularPID = lemlib::PID(1.43, 0.0, 0.086);
    lemlib::turnTo(45_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {-30, 359, 300};
    liftGo = true;



    ClawOpen();
    ClawIntake();
    GoForWardCurve(1.0,19,1600,9.0f);
    pros::delay(100);
    ClawClose();
    //ClawStopIntake();
    liftCmd = {50, 315, 800};
    liftGo = true;
    GoForWardCurve(1.0,2,500,1.0f);
    turnSettings.angularPID = lemlib::PID(1.3, 0.0, 0.1);
    lemlib::turnTo(180_stDeg, 900_msec, turnParams, turnSettings);
    GoForWardCurve(1.0,12.4,1000,8.0f); 
    left_motors.move(0.1);
	right_motors.move(0.1);
    LiftUpDegree(-25, 350, 500);
    ClawOpen();
    pros::delay(100);
    /////
    left_motors.move(0);
	right_motors.move(0);
    GoForWardCurve(1.0,-6,1000,3.9f); 

    if(StopFlag == 2){
        // 通知后台任务（autoSubsystems/debugTask）退出，避免进入手动阶段后仍在轮询/刷屏
        autoActive = false;
        pros::delay(100);
        return;
    }

    liftCmd = {-30, 359, 600};
    liftGo = true;
    turnSettings.angularPID = lemlib::PID(1.44, 0.0, 0.08);
    lemlib::turnTo(135_stDeg, 900_msec, turnParams, turnSettings);
    ClawOpen();
    ClawIntake();
    GoForWardCurve(1.0,19.3,1600,8.0f);
    pros::delay(100);
    ClawClose();
    //ClawStopIntake();
    liftCmd = {80, 305, 1000};
    liftGo = true;
    GoForWardCurve(1.0,4.0,800,1.0f);
    liftCmd = {40, 305, 300};
    liftGo = true;
    turnSettings.angularPID = lemlib::PID(1.44, 0.0, 0.11);
    lemlib::turnTo(-90_stDeg, 900_msec, turnParams, turnSettings);
    GoForWardCurve(1.0,15.5,1000,9.0f); 
    left_motors.move(0.1);
	right_motors.move(0.1);
    LiftUpDegree(-20, 332, 800);
    ClawOpen();
    pros::delay(400);
    GoForWardCurve(1.0,-7.1,1000,3.9f); 
    ClawClose();
    liftCmd = {-40, 359, 800};
    liftGo = true;
    turnSettings.angularPID = lemlib::PID(1.5, 0.0, 0.08);
    lemlib::turnTo(135_stDeg, 900_msec, turnParams, turnSettings);
    // 通知后台任务（autoSubsystems/debugTask）退出，避免进入手动阶段后仍在轮询/刷屏
    autoActive = false;
    pros::delay(100);
    return;


}
