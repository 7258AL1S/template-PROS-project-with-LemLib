#include "auto.h"


void auto1(int StopFlag) {

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
    turnSettings.angularPID = lemlib::PID(1.3, 0.0, 0.1);
    lemlib::turnTo(-90_stDeg, 900_msec, turnParams, turnSettings);
    GoForWardCurve(1.0,14.5,1000,9.0f); 
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
    GoForWardCurve(1.0,-7.5,1000,4.0f);

    if(StopFlag == 1){
        // 通知后台任务（autoSubsystems/debugTask）退出，避免进入手动阶段后仍在轮询/刷屏
        autoActive = false;
        pros::delay(100);
        return;
    }

    turnSettings.angularPID = lemlib::PID(1.43, 0.0, 0.086);
    lemlib::turnTo(-45_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {-30, 359, 300};
    liftGo = true;



    ClawOpen();
    ClawIntake();
    GoForWardCurve(1.0,19.8,1600,9.0f);
    pros::delay(100);
    ClawClose();
    //ClawStopIntake();
    liftCmd = {60, 306, 1000};
    liftGo = true;
    GoForWardCurve(1.0,2.6,500,1.0f);
    turnSettings.angularPID = lemlib::PID(1.28, 0.0, 0.115);
    lemlib::turnTo(-180_stDeg, 900_msec, turnParams, turnSettings);
    GoForWardCurve(1.0,13.3,1000,7.0f); 
    left_motors.move(0.1);
	right_motors.move(0.1);
    LiftUpDegree(-22, 350, 500);

    
    pros::delay(200);
    lift1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift1.brake();
	lift2.brake();
    ClawOpen();
    pros::delay(100);

    /////
    left_motors.move(0);
	right_motors.move(0);

    GoForWardCurve(1.0,-7.1,1000,3.9f); 
    liftCmd = {-30, 359, 600};
    liftGo = true;

    if(StopFlag == 2){
        // 通知后台任务（autoSubsystems/debugTask）退出，避免进入手动阶段后仍在轮询/刷屏
        autoActive = false;
        pros::delay(100);
        return;
    }

    turnSettings.angularPID = lemlib::PID(1.44, 0.0, 0.085);
    lemlib::turnTo(-135_stDeg, 900_msec, turnParams, turnSettings);
    ClawOpen();
    ClawIntake();
    GoForWardCurve(1.0,19.7,1600,8.0f);
    pros::delay(100);
    ClawClose();
    //ClawStopIntake();
    liftCmd = {90, 305, 1000};
    liftGo = true;

    if(StopFlag == 3){//30分
        GoForWardCurve(1.0,3.2,500,1.0f);
        liftCmd = {40, 305, 300};
        liftGo = true;
        turnSettings.angularPID = lemlib::PID(1.3, 0.0, 0.106);
        lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
        GoForWardCurve(1.0,13.9,1000,8.0f); 
        left_motors.move(0.1);
        right_motors.move(0.1);
        LiftUpDegree(-30, 345, 1000);

        pros::delay(200);
        lift1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        lift2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        lift1.brake();
        lift2.brake();
        ClawOpen();
        pros::delay(100);

        /////
        left_motors.move(0);
        right_motors.move(0);
        GoForWardCurve(1.0,-7.1,1000,3.9f); 
        ClawClose();
    

        
    } else {//AWP
        pros::delay(100);
        GoForWardCurve(1.0,-16,1600,8.0f);
        turnSettings.angularPID = lemlib::PID(1.22, 0.0, 0.12);
        lemlib::turnTo(116.57_stDeg, 900_msec, turnParams, turnSettings);
        liftCmd = {40, 300, 500};
        liftGo = true;
        GoForWardCurve(1.0,33,2500,8.0f);
        left_motors.move(0.1);
        right_motors.move(0.1);
        LiftUpDegree(-30, 345, 1000);

        pros::delay(200);
        lift1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        lift2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        lift1.brake();
        lift2.brake();
        ClawOpen();
        pros::delay(100);
        left_motors.move(0);
        right_motors.move(0);
        GoForWardCurve(1.0,-3,1000,1.5f); 
        ClawClose();

    }
    // 通知后台任务（autoSubsystems/debugTask）退出，避免进入手动阶段后仍在轮询/刷屏
    autoActive = false;
    pros::delay(100);
    return;

/*
    ClawUP = true;
    clawGo = true;
    GoForWard(1.0,10.5,2000,pid);
    LiftUpDegree(-30, 359, 500);
    ClawOpen();
    pros::delay(60);
    GoForWard(1.0,-4.2,1200,pidShort);
    turnSettings.angularPID = lemlib::PID(1.13, 0.0, 0.06);
    lemlib::turnTo(135_stDeg, 800_msec, turnParams, turnSettings);
    liftCmd = {-80, 359, 600};
    liftGo = true;
    GoForWard(1.0,5.1,1200,pidShort);//
    ClawClose();
    liftCmd = {60, 320, 600};
    liftGo = true;
    GoForWard(1.0,9.7,1500,pid);
    turnSettings.angularPID = lemlib::PID(0.99, 0.0, 0.06);
    lemlib::turnTo(0_stDeg, 800_msec, turnParams, turnSettings);
    liftCmd = {60, 320, 250};
    liftGo = true;
    GoForWard(1.0,14,1000,pid);
    LiftUpDegree(-25, 359, 480);
    ClawOpen();
    pros::delay(60);
    GoForWard(1.0,-13.5,2000,pid);
    turnSettings.angularPID = lemlib::PID(1.52, 0.0, 0.04);
    lemlib::turnTo(45_stDeg, 800_msec, turnParams, turnSettings);
    liftCmd = {-50, 359, 600};
    liftGo = true;
    GoForWard(1.0,22.4,2200,pid);
    ClawClose();
    pros::delay(40);
    liftCmd = {60, 300, 800};
    liftGo = true;
    GoForWard(1.0,-22.3,2200,pid);

    //(Cpy)
    liftCmd = {80, 310, 600};
    liftGo = true;
    turnSettings.angularPID = lemlib::PID(1.5, 0.0, 0.04);
    lemlib::turnTo(0_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {70, 310, 340};
    liftGo = true;
    GoForWard(1.0,12.7,2000,pid);
    LiftUpDegree(-50, 330, 540);
    ClawOpen();
    pros::delay(60);
    //
    GoForWard(1.0,-32.5,120,pid);
    xMacroGo = true;  // 触发 X 键宏（归位+降底）
    turnSettings.angularPID = lemlib::PID(1.52, 0.0, 0.04);
    lemlib::turnTo(45_stDeg, 800_msec, turnParams, turnSettings);
    GoForWard(1.0,-30,800,pid);
    // 等后台 task 完成所有阻塞操作后再通知退出
    while (autoBusy) { pros::delay(10); }
    autoActive = false;
    pros::delay(100);
    return;
    

    





    

    params.reversed = true;
    lemlib::moveToPoint({-12.7_in, -3_in}, 850_msec, params, settings);
    liftCmd = {-33, 360, 500};
    liftGo = true;
    turnSettings.angularPID = lemlib::PID(1.16, 0.0, 0.04);
    lemlib::turnTo(135_stDeg, 900_msec, turnParams, turnSettings);

  
    params.reversed = false;
    //lemlib::moveToPose({-25.7_in, 13_in, 135_stDeg}, 2000_msec, Poseparams, Posesettings);
    lemlib::moveToPoint({-29.1_in, 13.4_in}, 1100_msec, params, settings);
    //-27.3,14.3
    ClawClose();
    pros::delay(50);
    liftCmd = {75, 305, 900};
    liftGo = true;
    params.reversed = true;
    lemlib::moveToPoint({-24.5_in, 8.8_in}, 1000_msec, params, settings);
    turnSettings.angularPID = lemlib::PID(1.05, 0.0, 0.068);
    lemlib::turnTo(45_stDeg, 800_msec, turnParams, turnSettings);
    params.reversed = false;
    lemlib::moveToPoint({-19.5_in, 13.8_in}, 700_msec, params, settings);
    LiftUpDegree(-37, 360, 400);
    ClawOpen();
    pros::delay(50);
    params.reversed = true;
    lemlib::moveToPoint({-24.5_in, 8.8_in}, 800_msec, params, settings);
    lemlib::turnTo(135_stDeg, 800_msec, turnParams, turnSettings);
    liftCmd = {-40, 360, 600};
    liftGo = true;
    
    params.reversed = false;
    lemlib::moveToPoint({-34.1_in, 18.4_in}, 800_msec, params, settings);

    turnSettings.angularPID = lemlib::PID(1.05, 0.0, 0.06);
    lemlib::turnTo(45_stDeg, 800_msec, turnParams, turnSettings);
    */
}
