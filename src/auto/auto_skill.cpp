#include "auto.h"

void auto_skill(int StopFlag) {
    // 保持与现有自动程序相同的后台任务生命周期；路线内容可在此继续补充。
    (void)StopFlag;
    startAutoBackgroundTasks();
    lemlib::TurnToParams turnParams;
    lemlib::TurnToSettings turnSettings;


    //GoForWardCurve(1.0,0.3,1000,0.5f); 
    turnSettings.angularPID = lemlib::PID(1.51, 0.0, 0.1);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    ClawIntake();
    GoForWardCurve(1.0,7,1000,3.9f); 
    pros::delay(500);//吸第一个导入
    
    GoForWardCurve(1.0,-5.0,1000,3.9f); 
    turnSettings.angularPID = lemlib::PID(1.28, 0.0, 0.1);
    lemlib::turnTo(0_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {50, 270, 1000};
    liftGo = true;
    pros::delay(400);
    GoForWardCurve(0.8,16.7,1000,8.0f); 
    ClawStopIntake();
    left_motors.move(0.1);
    right_motors.move(0.1);
    LiftUpDegree(-30, 345, 1000);//放第一个导入
    ClawOpen();

    liftCmd = {70, 359, 1000};
    liftGo = true;
    GoForWardCurve(1.0,-16.0,1000,8.0f);
    turnSettings.angularPID = lemlib::PID(1.2, 0.0, 0.1);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    ClawClose();
    ClawIntake();
    GoForWardCurve(1.0,6,1000,3.9f); 
    pros::delay(500);//吸第二个导入
    
    GoForWardCurve(1.0,-5.0,1000,3.9f); 
    turnSettings.angularPID = lemlib::PID(1.28, 0.0, 0.1);
    lemlib::turnTo(0_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {50, 260, 1200};
    liftGo = true;
    pros::delay(400);
    GoForWardCurve(0.8,16.7,1000,8.6f); 
    ClawStopIntake();
    left_motors.move(0.1);
    right_motors.move(0.1);
    LiftUpDegree(-30, 345, 1000);//放第二个导入
    ClawOpen();

    liftCmd = {70, 359, 1000};
    liftGo = true;
    GoForWardCurve(1.0,-16.0,1000,8.0f);
    turnSettings.angularPID = lemlib::PID(1.2, 0.0, 0.1);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    ClawClose();
    ClawIntake();
    GoForWardCurve(1.0,6,1000,3.9f); 
    pros::delay(500);//吸第三个导入
    
    GoForWardCurve(1.0,-5.0,1000,3.9f); 
    turnSettings.angularPID = lemlib::PID(1.28, 0.0, 0.1);
    lemlib::turnTo(0_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {50, 260, 1200};
    liftGo = true;
    pros::delay(400);
    GoForWardCurve(0.8,16.7,1000,8.6f); 
    ClawStopIntake();
    left_motors.move(0.1);
    right_motors.move(0.1);
    LiftUpDegree(-30, 345, 1000);//放第三个导入
    ClawOpen();

    autoActive = false;
    pros::delay(100);
}
