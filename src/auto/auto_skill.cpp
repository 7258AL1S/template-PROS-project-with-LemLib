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
    ClawOpen();
    ClawIntake();
    LaserGoForWardCurve(0.8,180,1200,300.0f);
    pros::delay(500);//吸第一个导入
    ClawClose();
    LaserGoForWardCurve(0.8,425,1200,300.0f);

    turnSettings.angularPID = lemlib::PID(1.29, 0.0, 0.1);
    lemlib::turnTo(0_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {50, 270, 1000};
    liftGo = true;
    pros::delay(400);
    GoForWardCurve(0.5,23,2000,8.0f); 
    ClawStopIntake();
    left_motors.move(0.1);
    right_motors.move(0.1);
    pros::delay(100);
    LiftUpDegree(-30, 345, 1000);//放第一个导入
    ClawOpen();

    liftCmd = {-90, 359, 2000};
    liftGo = true;
    GoForWardCurve(0.5,-35.0,2000,10.0f);
    left_motors.move(-0.2);
    right_motors.move(-0.2);
    pros::delay(400);
    //turnSettings.angularPID = lemlib::PID(1.2, 0.0, 0.1);
    //lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    




    turnSettings.angularPID = lemlib::PID(1.51, 0.0, 0.1);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    ClawOpen();
    ClawIntake();
    LaserGoForWardCurve(0.8,180,1200,300.0f);
    pros::delay(500);//吸第二个导入
    ClawClose();
    LaserGoForWardCurve(0.8,425,1200,300.0f);

    turnSettings.angularPID = lemlib::PID(1.29, 0.0, 0.1);
    lemlib::turnTo(0_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {50, 270, 1000};
    liftGo = true;
    pros::delay(400);
    GoForWardCurve(0.5,23,2000,8.0f); 
    ClawStopIntake();
    left_motors.move(0.1);
    right_motors.move(0.1);
    pros::delay(100);
    LiftUpDegree(-30, 345, 1000);//放第二个导入
    ClawOpen();

    liftCmd = {-90, 359, 2000};
    liftGo = true;
    GoForWardCurve(0.5,-35.0,2000,10.0f);
    left_motors.move(-0.2);
    right_motors.move(-0.2);
    pros::delay(400);


    turnSettings.angularPID = lemlib::PID(1.51, 0.0, 0.1);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    ClawOpen();
    ClawIntake();
    LaserGoForWardCurve(0.8,180,1200,300.0f);
    pros::delay(500);//吸第三个导入
    ClawClose();
    LaserGoForWardCurve(0.8,425,1200,300.0f);

    turnSettings.angularPID = lemlib::PID(1.29, 0.0, 0.1);
    lemlib::turnTo(0_stDeg, 900_msec, turnParams, turnSettings);
    liftCmd = {55, 260, 1200};
    liftGo = true;
    pros::delay(400);
    GoForWardCurve(0.5,23,2000,8.0f); 
    ClawStopIntake();
    left_motors.move(0.1);
    right_motors.move(0.1);
    pros::delay(100);
    LiftUpDegree(-30, 345, 1000);//放第三个导入
    ClawOpen();

    liftCmd = {-100, 359, 2200};
    liftGo = true;
    GoForWardCurve(0.5,-35.0,2000,10.0f);
    left_motors.move(-0.2);
    right_motors.move(-0.2);
    pros::delay(400);



    ////////////////倒填////////////////
    turnSettings.angularPID = lemlib::PID(1.51, 0.0, 0.1);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    ClawOpen();
    ClawIntake();
    LaserGoForWardCurve(0.8,180,1200,300.0f);
    pros::delay(500);//吸第四个导入
    ClawClose();
    LaserGoForWardCurve(0.8,425,1200,300.0f);
    turnSettings.angularPID = lemlib::PID(1.29, 0.0, 0.1);
    lemlib::turnTo(-75_stDeg, 1200_msec, turnParams, turnSettings);
    liftCmd = {55, 340, 600};
    liftGo = true;
    GoForWardCurve(0.5,20.0,2000,10.0f);
    ClawStopIntake();
    left_motors.move(0.1);
    right_motors.move(0.1);
    pros::delay(100);
    LiftUpDegree(-30, 345, 1000);//放第四个导入
    ClawOpen();

    autoActive = false;
    pros::delay(100);
}
