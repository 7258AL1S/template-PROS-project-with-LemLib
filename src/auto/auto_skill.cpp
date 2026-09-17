#include "auto.h"

void auto_skill(int StopFlag) {
    // 保持与现有自动程序相同的后台任务生命周期；路线内容可在此继续补充。
    (void)StopFlag;
    startAutoBackgroundTasks();
    lemlib::TurnToParams turnParams;
    lemlib::TurnToSettings turnSettings90;
    lemlib::TurnToSettings turnSettings0;

    struct ImportCycle {
        LiftParams liftToScore;
        LiftParams liftToReset;
        bool useIntermediateRetreat;
    };

    const ImportCycle importCycles[] = {
        {{50, 270, 1000}, {-90, 359, 2000}, false},
        {{50, 270, 1000}, {-100, 359, 2500}, true},
        {{55, 260, 1200}, {-100, 359, 2500}, true},
    };


    const int IntakeLaser = 180;// 吸入导入的距离
    const int PutLaser = 395; //放导入的距离
    turnSettings90.angularPID = lemlib::PID(1.51, 0.0, 0.1);//贴墙转90的pid
    turnSettings0.angularPID = lemlib::PID(1.3, 0.0, 0.1);//取完导入转90的pid

    // 前三个导入路线相同，仅升降参数和第一次回撤步骤不同。
    for (const ImportCycle& cycle : importCycles) {
        lemlib::turnTo(90_stDeg, 1500_msec, turnParams, turnSettings90);
        ClawOpen();
        ClawIntake();
        LaserGoForWardCurve(0.8, IntakeLaser, 1200, 300.0f);
        pros::delay(500);
        ClawClose();
        LaserGoForWardCurve(0.8, PutLaser, 1200, 300.0f);

        lemlib::turnTo(0_stDeg, 1200_msec, turnParams, turnSettings0);
        liftCmd = cycle.liftToScore;
        liftGo = true;
        pros::delay(400);
        GoForWardCurve(0.5, 24, 2000, 8.0f);
        ClawStopIntake();
        left_motors.move(0.1);
        right_motors.move(0.1);
        pros::delay(100);
        LiftUpDegree(-30, 345, 1000);
        ClawOpen();

        liftCmd = cycle.liftToReset;
        liftGo = true;
        if (cycle.useIntermediateRetreat) {
            GoForWardCurve(0.5, -10.0, 1200, 10.0f);
            pros::delay(400);
        }
        GoForWardCurve(0.5, -35.0, 2000, 10.0f);
        left_motors.move(-0.2);
        right_motors.move(-0.2);
        pros::delay(400);
    }

    ////////////////倒填////////////////
    lemlib::turnTo(90_stDeg, 1500_msec, turnParams, turnSettings90);
    ClawOpen();
    ClawIntake();
    LaserGoForWardCurve(0.8,IntakeLaser,1200,300.0f);
    pros::delay(500);//吸第四个导入
    ClawClose();
    LaserGoForWardCurve(0.8,PutLaser,1200,300.0f);
    lemlib::turnTo(-75_stDeg, 1200_msec, turnParams, turnSettings0);
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
