#include "auto.h"

void auto_skill(int StopFlag) {
    // 保持与现有自动程序相同的后台任务生命周期；路线内容可在此继续补充。
    (void)StopFlag;
    auto1(5);

    liftCmd = {-85, 359, 2000};
    liftGo = true;

    GoForWardCurve(1.0, -19.7, 1200, 11.5f);
    ClawOpen();

    lemlib::TurnToParams turnParams;
    lemlib::TurnToSettings turnSettings90;
    lemlib::TurnToSettings turnSettings0;
    lemlib::TurnToSettings turnSettings45;
    lemlib::TurnToSettings turnSettingsDT;
    lemlib::TurnToSettings turnSettingsDZ;

    struct ImportCycle {
        LiftParams liftToScore;
        LiftParams liftToReset;
        bool useIntermediateRetreat;
    };

    const ImportCycle importCycles[] = {
        {{65, 275, 1000}, {-90, 359, 2000}, false},
        {{85, 258, 1100}, {-100, 359, 2500}, false},
        {{90, 250, 1200}, {-100, 359, 2500}, false},
    };

    const int IntakeLaser = 172;// 吸入导入的距离
    const int PutLaser = 369; //放导入的距离
    //所有转的p都加了0.5
    turnSettings90.angularPID = lemlib::PID(1.35, 0.0, 0.1);//转90的pid
    turnSettings0.angularPID = lemlib::PID(1.4, 0.0, 0.09);//取完导入转90的pid
    turnSettings45.angularPID = lemlib::PID(1.4, 0.0, 0.1);
    turnSettingsDT.angularPID = lemlib::PID(1.46, 0.0, 0.1);//倒填
    turnSettingsDZ.angularPID = lemlib::PID(2.5, 0.0, 0.1);//底座--没加0.5

    // 前三个导入路线相同，仅升降参数和第一次回撤步骤不同。
    for (const ImportCycle& cycle : importCycles) {
        lemlib::turnTo(180_stDeg, 1000_msec, turnParams, turnSettings90);
        ClawOpen();
        ClawIntake();
        LaserGoForWardCurveIMU(0.8, IntakeLaser, 1100, 300.0f);
        pros::delay(200);
        ClawClose();
        liftCmd = {30,350,300};
        liftGo = true;
        LaserGoForWardCurveIMU(0.8, PutLaser, 1100, 300.0f);
        ClawStopIntake();
        
        lemlib::turnTo(90_stDeg, 1000_msec, turnParams, turnSettings0);
        liftCmd = cycle.liftToScore;
        liftGo = true;
        pros::delay(300);
        GoForWardCurveIMU(0.6, 23.2, 2000, 10.0f);
        
        left_motors.move(0.27);
        right_motors.move(0.27);
        pros::delay(180);
        LiftUpDegree(-26, 342, 650);
        lift1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	    lift2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	    lift1.brake();
	    lift2.brake();
        pros::delay(50);
        ClawOpen();
        pros::delay(100);
        liftCmd = cycle.liftToReset;
        liftGo = true;
        
        GoForWardCurveIMU(1.0, -20.6, 1200, 11.5f);
       
        pros::delay(50);
    }

    lemlib::turnTo(180_stDeg, 1000_msec, turnParams, turnSettings90);
    //ClawOpen();
    //ClawClose();
    ClawIntake();
    LaserGoForWardCurve(0.8,IntakeLaser-10,1200,300.0f);
    pros::delay(280);//吸第四个导入
    ClawClose();
    pros::delay(60);
    /*
    //////////////停泊/////////////////78停泊
    LaserGoForWardCurve(0.8,PutLaser-60,1200,300.0f);

    lemlib::turnTo(50_stDeg, 1200_msec, turnParams, turnSettings45);
    liftCmd = {65, 300, 1100};
    liftGo = true;
    GoForWardCurveIMU(0.9,50.0,3000,30.0f);
    ClawStopIntake();
    left_motors.move(0.2);
    right_motors.move(0.2);
    pros::delay(200);
    LiftUpDegree(-55, 345, 1500);//放第四个导入
    ClawOpen();
    pros::delay(200);
    LiftUpDegree(30, 345, 200);
    GoForWardCurve(0.4,-4.0,1000,1.0f);
    LiftUpDegree(-50, 359, 1500);
    autoActive = false;
    pros::delay(100);
    return;
    */
    

    ////////////////倒填/////////////////
    liftCmd = {40, 344, 300};
    liftGo = true;

    LaserGoForWardCurveIMU(0.8,PutLaser+110,2000,300.0f);
    lemlib::turnTo(28_stDeg, 1000_msec, turnParams, turnSettingsDT);
    liftCmd = {55, 340, 600};
    liftGo = true;
    GoForWardCurveIMU(0.8,9.0,800,8.0f);
    ClawStopIntake();
    left_motors.move(0.3);
    right_motors.move(0.3);
    pros::delay(100);
    LiftUpDegree(-60, 353, 500);//放第四个导入
    pros::delay(100);
    ClawOpen();
    LiftUpDegree(30, 347, 250);
    
    pros::delay(200);




    GoForWardCurveIMU(0.8,-12.2,1200,10.0f);
    lemlib::turnTo(180_stDeg, 1000_msec, turnParams, turnSettings0);
    liftCmd = {-55, 359, 600};
    liftGo = true;
    ClawIntake();
    LaserGoForWardCurve(0.8,IntakeLaser-10,1500,300.0f);
    pros::delay(280);//吸第5个导入
    ClawClose();
    liftCmd = {70, 325, 900};
    liftGo = true;
    ClawRevise(false);
    LaserGoForWardCurveIMU(0.8,PutLaser+110,1600,300.0f);
    lemlib::turnTo(30_stDeg, 1200_msec, turnParams, turnSettingsDT);
    
    GoForWardCurveIMU(0.8,9.0,900,8.0f);
    ClawStopIntake();
    left_motors.move(0.3);
    right_motors.move(0.3);
    pros::delay(100);
    LiftUpDegree(-40, 345, 500);//放第5个导入
    ClawOpen();
    pros::delay(100);




//王姬路线//
    GoForWardCurveIMU(0.8,-12.4,1200,10.0f);
    lemlib::turnTo(180_stDeg, 1000_msec, turnParams, turnSettings0);
    liftCmd = {-55, 359, 600};
    liftGo = true;
    ClawIntake();
    LaserGoForWardCurve(0.8,IntakeLaser-7,1200,300.0f);
    pros::delay(250);//吸第6个导入
    ClawClose();

    GoForWardCurveIMU(1.0,-6,1200,10.0f);
    lemlib::turnTo(155_stDeg, 800_msec, turnParams, turnSettings90);
    GoForWardCurveIMU(1.0,-47,2300,5.0f);
    pros::delay(100);
    GoForWardCurveIMU(0.8,1.4,400,1.0f);
    lemlib::turnTo(90_stDeg, 1000_msec, turnParams, turnSettingsDZ);
    GoForWardCurveIMU(1.0,-5,600,2.0f);
    left_motors.move(-0.3);
    right_motors.move(-0.3);
    pros::delay(300);
    TugglePick.move(127);
    pros::delay(600);
    TugglePick.move(0);
    liftCmd = {70, 325, 900};
    liftGo = true;
    GoForWardCurveIMU(1.0,40,2300,5.0f);
    lemlib::turnTo(45_stDeg, 800_msec, turnParams, turnSettings90);
    GoForWardCurveIMU(1.0,12,1000,3.0f);
    LiftUpDegree(-55, 350, 1200);//中场放导入
    ClawOpen();
    return;






/////////////注释掉这一段就是88自动////////////////////////////////////////////////

    GoForWardCurveIMU(0.8,-12.3,1200,10.0f);
    lemlib::turnTo(180_stDeg, 1200_msec, turnParams, turnSettings0);
    liftCmd = {-55, 359, 600};
    liftGo = true;
    ClawIntake();
    LaserGoForWardCurve(0.8,IntakeLaser-10,1500,300.0f);
    pros::delay(250);//吸第6个导入
    ClawClose();
    liftCmd = {75, 265, 850};
    liftGo = true;
    ClawRevise(false);
    LaserGoForWardCurveIMU(0.8,PutLaser+110,1600,300.0f);
    lemlib::turnTo(30_stDeg, 1200_msec, turnParams, turnSettingsDT);
    
    GoForWardCurveIMU(0.8,9.0,900,8.0f);
    ClawStopIntake();
    left_motors.move(0.3);
    right_motors.move(0.3);
    pros::delay(100);
    LiftUpDegree(-40, 342, 550);//放第6个导入
    ClawOpen();
    pros::delay(100);

///////////////////////////////////////////////////////////////////////////////////


    GoForWardCurveIMU(1.0,-5,1000,2.0f);
    lemlib::turnTo(0_stDeg, 700_msec, turnParams, turnSettingsDZ);
    GoForWardCurveIMU(1.0,-10,1200,6.0f);
    liftCmd = {60, 359, 800};
    liftGo = true;
    lemlib::turnTo(38.5_stDeg, 800_msec, turnParams, turnSettings45);
    ClawClose();
    GoForWardCurve(1.0,46.0,2000,2.0f);
    //GoForWardCurve(1.0,-2.0,500,4.0f);
    lemlib::turnTo(90_stDeg, 800_msec, turnParams, turnSettings90);
    GoForWardCurve(1.0,-40.0,2000,2.0f);
    left_motors.move(-0.4);
    right_motors.move(-0.4);
    pros::delay(630);
    left_motors.move(-0.2);
    right_motors.move(-0.2);
    TugglePick.move(127);
    pros::delay(575);
    TugglePick.move(0);
    GoForWardCurve(1.0,40.0,2000,10.0f);

    /*外场通道去翻toggle()
    lemlib::turnTo(-20_stDeg, 800_msec, turnParams, turnSettingsDT);
    GoForWardCurve(1.0,50.0,3000,10.0f);
    GoForWardCurve(1.0,-4.0,400,2.0f);
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettingsDZ);
    left_motors.move(-0.8);
    right_motors.move(-0.8);
    pros::delay(550);
    left_motors.move(0);
    right_motors.move(0);
    TugglePick.move(127);
    pros::delay(700);
    TugglePick.move(0);
    
    GoForWardCurve(1.0,50.0,2000,10.0f);
*/
    /*88分内场翻toggle
    lemlib::turnTo(0_stDeg, 1200_msec, turnParams, turnSettingsDZ);
    GoForWardCurveIMU(1.0,-13.5,1200,6.0f);
    lemlib::turnTo(38_stDeg, 1200_msec, turnParams, turnSettings45);
    liftCmd = {55, 359, 500};
    liftGo = true;
    ClawClose();
    GoForWardCurve(1.0,40.0,2000,2.0f);
    //GoForWardCurve(1.0,-2.0,500,4.0f);
    lemlib::turnTo(90_stDeg, 1200_msec, turnParams, turnSettings90);
    GoForWardCurve(1.0,-40.0,2000,2.0f);
    left_motors.move(-0.4);
    right_motors.move(-0.4);
    pros::delay(250);
    TugglePick.move(127);
    pros::delay(680);
    TugglePick.move(0);
    GoForWardCurve(1.0,50.0,2000,10.0f);


    */
    autoActive = false;
    pros::delay(100);
    return;
}
