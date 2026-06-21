#include "auto.h"


void auto1() {
    Claw_Rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);// 旋转电机保持当前位置不动
    Claw_Rot.brake();
    // === 运动参数 ===
    lemlib::MoveToPoseParams Poseparams;
    lemlib::MoveToPoseSettings Posesettings;

    lemlib::MoveToPointParams params;
            // 倒车
    params.maxAngularSpeed = 0;      // 禁用转向
    lemlib::MoveToPointSettings settings;


lemlib::TurnToParams turnParams;
lemlib::TurnToSettings turnSettings;
    ClawClose();
    params.reversed = true;  
    lemlib::moveToPoint({-13_in, 0_in}, 2000_msec, params, settings);
    lemlib::turnTo(90_stDeg, 2000_msec, turnParams, turnSettings);
     //LiftUpDegree(70, 360, 300);
    params.reversed = false;   
    lemlib::moveToPoint({-13_in,6.7_in},1000_msec,params,settings);
    // === 自定义退出条件（倒车里程计可容忍更大误差）===
    LiftUpDegree(-40, 352, 800);
    pros::delay(150);
    ClawOpen();
    
    params.reversed = true;   
    lemlib::moveToPoint({-13_in,3_in},1000_msec,params,settings);
    //LiftUpDegree(-60, 300, 700);
    //lemlib::turnTo(-90_stDeg, 2000_msec, turnParams, turnSettings);
    // 倒车 14.2 英寸，超时 2 秒
    

    // === 调试：打印里程计坐标 ===
    auto pose = pose_getter();
    pros::lcd::print(3, "X: %.1f in", to_in(pose.x));
    pros::lcd::print(4, "Y: %.1f in", to_in(pose.y));
    pros::lcd::print(5, "H: %.0f deg", to_stDeg(pose.orientation));
    pros::delay(2000);  // 保持屏幕 2 秒便于读数

    // LiftUpDegree(-40, 340, 300);
    // ClawClose();
    // lemlib::turnTo(-90_stDeg, 2000_msec, {}, {});
}