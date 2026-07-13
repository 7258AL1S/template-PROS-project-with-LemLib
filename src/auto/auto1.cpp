#include "auto.h"

// ============================================================
// 子系统多线程控制变量
// ============================================================

// 升降机构参数（对应 LiftUpDegree 的三个 float 参数）
struct LiftParams {
    float Power;
    float Target;
    float Fulltime;
};

static LiftParams liftCmd  = {0, 0, 1000};  // 升降指令
static bool      liftGo    = false;       // true = 触发升降
static bool      ClawUP    = false;       // false=收回(Turn0), true=伸出(Turn90)
static bool      clawGo    = false;       // true = 触发夹爪旋转

// 后台任务：轮询控制变量，执行升降 / 夹爪旋转（阻塞式，与底盘并行）
void autoSubsystems() {
    while (true) {
        if (liftGo) {
            liftGo = false;
            LiftUpDegree(liftCmd.Power, liftCmd.Target, liftCmd.Fulltime);
        }
        if (clawGo) {
            clawGo = false;
            if (ClawUP) Claw_Turn90();
            else        Claw_Turn0();
        }
        pros::delay(10);
    }
}

// ============================================================
// 自动程序
// ============================================================
void auto1() {

    //left_motors.setBrakeMode(lemlib::BrakeMode::HOLD);
    //right_motors.setBrakeMode(lemlib::BrakeMode::HOLD);
    // 启动子系统后台任务（升降 + 夹爪旋转与底盘并行）
    pros::Task subTask(autoSubsystems);

    // 启动里程计实时调试任务
    pros::Task debugTask([] {
        while (true) {
            auto pose = pose_getter();
            pros::lcd::print(3, "X: %.1f in", to_in(pose.x));
            pros::lcd::print(4, "Y: %.1f in", to_in(pose.y));
            pros::lcd::print(5, "H: %.0f deg", to_stDeg(pose.orientation));
            pros::delay(100);
        }
    });

    Claw_Rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Claw_Rot.brake();

    // === 运动参数 ===
    lemlib::MoveToPoseParams Poseparams;
    lemlib::MoveToPoseSettings Posesettings;

    lemlib::MoveToPointParams params;
    params.maxAngularSpeed = 0;
    lemlib::MoveToPointSettings settings;

    lemlib::TurnToParams turnParams;
    lemlib::TurnToSettings turnSettings;
    Piston_tuggle.set_value(true);
   // pros::delay(200);
    Piston_tuggle.set_value(false);
   // pros::delay(200);
    Piston_tuggle.set_value(true);
   // pros::delay(200);
    Piston_tuggle.set_value(false);
    
    ClawClose();
    params.reversed = true;
    lemlib::moveToPoint({-12.7_in, 0_in}, 900_msec, params, settings);

    liftCmd = {60, 340, 500};
    liftGo = true;
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    
    ClawUP = true;
    clawGo = true;

    params.reversed = false;
    lemlib::moveToPoint({-12.7_in, 11.5_in}, 800_msec, params, settings);

    LiftUpDegree(-33, 352, 500);
    pros::delay(80);
    ClawOpen();
    pros::delay(80);

    params.reversed = true;
    lemlib::moveToPoint({-12.7_in, -3_in}, 850_msec, params, settings);
    liftCmd = {-33, 360, 500};
    liftGo = true;
    turnSettings.angularPID = lemlib::PID(1.42, 0.0, 0.06);
    lemlib::turnTo(135_stDeg, 900_msec, turnParams, turnSettings);

  
    params.reversed = false;
    //lemlib::moveToPose({-25.7_in, 13_in, 135_stDeg}, 2000_msec, Poseparams, Posesettings);
    lemlib::moveToPoint({-29.1_in, 13.4_in}, 1100_msec, params, settings);
    //-27.3,14.3
    ClawClose();
    pros::delay(50);
    liftCmd = {75, 305, 600};
    liftGo = true;
    params.reversed = true;
    lemlib::moveToPoint({-24.5_in, 8.8_in}, 1000_msec, params, settings);
    turnSettings.angularPID = lemlib::PID(1.05, 0.0, 0.068);
    lemlib::turnTo(45_stDeg, 800_msec, turnParams, turnSettings);
    params.reversed = false;
    lemlib::moveToPoint({-19.5_in, 13.8_in}, 700_msec, params, settings);
    LiftUpDegree(-37, 360, 400);
    pros::delay(60);
    ClawOpen();
    pros::delay(60);
    params.reversed = true;
    lemlib::moveToPoint({-24.5_in, 8.8_in}, 800_msec, params, settings);
    lemlib::turnTo(135_stDeg, 800_msec, turnParams, turnSettings);
    liftCmd = {-40, 360, 600};
    liftGo = true;
    
    params.reversed = false;
    lemlib::moveToPoint({-34.1_in, 18.4_in}, 800_msec, params, settings);

    turnSettings.angularPID = lemlib::PID(1.05, 0.0, 0.06);
    lemlib::turnTo(45_stDeg, 800_msec, turnParams, turnSettings);
}