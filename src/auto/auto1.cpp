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

    ClawClose();
    params.reversed = true;
    lemlib::moveToPoint({-12.7_in, 0_in}, 1000_msec, params, settings);

    liftCmd = {60, 340, 500};
    liftGo = true;
    lemlib::turnTo(90_stDeg, 900_msec, turnParams, turnSettings);
    
    ClawUP = true;
    clawGo = true;

    params.reversed = false;
    lemlib::moveToPoint({-12.7_in, 11.5_in}, 1000_msec, params, settings);

    LiftUpDegree(-33, 352, 500);
    pros::delay(200);
    ClawOpen();
    pros::delay(200);

    params.reversed = true;
    lemlib::moveToPoint({-12.7_in, -2_in}, 1000_msec, params, settings);
    liftCmd = {-33, 360, 500};
    liftGo = true;
    turnSettings.angularPID = lemlib::PID(1.3, 0.0, 0.05);
    lemlib::turnTo(135_stDeg, 1000_msec, turnParams, turnSettings);
    params.reversed = false;
    //lemlib::moveToPose({-25.7_in, 13_in, 135_stDeg}, 2000_msec, Poseparams, Posesettings);
    lemlib::moveToPoint({-28.5_in, 13.8_in}, 2500_msec, params, settings);

    ClawClose();

}