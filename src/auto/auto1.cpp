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
static bool      xMacroGo  = false;       // true = 触发 X 键宏（归位+降底）
static bool      autoActive = true;       // true = 自动进行中，task 用它退出
static bool      autoBusy   = false;      // true = task 正在执行阻塞操作

// 后台任务：轮询控制变量，执行升降 / 夹爪旋转（阻塞式，与底盘并行）
void autoSubsystems() {
    while (autoActive) {
        if (liftGo) {
            liftGo = false;
            autoBusy = true;
            LiftUpDegree(liftCmd.Power, liftCmd.Target, liftCmd.Fulltime);
            autoBusy = false;
        }
        if (clawGo) {
            clawGo = false;
            autoBusy = true;
            if (ClawUP) Claw_Turn90();
            else        Claw_Turn0();
            autoBusy = false;
        }
        if (xMacroGo) {
            xMacroGo = false;
            autoBusy = true;
            x_macro(1);                 // 模拟按键上升沿触发
            pros::delay(20);
            while (x_macro(0)) {        // 持续喂状态机直到完成
                pros::delay(20);
            }
            autoBusy = false;
        }
        pros::delay(10);
    }
}

// ============================================================
// 自动程序
// ============================================================
void auto1() {

    autoActive = true;  // 每次自动开始时重置标志位

    // static 防止 auto1 返回时析构干扰调度器（每次运行只跑一遍自动）
    static pros::Task subTask(autoSubsystems);
    static pros::Task debugTask([] {
        // 显示 GoForWard 自己用 horizontalEncoder 算出的位移，不依赖 LemLib 里程计
        pros::lcd::clear_line(5);
        while (autoActive) {
            pros::lcd::print(3, "Dist: %.1f in", GetWalkDist());
            pros::lcd::print(4, "Target: %.1f in", GetWalkTarget());
            pros::lcd::print(5, "IMU:%.1f", imu.getRotation().convert(deg));  // 陀螺仪航向，持续显示
            pros::delay(100);
        }
    });

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
   
    ClawClose();

    // GoForWard(1.0,-10.3,1000,pid);            // 旧版
    GoForWardCurve(1.0,10.3,1000,5.0f);        // 新版：Power, Target, FullTime, DecelDist


    pros::delay(100);
    liftCmd = {60, 330, 550};
    liftGo = true;
    //TurnCurve(1.0, 90.0, 800, 15.0);  
    turnSettings.angularPID = lemlib::PID(1.32, 0.0, 0.1);
    lemlib::turnTo(-90_stDeg, 900_msec, turnParams, turnSettings);
    return;


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
}
