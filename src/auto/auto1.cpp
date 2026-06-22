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
    //ClawUP = false;   // 预设夹爪状态
    //clawGo = true;   // 触发夹爪旋转
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
    lemlib::moveToPoint({-12.5_in, 0_in}, 1300_msec, params, settings);

    liftCmd = {60, 340, 500};
    liftGo = true;  // 触发升降

    lemlib::turnTo(90_stDeg, 1400_msec, turnParams, turnSettings);
    
    ClawUP = true;   
    clawGo = true;   // 触发夹爪旋转

    params.reversed = false;   
    lemlib::moveToPoint({-12.5_in,3.7_in},1000_msec,params,settings);

    // === 自定义退出条件（倒车里程计可容忍更大误差）===
    //liftCmd = {-40, 352, 800};
    //liftGo = true;  // 触发升降
    LiftUpDegree(-30, 357, 400);
    pros::delay(150);
    ClawOpen();
    
    params.reversed = true;   
    lemlib::moveToPoint({-12.5_in,3_in},1000_msec,params,settings);
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