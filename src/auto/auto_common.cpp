#include "auto.h"

// ============================================================
// 自动程序公共后台任务：定义
// ============================================================

LiftParams liftCmd = {0, 0, 1000};   // 升降指令
bool       liftGo    = false;        // true = 触发升降
bool       ClawUP    = false;        // false=收回(Turn0)，true=伸出(Turn90)
bool       clawGo    = false;        // true = 触发夹爪旋转
bool       xMacroGo  = false;        // true = 触发 X 键宏（归位/降底）
bool       autoActive = true;        // true = 自动进行中，后台任务据此退出
bool       autoBusy   = false;       // true = 后台任务正在执行阻塞操作

void resetAutoBackgroundState() {
    liftCmd    = {0, 0, 1000};
    liftGo     = false;
    ClawUP     = false;
    clawGo     = false;
    xMacroGo   = false;
    autoActive = true;
    autoBusy   = false;
}

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

// 后台任务：显示 GoForWard 位移、目标距离和 IMU 航向
void autoDebug() {
    pros::lcd::clear_line(5);
    while (autoActive) {
        pros::lcd::print(3, "Dist: %.1f in", GetWalkDist());
        pros::lcd::print(4, "Target: %.1f in", GetWalkTarget());
        pros::lcd::print(5, "IMU:%.1f", imu.getRotation().convert(deg));
        pros::delay(100);
    }
}

// 启动公共后台任务
// static 防止 auto 返回时析构干扰调度器（每次运行只跑一遍自动）
void startAutoBackgroundTasks() {
    resetAutoBackgroundState();

    static pros::Task subTask(autoSubsystems);
    static pros::Task debugTask(autoDebug);
}
