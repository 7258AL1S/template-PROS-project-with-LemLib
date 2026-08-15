#pragma once

// ============================================================
// 自动程序公共后台任务状态与接口
// ============================================================

// 升降机构后台指令参数（对应 LiftUpDegree 的三个 float 参数）
struct LiftParams {
    float Power;
    float Target;
    float Fulltime;
};

// 公共后台状态：由自动主流程写入，后台任务轮询执行
extern LiftParams liftCmd;    // 升降指令
extern bool      liftGo;      // true = 触发升降
extern bool      ClawUP;      // false=收回(Turn0)，true=伸出(Turn90)
extern bool      clawGo;      // true = 触发夹爪旋转
extern bool      xMacroGo;    // true = 触发 X 键宏（归位/降底）
extern bool      autoActive;  // true = 自动进行中，后台任务据此退出
extern bool      autoBusy;    // true = 后台任务正在执行阻塞操作

// 重置所有共享状态，供每次自动程序开始时调用
void resetAutoBackgroundState();

// 启动公共后台任务：subsystems + debug
void startAutoBackgroundTasks();

// 后台任务函数
void autoSubsystems();
void autoDebug();
