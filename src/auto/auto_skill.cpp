#include "auto.h"

void auto_skill(int StopFlag) {
    // 保持与现有自动程序相同的后台任务生命周期；路线内容可在此继续补充。
    (void)StopFlag;
    startAutoBackgroundTasks();

    autoActive = false;
    pros::delay(100);
}
