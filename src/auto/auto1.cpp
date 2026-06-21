#include "auto.h"


void auto1() {
    // === 运动参数 ===
    lemlib::MoveToPointParams params;
    params.reversed = true;          // true = 倒车（保持朝向，反向行驶至目标）

    // === 运动设置（均使用 lemlib_config.cpp 中的全局默认值）===
    lemlib::MoveToPointSettings settings;
    // settings.angularPID      → angular_pid       (转向 PID)
    // settings.lateralPID      → lateral_pid       (横向 PID)
    // settings.exitConditions  → lateral_exit_conditions (0.5_in, 250ms)
    // settings.poseGetter      → pose_getter       (里程计回调)
    // settings.leftMotors      → left_motors       (左侧电机组)
    // settings.rightMotors     → right_motors      (右侧电机组)

    // 倒车 23 英寸（-X 方向），超时 2 秒
    lemlib::moveToPoint({-14.17_in, 0_in}, 2000_msec, params, settings);

    // 升降机构动作：角度 -50°，速度 340，超时 600ms
    LiftUpDegree(-50, 340, 600);
    // 爪子张开
    ClawOpen();
} 