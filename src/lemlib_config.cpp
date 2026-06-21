#include "lemlib/config.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "sensor.h"
#include "main.h"

// ============================================================
// LemLib 全局配置变量
// ============================================================

// PID 控制器
const lemlib::PID angular_pid(1.0, 0.0, 0);    // 转向 PID
const lemlib::PID lateral_pid(1.0, 0.0, 10);    // 横向 PID

// 底盘电机组（端口与 sensor.cpp 保持一致）
// 左侧：正转端口 -10, 9；反转端口 -8
lemlib::MotorGroup left_motors({-10, 9, -8}, 360_rpm);
// 右侧：正转端口 1；反转端口 -2, 3
lemlib::MotorGroup right_motors({1, -2, 3}, 360_rpm);

// ============================================================
// 定位轮参数
// ============================================================

// --- 垂直定位轮（测量前后移动）---
static const Length verticalWheelDiameter = 2_in;
// 偏置：正 = 中心前方，负 = 中心后方
static const Length verticalWheelOffset = -1.732_in;//4,4
static const Number verticalGearRatio = 1.0;

// --- 水平定位轮（测量左右移动）---
static const Length horizontalWheelDiameter = 2_in;
// 偏置：正 = 中心左侧，负 = 中心右侧
static const Length horizontalWheelOffset = -2.24_in;
static const Number horizontalGearRatio = 1.0;

// ============================================================
// 里程计（在 lemLibInit() 中初始化）
// ============================================================
static lemlib::TrackingWheelOdometry* odom = nullptr;

// 姿态获取回调 — 里程计初始化后才有值
const std::function<units::Pose()> pose_getter = []() { return odom->getPose(); };

// 退出条件
const lemlib::ExitConditionGroup<AngleRange> angular_exit_conditions(
    std::vector{lemlib::ExitCondition<AngleRange>(5_stDeg, 250_msec)});
const lemlib::ExitConditionGroup<Length> lateral_exit_conditions(
    std::vector{lemlib::ExitCondition<Length>(0.5_in, 250_msec)});

// 底盘参数
const Length track_width = 15.43_in;// 轮距（左右轮中心间距）
const Number drift_compensation = 0;// 漂移补偿系数（0~1，0 = 无补偿，1 = 完全补偿）
const Number angular_slew = 0;// 横向速度斜坡
const Number lateral_slew = 0;// 角速度斜坡

// ============================================================
// LemLib 初始化（在 initialize() 中调用）
// ============================================================
void lemLibInit() {
    // --- 注意：IMU 和编码器在 sensor.cpp 中定义，这里通过 extern 引用 ---

    // 校准 IMU（需要 2~3 秒，期间机器人保持静止）
    imu.calibrate();
    while (imu.isCalibrating()) {
        pros::delay(10);
    }

    // 底盘刹车模式设为 HOLD（主动保持位置），避免惯性滑行
    left_motors.setBrakeMode(lemlib::BrakeMode::HOLD);
    right_motors.setBrakeMode(lemlib::BrakeMode::HOLD);

    // 构建追踪轮（使用 sensor.cpp 中的编码器对象）
    static lemlib::TrackingWheel verticalWheel(&verticalEncoder, verticalWheelDiameter,
                                               verticalWheelOffset, verticalGearRatio);
    static lemlib::TrackingWheel horizontalWheel(&horizontalEncoder, horizontalWheelDiameter,
                                                 horizontalWheelOffset, horizontalGearRatio);

    // 构建里程计实例
    // 参数：(1) IMU列表 (2) 垂直定位轮 (3) 水平定位轮
    // 没有水平定位轮就把 &horizontalWheel 删掉，第三个参数改为 {}
    static lemlib::TrackingWheelOdometry odomInstance({&imu},
                                                      {&verticalWheel},
                                                      {&horizontalWheel});
    odom = &odomInstance;

    // 启动追踪任务，每 10ms 更新
    odom->startTask(10_msec);

    // 陀螺仪朝向归零
    pros::delay(50);
    odom->setPose({0_in, 0_in, 0_stDeg});

 
}
