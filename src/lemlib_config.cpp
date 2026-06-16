#include "lemlib/config.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "main.h"

// ============================================================
// LemLib 全局配置变量
// ============================================================

// PID 控制器
const lemlib::PID angular_pid(2.0, 0.0, 10.0);    // 转向 PID
const lemlib::PID lateral_pid(5.0, 0.0, 15.0);    // 横向 PID

// 底盘电机组（端口与 sensor.cpp 保持一致）
// 左侧：正转端口 -10, 9；反转端口 -8
lemlib::MotorGroup left_motors({-10, 9, -8}, 360_rpm);
// 右侧：正转端口 1；反转端口 -2, 3
lemlib::MotorGroup right_motors({1, -2, -3}, 360_rpm);

// ============================================================
// IMU（惯性传感器）
// ============================================================
// 修改端口号为你 IMU 实际插入的 Smart Port
static lemlib::V5InertialSensor imu(11);

// ============================================================
// 定位轮 — 根据需要修改以下参数
// ============================================================

// --- 垂直定位轮（测量前后移动）---
// 轮径：用卡尺测量定位轮的实际直径
static const Length verticalWheelDiameter = 2.75_in;

// 偏置：定位轮到机器人追踪中心的距离
// 正数 = 在中心前方，负数 = 在中心后方，0 = 正好在中心
static const Length verticalWheelOffset = 0.0_in;

// 传动比：如果定位轮通过齿轮连接，填写传动比；直连则填 1
static const Number verticalGearRatio = 1.0;

// 编码器端口：V5 Rotation Sensor 插入的 Smart Port
// 端口号前加负号表示反转编码器方向
static lemlib::V5RotationSensor verticalEncoder(12);
static lemlib::TrackingWheel verticalWheel(&verticalEncoder, verticalWheelDiameter,
                                           verticalWheelOffset, verticalGearRatio);

// --- 水平定位轮（测量左右移动）---
// 如果没有水平定位轮，删掉下面 7 行，并把 odom 构造中的 horizontalWheels 改为 {}
static const Length horizontalWheelDiameter = 2.75_in;

// 偏置：正数 = 在中心左侧，负数 = 在中心右侧
static const Length horizontalWheelOffset = -3.0_in;

static const Number horizontalGearRatio = 1.0;

static lemlib::V5RotationSensor horizontalEncoder(13);
static lemlib::TrackingWheel horizontalWheel(&horizontalEncoder, horizontalWheelDiameter,
                                             horizontalWheelOffset, horizontalGearRatio);

// ============================================================
// 里程计实例
// ============================================================
// 构造参数：
//   (1) IMU 列表      — 用于获取朝向角
//   (2) 垂直定位轮列表 — 测量前后位移
//   (3) 水平定位轮列表 — 测量左右位移
// 如果没有水平定位轮，将 &horizontalWheel 删掉即可
static lemlib::TrackingWheelOdometry odom({&imu},               // IMU 列表
                                          {&verticalWheel},     // 垂直定位轮
                                          {&horizontalWheel});  // 水平定位轮

// 姿态获取回调 — LemLib 运动函数通过它获取机器人当前坐标
std::function<units::Pose()> pose_getter = [&]() { return odom.getPose(); };

// 退出条件（达到目标附近 250ms 视为完成）
const lemlib::ExitConditionGroup<AngleRange> angular_exit_conditions(
    std::vector{lemlib::ExitCondition<AngleRange>(5_stDeg, 250_msec)});
const lemlib::ExitConditionGroup<Length> lateral_exit_conditions(
    std::vector{lemlib::ExitCondition<Length>(0.5_in, 250_msec)});

// 底盘参数
const Length track_width = 12.0_in;   // 左右驱动轮距，根据实际测量调整
const Number drift_compensation = 0;  // 漂移补偿系数

// 速度变化率限制（0 = 无限制）
const Number angular_slew = 0;
const Number lateral_slew = 0;

// ============================================================
// LemLib 初始化（在 initialize() 中调用）
// ============================================================
void lemLibInit() {
    // 校准 IMU（需要 2~3 秒，期间机器人保持静止）
    imu.calibrate();
    while (imu.isCalibrating()) {
        pros::delay(10);
    }

    // 启动里程计追踪任务，每 10ms 更新一次
    odom.startTask(10_msec);
}
