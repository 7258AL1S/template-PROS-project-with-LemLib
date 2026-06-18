#pragma once

#include "main.h"

// ============================================================
// 升降机构专用 PID（六段式：上升 3 段 + 下降 3 段）
// ============================================================
struct LiftPID {
    float kP;
    float kI;
    float kD;

    /**
     * @brief 计算 PID 输出
     * @param error     当前误差 (target - current)
     * @param lastError 上一次误差（用于 D 项差分）
     * @param accError  累积积分（已由外部做 anti-windup 处理）
     * @return PID 输出值
     */
    float output(float error, float lastError, float accError) const {
        return kP * error + kI * accError + kD * (error - lastError);
    }
};

// ============================================================
// 升降机构函数声明
// ============================================================

/**
 * @brief 升降机构控制（六段式 PID）
 * @param joystickValue 摇杆输入 [-127, 127]
 *                      正摇杆 → 上升(编码器减小)
 *                      负摇杆 → 下降(编码器增大)
 * @return 当前目标角度 (0° ~ 360°)
 */
float Lift_pid(int joystickValue);

/**
 * @brief 升降机构简化控制（直接摇杆映射 + 限位减速）
 * @param joystickValue 摇杆输入 [-127, 127]
 *                      正摇杆 → 上升，负摇杆 → 下降
 */
void Lift_simple(int joystickValue);

/**
 * @brief 升降机构控制（无 PID，直接映射）
 * @param Power 电机功率 [-127, 127]
 *              正值 → 上升，负值 → 下降
 */
void Lift(float Power);

/**
 * @brief 升降机构控制（定角度 + 定时+给定功率）
 * @param Power  电机功率 [-127, 127]
 * @param Target 目标角度 (0° ~ 360°)
 * @param Fulltime 全功率持续时间（毫秒）
 */
void LiftUpDegree(float Power,float Target,float Fulltime);

// ============================================================
// 爪子函数声明
// ============================================================


/**
 * @brief 爪子控制
 * @param BtnPressed 按键按下状态
 */
void Claw_control(int BtnPressed);

/**
 * @brief 爪子定时控制（单键切换 + 脉冲时序）
 * @param BtnPressed 按键按下状态 (1=按下, 0=松开)
 *
 * 时序：按键按下 → 全功率 200ms → 反向全功率 200ms → 低功率保持
 * 再次按下反向执行。
 */
void Claw_control_time(int BtnPressed);
<<<<<<< HEAD
=======

/**
 * @brief 底盘电机控制（Arcade Drive）
 * @param dir  前进/后退量 [-127, 127]
 * @param turn 左转/右转量 [-127, 127]
 */
void drive(int dir, int turn);


/**
 * @brief 爪子控制(时间控制)
 * @param BtnPressed 按键按下状态
 */
void Claw_control_time(int BtnPressed);

>>>>>>> a4e4232afc9e5402e61616348de9a65df263e3ca

/**
 * @brief 爪子开闭控制（单次全功率 + 保持）
 * ClawOpen() → 全功率打开 200ms → HOLD 刹车
 * ClawClose() → 全功率关闭 200ms → 低功率保持
 */
void ClawOpen();
void ClawClose();









/**
 * @brief 底盘电机控制（Arcade Drive）
 * @param dir  前进/后退量 [-127, 127]
 * @param turn 左转/右转量 [-127, 127]
 */
void drive(int dir, int turn);







/**
 * @brief LemLib 里程计初始化（IMU 校准 + 定位轮启动）
 * 在 initialize() 中调用
 */
void lemLibInit();

