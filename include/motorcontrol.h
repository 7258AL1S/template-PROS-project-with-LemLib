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
float Lift(int joystickValue);

/**
 * @brief 升降机构简化控制（直接摇杆映射 + 限位减速）
 * @param joystickValue 摇杆输入 [-127, 127]
 *                      正摇杆 → 上升，负摇杆 → 下降
 */
void Lift_simple(int joystickValue);


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


/**
 * @brief 底盘控制
 * @param dir  前进/后退量
 * @param turn 转向量
 */
void drive(int dir, int turn);

/**
 * @brief LemLib 里程计初始化（IMU 校准 + 定位轮启动）
 * 在 initialize() 中调用
 */
void lemLibInit();

