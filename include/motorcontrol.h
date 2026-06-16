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

