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

/**
 * @brief 升降机构 PID 定位（非阻塞，需每帧调用）
 * @param targetDeg 目标角度（度），自动归一化到 [0,360) 并钳位到可达范围
 *
 * 单组 PID（kP=3.5 kD=10），带目标切换检测、积分抗饱和、
 * 最小输出保底（远端≥15）和到位 HOLD 死区。
 * 适用于 autonomous() 和 opcontrol() 中的自动定位。
 */
void lift_go(float targetDeg);

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


/**
 * @brief 爪子定时控制（单键切换 + 脉冲时序）
 * @param BtnPressed 按键按下状态 (1=按下, 0=松开)
 *

 */
void Claw_Turn(int BtnPressed);

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
 * @brief 爪子开闭控制（单次全功率 + 保持）
 * ClawOpen() → 全功率打开 200ms → HOLD 刹车
 * ClawClose() → 全功率关闭 200ms → 低功率保持
 */
void ClawOpen();
void ClawClose();


/**
 * @brief 爪子控制_气动
 * @param BtnPressed 按键按下状态
 */
void ClawControl(bool BtnPressed);
/**
 * @brief 夹爪旋转至 90°（正转，堵转检测 + HOLD 刹停）
 */
void Claw_Turn90();

/**
 * @brief 夹爪旋转至 0°（反转，堵转检测 + HOLD 刹停）
 */
void Claw_Turn0();

/**
 * @brief   夹爪滚转轴旋转控制（单键切换 0°/180° + 双向堵转检测）
 * @param btn 按键原始状态 (1=按下, 0=松开)
 *
 * 状态机：每次按键上升沿在 0° 和 180° 之间切换。
 * 两侧均有物理限位，通过编码器堵转检测判定到位并 HOLD 刹停。
 */
void Claw_Return180(int btn);



/**
 * @brief   吸球电机控制（前 + 后）
 * @param BtnPressed 按键按下状态
 */
void IntakeAll(int BtnPressed);
/**
 * @brief   吸球电机控制（仅前）
 * @param BtnPressed 按键按下状态
 */
void IntakeFrontOnly(int BtnPressed);

/**
 * @brief   吸球电机控制
 * @param BtnIntakeAll R1 按键按下状态
 * @param BtnIntakeFront R2 按键按下状态
 * @param BtnIntakeReverse Y 按键按下状态
 */
void IntakeControl(int BtnIntakeAll, int BtnIntakeFront,int BtnIntakeReverse);

/**
 * @brief   吸球电机倒退
 * @param BtnPressed 按键按下状态
 */
void IntakeReverse(int BtnPressed);

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

