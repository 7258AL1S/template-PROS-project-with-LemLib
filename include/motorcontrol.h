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
 * 单组 PID（kP=2.5 kD=10），带目标切换检测、积分抗饱和、
 * 最小输出保底（远端≥15）和到位 HOLD 死区。
 * 适用于 autonomous() 和 opcontrol() 中的自动定位。
 */
void lift_go(float targetDeg);

// ============================================================
// 半自动宏函数声明
// ============================================================

/**
 * @brief A 键一键宏（非阻塞状态机）
 * @param btn      按键原始状态 (1=按下, 0=松开)
 * @param clawAt90 [输出] 宏结束后设为 true，同步夹子俯仰状态
 * @return true=宏运行中, false=空闲
 *
 * 流程：lift_go(330°) → 到位 → turn_claw(true) 转出夹子
 */
bool a_macro(int btn, bool& clawAt90);

/**
 * @brief X 键一键宏（非阻塞状态机）
 * @param btn 按键原始状态 (1=按下, 0=松开)
 * @return true=宏运行中, false=空闲
 *
 * 流程：turn_claw(true) → ClawOpen() → lift_go(0°) → 小功率压底+堵转检测
 */
bool x_macro(int btn);

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
 * @brief 底盘锁：按住上键 HOLD，松开 COAST；锁定期间跳过正常 drive 输入
 * @param BtnPressed 上键原始状态 (1=按住, 0=松开)
 * @return true=锁定中，false=正常手动
 */
bool ChassisLock(int BtnPressed);


/**
 * @brief 爪子控制(时间控制)
 * @param BtnPressed 按键按下状态
 */
void Claw_control_time(int BtnPressed);


/**
 * @brief 爪子开闭控制（计时器 + 满功率脉冲 + HOLD 锁死）
 * ClawOpen() → 满功率正转(开) 250ms → HOLD 刹车
 * ClawClose() → 满功率反转(关) 250ms → COAST 刹车
 * ClawOpenSimple() → 满功率正转(开)
 * ClawCloseSimple() → COAST 刹车(关)
 * 通过共享文件作用域静态变量实现状态切换检测，每次目标切换时重置计时器。
 */
void ClawOpen();
void ClawClose();
void ClawOpenSimple();

void ClawCloseSimple();


void ClawIntake();
void ClawOuttake();
void ClawStopIntake();

/**
 * @brief 爪子控制_气动
 * @param BtnPressed 按键按下状态
 */
void ClawControl(bool IntakePressed,bool OuttakePressed,bool ClawPressed);
/**
 * @brief tuggle拨片控制
 * @param tuggleActive 拨片按下状态
 */
void PickControl(bool tuggleActive);

/**
 * @brief 拨片气缸控制（A 键切换，上升沿 toggle）
 * @param BtnA A 键按下状态
 */
void TugglePistonControl(bool BtnA);

/*
 * @brief 俯仰轴旋转（目标驱动，非阻塞）
 * @param target true=90°位置, false=0°位置
 *
 * 与 Claw_Turn 使用相同的堵转阈值（250ms / 3.0°）和保持功率（±7）。
 * 无内部边沿检测，由调用方管理 toggle 逻辑。供宏和手动控制共用。
 */
void turn_claw(bool target);

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
 * @brief   夹爪滚转轴旋转控制（计时版，非阻塞状态机）
 * @param btn 按键原始状态 (1=按下, 0=松开)
 *
 * 状态机：每次按键上升沿在 0° 和 180° 之间切换。
 * 取消堵转检测，改为 500ms 全功率 + 400mV 小电压保持。
 */
void Claw_Return180_time(int btn);



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







// ============================================================
// 底盘自动运动函数
// ============================================================

/**
 * @brief 前进/后退函数（竖直定位轮 + PID 闭环控制）
 * @param Power    最大功率绝对值 [0, 1.0]，方向由 Target 符号决定
 * @param Target   目标距离（英寸），正=前进，负=后退
 * @param FullTime 超时时间（毫秒），到时强制刹车退出
 * @param pid      PID 参数（使用 LiftPID 结构体）
 *
 * 读取竖直定位轮（horizontalEncoder）的编码器角度变化量，
 * 经轮径换算为英寸作为当前位置反馈，与目标距离比较得到误差，
 * 由 PID 计算输出功率驱动底盘。
 * 包含：软启动斜坡（前 300ms）、分段积分抗饱和、功率限幅、
 * 到位误差退出（< 0.3 in）和超时保护。
 *
 * 注意：此函数为阻塞式，仅用于 autonomous() 中，
 * 不得在 opcontrol() 循环内调用。
 */
void GoForWard(float Power, float Target, float FullTime, const LiftPID& pid);

/**
 * @brief 获取 GoForWard 实时已走距离（英寸，负=后退）
 * 供 autonomous 屏幕调试显示，非阻塞，可随时读取。
 */
float GetWalkDist();

/**
 * @brief 获取当前 GoForWard 的目标距离（英寸）
 */
float GetWalkTarget();

/**
 * @brief 功率-距离曲线前进/后退（摩擦不敏感版，无 PID）
 * @param Power     最大功率绝对值 [0, 1.0]，方向由 Target 符号决定
 * @param Target    目标距离（英寸），正=前进，负=后退
 * @param FullTime  超时时间（毫秒），到时强制刹车退出
 * @param DecelDist 减速区长度（英寸）：剩余距离小于该值时按比例线性降功率
 *
 * 读取竖直定位轮（horizontalEncoder）编码器角度，换算为英寸作为位置反馈；
 * 剩余距离 > DecelDist 时满功率巡航，进入减速区后按 剩余距离/DecelDist
 * 线性降功率，并在低功率段保留保底功率克服静摩擦。
 * 不含 PID，摩擦敏感性远低于 GoForWard。
 * 到位窗口 1.0in；含过冲保护：误差一旦越过目标立即刹停，不会反向加速冲出。
 *
 * 注意：此函数为阻塞式，仅用于 autonomous() 中，
 * 不得在 opcontrol() 循环内调用。
 */
void GoForWardCurve(float Power, float Target, float FullTime, float DecelDist);

/**
 * @brief 功率-角度曲线转向（IMU 航向反馈，无 PID，摩擦不敏感版）
 * @param Power    最大功率绝对值 [0, 1.0]，方向自动取最短路径
 * @param Target   相对转角（度），正=逆时针，以进入函数时的航向为基准
 * @param FullTime 超时时间（毫秒），到时强制刹车退出
 * @param DecelDeg 减速区角度（度）：剩余误差小于该值时按比例线性降功率
 *
 * 读取 IMU 航向角（imu.getRotation）作为反馈，以进入函数时的航向为基准
 * 转过 Target 度（与 GoForWardCurve 的距离语义一致）；剩余误差 > DecelDeg
 * 时满功率巡航，进入减速区后按 剩余误差/DecelDeg 线性降功率，并保留保底
 * 功率克服静摩擦。不含 PID；方向按当前误差实时决定，过冲后可自行回正。
 * 若需绝对航向，调用时传 (目标航向 - 当前航向)。
 *
 * 注意：此函数为阻塞式，仅用于 autonomous() 中，
 * 不得在 opcontrol() 循环内调用。
 */
void TurnCurve(float Power, float Target, float FullTime, float DecelDeg);

/*
 * @brief LemLib 里程计初始化（IMU 校准 + 定位轮启动）
 * 在 initialize() 中调用
 */
void lemLibInit();

