#include "main.h"
#include "motorcontrol.h"
#include "sensor.h"
#include <cmath>
#include "lemlib/config.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"




// ============================================================
// 六段 PID 参数（上升 3 段 + 下降 3 段）
// ============================================================
static const LiftPID LIFT_UP_1   = {4.0f, 0.0f, 10.0f};  // 上升·底段：起步大力但抑制震动
static const LiftPID LIFT_UP_2   = {2.5f, 0.0f, 12.0f};  // 上升·中段：匀速低震荡
static const LiftPID LIFT_UP_3   = {1.5f, 0.0f, 10.0f};  // 上升·顶段：低P低D防止顶端震荡
static const LiftPID LIFT_DOWN_1 = {2.0f, 0.0f, 22.0f};  // 下降·顶段：重力辅助，P小
static const LiftPID LIFT_DOWN_2 = {2.0f, 0.0f, 24.0f};  // 下降·中段
static const LiftPID LIFT_DOWN_3 = {1.5f, 0.0f, 26.0f};  // 下降·底段：接近底部缓停

// ============================================================
// Lift() — 升降机构控制（六段式 PID）
// ============================================================
float Lift_pid(int joystickValue) {
	static float lift_target  = 0;        // 目标角度
	static bool  target_init  = false;    // 首次初始化标志
	static float last_err[6]  = {0};      // 六段各自的 lastError
	static float acc_err[6]   = {0};      // 六段各自的积分

	// PROS Rotation 传感器返回厘度（centidegrees），除以 100 得到度
	float currentAngle = liftRotation.get_angle() / 100.0f;

	// === 首次上电：锁定当前位置为 target，避免猛冲 ===
	if (!target_init) {
		lift_target = currentAngle;
		// 当前位置在不可达区间 (0, 267) 时，夹到最近边界
		if (lift_target > 0.0f && lift_target < 267.0f) {
			lift_target = (lift_target < 133.5f) ? 0.0f : 267.0f;
		}
		target_init = true;
	}

	// === 摇杆调整 target ===
	// 正摇杆 → 上升（编码器减小）  → target 减小
	// 负摇杆 → 下降（编码器增大）  → target 增大
	const float increment = 0.003f; // 基础增量（0.03→0.003，10 倍精度提升）

	// 粘性边界：到顶或到底时推杆无效
	const bool atTop    = (lift_target == 267.0f);
	const bool atBottom = (lift_target == 0.0f);
	if (!(atTop && joystickValue > 10) && !(atBottom && joystickValue < -10)) {
		lift_target -= increment * joystickValue;
	}

	// target 归一化到 [0, 360)
	lift_target = std::fmod(lift_target, 360.0f);
	if (lift_target < 0.0f) lift_target += 360.0f;

	// 机械限位：可达范围 0° 和 [267°, 359°]
	if (lift_target > 0.0f && lift_target < 267.0f) {
		lift_target = (lift_target < 133.5f) ? 0.0f : 267.0f;
	} else if (lift_target >= 359.0f) {
		lift_target = 359.0f;
	}

	// === 计算误差 + 角度环绕修正（取最短路径） ===
	float err = lift_target - currentAngle;
	if (err > 180.0f)  err -= 360.0f;
	if (err < -180.0f) err += 360.0f;

	// === 当前段号 ===
	// 总行程 93°（底 0° ← 359°...267° 顶），每段 31°
	// distFromBottom = 0(底) ~ 93(顶)
	float distFromBottom = (currentAngle >= 267.0f) ? (360.0f - currentAngle) : 0.0f;
	int seg = static_cast<int>(distFromBottom / 31.0f);
	if (seg < 0) seg = 0;
	if (seg > 2) seg = 2;

	// === PID 计算 ===
	float out;
	if (err < 0.0f) {
		// ====== 上升：段 0(底)→段 1(中)→段 2(顶)，PID 索引 0,1,2 ======
		const int idx = seg;
		acc_err[3] = acc_err[4] = acc_err[5] = 0.0f; // 清零下降积分

		if (std::fabs(err) < 30.0f) acc_err[idx] += err;
		else                        acc_err[idx] = 0.0f;

		const LiftPID* upPids[3] = {&LIFT_UP_1, &LIFT_UP_2, &LIFT_UP_3};
		out = upPids[idx]->output(err, last_err[idx], acc_err[idx]);
		last_err[idx] = err;
	} else {
		// ====== 下降：段 2(顶)→段 1(中)→段 0(底)，PID 索引 5,4,3 ======
		const int idx = 3 + seg; // seg 0(底)→3, 1(中)→4, 2(顶)→5
		acc_err[0] = acc_err[1] = acc_err[2] = 0.0f; // 清零上升积分

		if (std::fabs(err) < 30.0f) acc_err[idx] += err;
		else                        acc_err[idx] = 0.0f;

		// 下降方向 PID：seg 0→底(DOWN_3), 1→中(DOWN_2), 2→顶(DOWN_1)
		const LiftPID* downPids[3] = {&LIFT_DOWN_3, &LIFT_DOWN_2, &LIFT_DOWN_1};
		out = downPids[seg]->output(err, last_err[idx], acc_err[idx]);
		last_err[idx] = err;

		// 接近底部平滑减速
		if (distFromBottom < 15.0f) {
			const float ratio = distFromBottom / 15.0f;
			const float scale = 0.4f + 0.6f * ratio;
			out *= scale * 1.4f;
		}
	}

	// === 输出限幅 ===
	const float maxOut = 100.0f;
	if (out >  maxOut) out =  maxOut;
	if (out < -maxOut) out = -maxOut;

	// === 死区 + 驱动电机 ===
	if (std::fabs(err) < 1.4f && std::fabs(out) < 17.0f) {
		// 已到位 → HOLD 锁死
		lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		lift1.move(0);
		lift2.move(0);
	} else {
		// VEXcode: 0.128 * out (伏特) → PROS: out * 128 (毫伏)
		lift1.move_voltage(static_cast<int32_t>(out * 128));
		lift2.move_voltage(static_cast<int32_t>(out * 128));
	}

	// 调试输出
	pros::lcd::print(2, "out: %.2f", out);

	return lift_target;
}




//无PID控制的简化LiftControl
void Lift_simple(int joystickValue) {
	// 读取当前升降角度（PROS Rotation 返回厘度，÷100 转度）
	float currentAngle = liftRotation.get_angle() / 100.0f;

	// 仅下降时接近底部 ±8°（含 359→0 环绕）：减速防撞底
	if (joystickValue < 0 && (currentAngle >= 346.0f || currentAngle <= 8.0f)) {
		joystickValue = static_cast<int>(joystickValue * 0.3f);
	}

	// ============================================================
	// 软刹车：松开摇杆后 150ms 内功率线性缓降到 0，再 HOLD 锁死
	// ============================================================
	static float     rampTargetPower = 0;       // 缓降起点功率
	static uint32_t  rampStartTime   = 0;       // 缓降开始时间戳
	static bool      ramping          = false;   // 是否正在缓降
	constexpr uint32_t kRampMs = 300;            // 缓降时长（毫秒）

	if (joystickValue > 25 || joystickValue < -25) {
		// ———— 摇杆活动区：正常驱动 ————
		float power = joystickValue * 0.8f;
		lift1.move(power);
		lift2.move(power);

		rampTargetPower = power;  // 记录当前功率，作为下次缓降起点
		ramping = false;
	} else {
		// ———— 摇杆死区：启动/继续缓降 ————
		if (!ramping) {
			ramping = true;
			rampStartTime = pros::millis();
		}

		uint32_t elapsed = pros::millis() - rampStartTime;
		if (elapsed < kRampMs) {
			// 线性缓降：rampTargetPower → 0
			/*
			float t = static_cast<float>(elapsed) / kRampMs;
			float power = rampTargetPower * (1.0f - t);
			lift1.move(power);
			lift2.move(power);
			*/
			lift1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			lift2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			lift1.brake();
			lift2.brake();
		} else {
			// 缓降完成 → HOLD 锁死
			lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lift1.brake();
			lift2.brake();
		}
	}
}


void Lift(float Power){
	static uint32_t zeroStartTime = 0;
	static bool     wasPowered    = false;

	if(Power == 0){
		if (wasPowered) {
			zeroStartTime = pros::millis();
			wasPowered = false;
		}

		uint32_t elapsed = pros::millis() - zeroStartTime;
		if (elapsed < 0) {
			// 前 150ms：BRAKE 模式（电阻制动，快速减速）
			lift1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			lift2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			lift1.brake();
			lift2.brake();
		} else {
			// 150ms 后：HOLD 模式（主动锁死位置）
			lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lift1.brake();
			lift2.brake();
		}
	}
	else{
		wasPowered = true;
		lift1.move(Power);
		lift2.move(Power);
	}
}


//Lift Autonomous

// ⚠️ 含阻塞循环，仅在 autonomous() 中使用；opcontrol() 请用 Lift_pid()
// 恒功率升降定位：指定功率朝着 Target 运行，±1° 到位或超时停止
void LiftUpDegree(float Power, float Target, float Fulltime) {
	// 机械零位偏移：传感器物理零位与逻辑零位差 20° Todo：测定实际偏移并调整
	constexpr float kOffset = 20.0f;
	Target += kOffset;

	// 当前角度（度），模 360 回绕到 [0, 360)
	float cur = std::fmod(liftRotation.get_angle() / 100.0f + kOffset, 360.0f);
	if (cur < 0.0f) cur += 360.0f;

	// 方向锁定：仅在循环前判定一次（若中途方向反转不会修正）
	bool isUp = (Target - cur) > 0.0f;

	// 超时计时起点
	uint32_t start = pros::millis();

	while (pros::millis() - start < static_cast<uint32_t>(Fulltime)) {
		pros::delay(10);  // 10ms 控制周期

		// 刷新当前角度
		cur = std::fmod(liftRotation.get_angle() / 100.0f + kOffset, 360.0f);
		if (cur < 0.0f) cur += 360.0f;

		float err = Target - cur;

		// ±1° 死区：进入即到位
		if (isUp) {
			if (err < 1.0f) break;    // 上升：越过目标
		} else {
			if (err > -1.0f) break;   // 下降：越过目标
		}

		Lift(Power);  // 恒功率驱动（无减速斜坡）
	}

	Lift(0);  // 到位或超时 → 刹车停止
}


// 非阻塞 PID 升降定位，需每帧调用
void lift_go(float targetDeg) {
	static float last_err  = 0;
	static float acc_err   = 0;
	static float prev_deg  = -999;  // 上一次目标，初始哨兵值用于首次检测

	// 当前角度（度），PROS Rotation 传感器返回厘度，÷100 转度
	float cur = liftRotation.get_angle() / 100.0f;

	// 目标归一化到 [0, 360)
	float deg = std::fmod(targetDeg, 360.0f);
	if (deg < 0.0f) deg += 360.0f;

	// 机械限位：可达范围 0° 和 [267°, 359°]
	if (deg > 0.0f && deg < 267.0f) {
		deg = (deg < 133.5f) ? 0.0f : 267.0f;
	} else if (deg >= 359.0f) {
		deg = 359.0f;
	}

	// 目标切换时重置 PID 状态，避免微分冲击
	if (std::fabs(deg - prev_deg) > 5.0f) {
		last_err = 0;
		acc_err  = 0;
	}
	prev_deg = deg;

	// 误差（deg - cur：正=升, 负=降）
	float err = -(deg - cur);

	// 传感器跨 0↔359 跳变时 reset last_err，防止 D 项巨大冲击
	if (std::fabs(err - last_err) > 180.0f) {
		last_err = err;
	}

	// PID 参数
	constexpr float kP = 2.5f;
	constexpr float kI = 0.0f;
	constexpr float kD = 10.0f;

	// 积分抗饱和：误差小时累积，大时清零
	if (std::fabs(err) < 25.0f) acc_err += err;
	else                        acc_err  = 0.0f;

	float out = kP * err + kI * acc_err + kD * (err - last_err);
	last_err = err;

	// 输出限幅
	const float maxOut = 100.0f;
	if (out >  maxOut) out =  maxOut;
	if (out < -maxOut) out = -maxOut;

	// 最低输出功率（远离目标 > 3° 时防太慢，逼近阶段交给 PID 自然衰减）
	const float minOut = 15.0f;
	if (std::fabs(err) > 3.0f) {
		if      (out > 0 && out <  minOut) out =  minOut;
		else if (out < 0 && out > -minOut) out = -minOut;
	}

	// 到位死区：HOLD 锁死
	if (std::fabs(err) < 0.4f && std::fabs(out) < 17.0f) {
		lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		lift1.brake();
		lift2.brake();
	} else {
		lift1.move_voltage(static_cast<int32_t>(out * 128));
		lift2.move_voltage(static_cast<int32_t>(out * 128));
	}
}


// ============================================================
// a_macro() — A 键一键宏（非阻塞状态机）
// 流程：lift_go(330°) → 到位 → turn_claw(true)
// ============================================================
bool a_macro(int btn, bool& clawAt90) {
	static bool     active   = false;
	static int      phase    = 0;
	static uint32_t t_start  = 0;
	static bool     prev_btn = false;

	// 上升沿触发
	if (!prev_btn && btn && !active) {
		active  = true;
		phase   = 0;
		t_start = pros::millis();
	}
	prev_btn = btn;

	if (!active) return false;

	uint32_t elapsed = pros::millis() - t_start;
	float    angle   = liftRotation.get_angle() / 100.0f;
	uint32_t now     = pros::millis();

	switch (phase) {
	case 0:  // 先升到 330°
		lift_go(330);
		{
			float err = std::fabs(angle - 330.0f);
			if (err > 180.0f) err = 360.0f - err;
			if ((err < 5.0f && elapsed > 300) || elapsed > 2500) {
				phase = 1; t_start = now;
			}
		}
		break;
	case 1:  // 升降到位后，夹子转出到 90°
		lift_go(330);
		
		break;
	}

	return true;
}


// ============================================================
// x_macro() — X 键一键宏（非阻塞状态机）
// 流程：turn_claw(true) → ClawOpen() → lift_go(0°) → 小功率压底
// ============================================================
bool x_macro(int btn) {
	static bool     active   = false;
	static int      phase    = 0;
	static uint32_t t_start  = 0;
	static uint32_t t_chk    = 0;
	static float    chk_pos  = 0;
	static bool     prev_btn = false;

	// 上升沿触发
	if (!prev_btn && btn && !active) {
		active  = true;
		phase   = 0;
		t_start = pros::millis();
	}
	prev_btn = btn;

	if (!active) return false;

	uint32_t elapsed = pros::millis() - t_start;
	float    angle   = liftRotation.get_angle() / 100.0f;
	uint32_t now     = pros::millis();

	switch (phase) {

	case 0:  // lift_go(0) 降到底
		lift_go(359);
		{
			float err = std::fabs(angle - 359.0f);
			if (err > 180.0f) err = 360.0f - err;
			if ((err < 2.0f && elapsed > 300) || elapsed > 2000) {
				phase   = 3;
				t_start = now;
				t_chk   = now;
				chk_pos = angle;
			}
		}
		break;
	case 1:  // 小功率压底 + 堵转检测
		turn_claw(false);
		if (now - t_chk >= 200) {
			float delta = std::fabs(angle - chk_pos);
			if (delta > 180.0f) delta = 360.0f - delta;
			if (delta < 3.0f || elapsed > 1000) {
				lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				lift1.brake();
				lift2.brake();
				active = false;
				return false;
			}
			t_chk   = now;
			chk_pos = angle;
		}
		Lift(-15);  // 小功率继续往下压
		break;
	}

	return true;
}































/*


//爪子（按下↔再按下切换 + 双向堵转检测）
// 状态机：每次 L1 按下（上升沿）在夹紧/松开之间切换
void Claw_control(int BtnPressed) {
	static bool     latched      = false;   // 闩锁状态：false=松开, true=夹紧
	static int      prevBtn      = 0;       // 上一次按钮状态（边缘检测）
	static bool     clampStalled = false;   // 夹紧方向堵转
	static bool     openStalled  = false;   // 松开方向堵转
	static uint32_t stallTimer   = 0;       // 堵转计时起点
	static double   lastPos      = 0;       // 上次编码器位置
	static uint32_t lastTime     = 0;       // 上次调用时间
	static bool     firstCall    = true;    // 首次调用初始化

	Claw_Rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);// 旋转电机保持当前位置不动
	Claw_Rot.brake();


	const uint32_t now = pros::millis();// 当前时间戳（毫秒）
	if (firstCall) {
		lastPos   = Claw.get_position();
		lastTime  = now;
		prevBtn   = BtnPressed;
		firstCall = false;
	}

	// === 上升沿检测：按钮从未按下→按下，切换闩锁 ===
	if (prevBtn == 0 && BtnPressed == 1) {
		latched = !latched;
	}
	prevBtn = BtnPressed;

	constexpr int     kPower       = 100;    // 基础功率
	constexpr double  kStallThresh = 0.4;   // 堵转速度阈值（deg/ms）
	constexpr uint32_t kStallTime  = 350;   // 持续堵转触发时间（ms）

	const double   currentPos = Claw.get_position();// 当前编码器位置（度）
	const uint32_t dt         = now - lastTime;
	const double   velocity   = (dt > 0) ? std::fabs(currentPos - lastPos) / dt : 0.0;// 计算速度（deg/ms）
	lastPos  = currentPos;
	lastTime = now;

	if (latched) {
		// ====== 夹紧方向 ======
		openStalled = false;

		if (clampStalled) {
			//Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			//Claw.brake();
			Claw.move(0.5 * kPower);
		} else {
			Claw.move(kPower);

			if (velocity < kStallThresh) {
				if (stallTimer == 0) stallTimer = now;
				else if (now - stallTimer > kStallTime) { clampStalled = true; stallTimer = 0; }
			} else {
				stallTimer = 0;
			}
		}
	} else {
		// ====== 松开方向 ======
		clampStalled = false;

		if (openStalled) {
			Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			Claw.brake();
		} else {
			Claw.move(-kPower);

			if (velocity < kStallThresh) {
				if (stallTimer == 0) stallTimer = now;
				else if (now - stallTimer > kStallTime) { openStalled = true; stallTimer = 0; }
			} else {
				stallTimer = 0;
			}
		}
	}
}

	constexpr uint32_t kPulseMs = 250;  // 全功率时长
	constexpr int      kFull    = 100;  // 全功率
	constexpr int      kHold    = 20;   // 保持功率

// ============================================================
// 爪子定时控制（单键 L1 切换夹紧/松开）
// 夹紧：全功率 200ms → 低功率保持
// 松开：全功率 200ms → HOLD 刹车
// ============================================================
constexpr uint32_t kPulseMs = 270;  // 全功率时长
constexpr int      kFull    = 100;  // 全功率
constexpr int      kHold    = 20;   // 保持功率
void Claw_control_time(int BtnPressed) {
	static uint32_t pulseStart = 0;     // 脉冲起始时间
	static bool     lastBtn     = false; // 上次按钮状态
	static bool     closed      = false; // 当前：夹紧/松开
	static bool     holdd      = false;

	// 上升沿：切换夹紧/松开
	if (!lastBtn && BtnPressed) {
		closed = !closed;
		 pulseStart = pros::millis();
		 }
	lastBtn = BtnPressed;

	uint32_t elapsed = pros::millis() - pulseStart;

	if (elapsed < kPulseMs) {
		// 全功率脉冲阶段
		Claw.move(closed ? kFull : -kFull);
	} else {
		// 脉冲结束
		if (!closed && holdd) Claw.move(-kHold);
		else {
			Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			Claw.brake(); 
		}
	}


}




*/
////以上为电机夹子控制函数，已废弃

// ============================================================
// ClawOpen / ClawClose — 计时器控制的电机夹子开关
// 正转 = 开，反转 = 关，满功率脉冲 250ms，之后 HOLD 锁死
// ============================================================
static bool     clawTargetOpen = false;   // 当前目标：true=开, false=关
static uint32_t clawPulseStart = 0;       // 脉冲起始时间
static bool     clawStateInit  = true;    // 首次调用标志
int ClawTime = 300; // 爪子开关脉冲时间（毫秒）

void ClawOpen(){
	if (!clawTargetOpen || clawStateInit) {
		clawTargetOpen = true;
		clawPulseStart = pros::millis();
		clawStateInit  = false;
	}

	if (pros::millis() - clawPulseStart < ClawTime) {
		Claw.move(127);  // 满功率正转 → 开
	} else {
		//Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		//Claw.brake();
		Claw.move(10);
	}
}


void ClawClose(){

	if (clawTargetOpen || clawStateInit) {
		clawTargetOpen = false;
		clawPulseStart = pros::millis();
		clawStateInit  = false;
	}

	if (pros::millis() - clawPulseStart < ClawTime) {
		Claw.move(-127);  // 满功率反转 → 关
	} else {
		Claw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Claw.brake();
	}
}

void ClawOpenSimple(){
	Claw.move(127);
}
void ClawCloseSimple(){
	Claw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Claw.brake();
}


void ClawIntake(){
	Claw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Claw_Left.move(70);
	Claw_Right.move(-70);
}

void ClawOuttake(){
	Claw_Left.move(-70);
	Claw_Right.move(70);
}



static bool clawOpen  = false;   // 当前目标：true=开, false=关
static bool prevClaw  = false;   // 上一帧按键状态
void ClawControl(bool IntakePressed,bool OuttakePressed,bool ClawPressed){
	Claw_Left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Claw_Right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	if(IntakePressed){
		ClawCloseSimple();
		clawOpen = false;
		ClawIntake();
	}
	else if(OuttakePressed){
		ClawOuttake();
	}
	else{
		Claw_Left.move(0);
		Claw_Right.move(0);
	}

	// 爪子：按下切换（上升沿 toggle）
	

	if (!prevClaw && ClawPressed) {
		clawOpen = !clawOpen;
	}
	prevClaw = ClawPressed;

	if (clawOpen) {
		ClawOpenSimple();
	} else {
		ClawCloseSimple();
	}
}

void PickControl(bool tuggleActive){
	
	if(tuggleActive){
		TugglePick.move(127);
	}
	else{
		TugglePick.move(0);
	}
}

void TugglePistonControl(bool BtnA){
	static bool pistonActive = false;  // 气缸状态：false=收, true=伸
	static bool prevA        = false;  // 上一帧 A 键状态（上升沿检测）

	// 上升沿：按下瞬间切换气缸状态
	if (!prevA && BtnA) {
		pistonActive = !pistonActive;
	}
	prevA = BtnA;

	Piston_tuggle.set_value(pistonActive);
	Piston_tuggle2.set_value(pistonActive);
}



void Claw_Turn(int btn){
	static bool     toggled   = false;
	static int      prevBtn   = 0;
	static bool     fwdStalled = false;  // 正转堵转
	static bool     revStalled = false;  // 反转堵转
	static double   lastPos   = 0;
	static uint32_t lastCheck = 0;

	// 上升沿切换
	if (prevBtn == 0 && btn == 1) {
		toggled = !toggled;
		fwdStalled = false;
		revStalled = false;
	}
	prevBtn = btn;

	uint32_t now = pros::millis();
	double curPos = Claw_Rot.get_position();

	// 每 250ms 检测一次编码器变化（< 3.0° 判定堵转，> 8.0° 才清除，迟滞防抖）
	if (now - lastCheck >= 250) {
		double delta = std::fabs(curPos - lastPos);
		if (delta < 3.0) {
			if (toggled)  fwdStalled = true;
			else          revStalled = true;
		} else if (delta > 8.0) {
			// 明显移动 → 清除堵转（迟滞带 3.0~8.0° 内维持原状态）
			if (toggled)  fwdStalled = false;
			else          revStalled = false;
		}
		lastPos   = curPos;
		lastCheck = now;
	}

	// 堵转后用 move_voltage 恒定电压顶住限位，避免 move() 速度指令触发 V5 内部堵转蜂鸣
	if (toggled) {
		if (fwdStalled) { Claw_Rot.move_voltage(1000);  }   // 恒定电压顶住正转限位
		else            { Claw_Rot.move(80); }
	} else {
		if (revStalled) { Claw_Rot.move_voltage(-1000); }   // 恒定电压顶住反转限位
		else            { Claw_Rot.move(-80); }
	}
}

// 俯仰轴旋转（目标驱动，非阻塞）
// target=true→90°(正转), false→0°(反转)
void turn_claw(bool target) {
	static bool     stallFwd    = false;
	static bool     stallRev    = false;
	static bool     prevTarget  = false;
	static double   lastPos     = 0;
	static uint32_t lastCheck   = 0;

	// 目标切换时重置堵转
	if (target != prevTarget) {
		stallFwd   = false;
		stallRev   = false;
		prevTarget = target;
		lastPos    = Claw_Rot.get_position();
		lastCheck  = pros::millis();
	}

	uint32_t now    = pros::millis();
	double   curPos = Claw_Rot.get_position();

	// 每 250ms 检测一次堵转（编码器变化 < 3.0° 判定堵转，> 8.0° 才清除堵转，迟滞防抖）
	if (now - lastCheck >= 250) {
		double delta = std::fabs(curPos - lastPos);
		if (delta < 3.0) {
			// 几乎未动 → 堵转
			if (target) stallFwd = true;
			else        stallRev = true;
		} else if (delta > 8.0) {
			// 明显移动 → 清除堵转（迟滞带 3.0~8.0° 内维持原状态）
			if (target) stallFwd = false;
			else        stallRev = false;
		}
		lastPos   = curPos;
		lastCheck = now;
	}

	// 堵转后用 move_voltage（恒定电压）顶住限位，避免 move() 速度指令触发 V5 内部堵转蜂鸣
	if (target) {
		if (stallFwd) { Claw_Rot.move_voltage(1000);  }
		else          { Claw_Rot.move(80); }
	} else {
		if (stallRev) { Claw_Rot.move_voltage(-1000); }
		else          { Claw_Rot.move(-80); }
	}
}

// 夹爪旋转至 90°（正转，堵转即停+HOLD）
void Claw_Turn90() {
	double lastPos = Claw_Rot.get_position();
	Claw_Rot.move(80);
	while (true) {
		pros::delay(200);
		double curPos = Claw_Rot.get_position();
		if (std::fabs(curPos - lastPos) < 5.0) break;
		lastPos = curPos;
	}
	Claw_Rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw_Rot.brake();
}

// 夹爪旋转至 0°（反转，堵转即停+HOLD）
void Claw_Turn0() {
	double lastPos = Claw_Rot.get_position();
	Claw_Rot.move(-80);
	while (true) {
		pros::delay(200);
		double curPos = Claw_Rot.get_position();
		if (std::fabs(curPos - lastPos) < 5.0) break;
		lastPos = curPos;
	}
	Claw_Rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw_Rot.brake();
}


void Claw_Return180(int btn) {
	static bool     toggled      = false;  // false=0°位置, true=180°位置
	static int      prevBtn      = 0;      // 上一帧按键状态（上升沿检测）
	static bool     stallAt180   = false;  // 180°方向堵转（碰到物理限位）
	static bool     stallAt0     = true;  // 0°方向堵转（碰到物理限位）
	static double   lastPos      = 0;
	static uint32_t lastCheck    = 0;

	// === 上升沿切换目标状态 ===
	if (prevBtn == 0 && btn == 1) {
		toggled    = !toggled;
		stallAt180 = false;
		stallAt0   = false;
	}
	prevBtn = btn;

	uint32_t now    = pros::millis();
	double   curPos = Claw_return.get_position();

	// === 每 250ms 检测一次堵转（< 3.0° 判定堵转，> 8.0° 才清除，迟滞防抖） ===
	if (now - lastCheck >= 250) {
		double delta = std::fabs(curPos - lastPos);
		if (delta < 8.0) {
			if (toggled) stallAt180 = true;
			else         stallAt0   = true;
		}
		lastPos   = curPos;
		lastCheck = now;
	}

	// === 驱动：堵转后用 move_voltage 恒定电压顶住，避免 move() 速度指令触发 V5 内部堵转蜂鸣 ===
	if (toggled) {
		// 目标：180°位置（电机反转）
		if (stallAt180) {
			Claw_return.move_voltage(-500);  // 恒定电压顶住180°限位
		} else {
			Claw_return.move(-80);
		}
	} else {
		// 目标：0°位置（电机正转）
		if (stallAt0) {
			Claw_return.move_voltage(500);   // 恒定电压顶住0°限位

		} else {
			Claw_return.move(80);
		}
	}
}

// ============================================================
// Claw_Return180_time — 计时版滚转轴（非阻塞状态机）
// 取消堵转检测，改为 500ms 全功率 + 400mV 保持
// ============================================================
void Claw_Return180_time(int btn) {
	static bool     toggled   = false;  // false=0°, true=180°
	static int      prevBtn   = 0;
	static bool     moving    = false;  // true=全功率移动中, false=保持中
	static uint32_t moveStart = 0;

	// 上升沿切换目标，启动移动
	if (prevBtn == 0 && btn == 1) {
		toggled   = !toggled;
		moving    = true;
		moveStart = pros::millis();
	}
	prevBtn = btn;

	uint32_t now     = pros::millis();
	uint32_t elapsed = now - moveStart;

	if (moving && elapsed < 800) {
		// 全功率移动
		if (toggled) Claw_return.move(-100);
		else         Claw_return.move(100);
	} else {
		// 时间到，切换为小电压保持
		moving = false;
		if (toggled) Claw_return.move_voltage(-400);
		else         Claw_return.move_voltage(400);
	}
}


void IntakeAll(int BtnPressed){
	if(BtnPressed){
		IntakeFront.move(100);
		IntakeBack.move(100);
	} else {
		IntakeFront.move(0);
		IntakeBack.move(0);
	}
}

void IntakeFrontOnly(int BtnPressed){
	if(BtnPressed){
		IntakeFront.move(100);
		IntakeBack.move(0);
	} else {
		IntakeFront.move(0);
		IntakeBack.move(0);
	}
}

void IntakeControl(int BtnIntakeAll, int BtnIntakeFront,int BtnIntakeReverse){
	if(BtnIntakeAll){
		IntakeFront.move(100);
		IntakeBack.move(100);
	} else if(BtnIntakeFront){
		IntakeFront.move(100);
		IntakeBack.move(0);
	} else if(BtnIntakeReverse){
		IntakeFront.move(-100);
		IntakeBack.move(-100);
	}
	else {
		IntakeFront.move(0);
		IntakeBack.move(0);
	}
}

void IntakeReverse(int BtnPressed){
	if(BtnPressed){
		IntakeFront.move(-100);
		IntakeBack.move(-100);
	} else {
		IntakeFront.move(0);
		IntakeBack.move(0);
	}
}



const int kDeadzone = 10; // 摇杆死区阈值
const float TurnScale = 1.0; // 转向缩放系数（可调节转向灵敏度，默认1.0）
void drive(int dir,int turn){
	turn = turn * TurnScale;
	// 摇杆 [-127,127] → LemLib move() 期望 [-1.0, +1.0]
	float leftPower  = (dir + turn) / 127.0f;
	float rightPower = (dir - turn) / 127.0f;
	// 钳位到 [-1.0, 1.0]（dir+turn 最大可达 ±254）
	if (leftPower  >  1.0f) leftPower  =  1.0f;
	if (leftPower  < -1.0f) leftPower  = -1.0f;
	if (rightPower >  1.0f) rightPower =  1.0f;
	if (rightPower < -1.0f) rightPower = -1.0f;
	if(fabs(dir) < kDeadzone && fabs(turn) < kDeadzone){// 前后死区 转向死区 ±10
		left_motors.brake();
		right_motors.brake();
	} else {
		left_motors.move(leftPower);      // 设置左电机电压
		right_motors.move(rightPower);     // 设置右电机电压
	} 
}

// ============================================================
// GoForWard — 竖直定位轮 + PID 闭环前进/后退
// ============================================================

// GoForWard 实时位移/目标（供 autonomous 屏幕调试显示，非阻塞读取）
static float walkDistDisplay   = 0.0f;  // 当前 GoForWard 已走距离（英寸）
static float walkTargetDisplay = 0.0f;  // 当前 GoForWard 目标距离（英寸）

float GetWalkDist() { return walkDistDisplay; }
float GetWalkTarget() { return walkTargetDisplay; }

void GoForWard(float Power, float Target, float FullTime, const LiftPID& pid) {
	// 竖直定位轮：horizontalEncoder 测量前进/后退（端口见 sensor.cpp）
	// 轮径 2 英寸，周长 = π × 2 ≈ 6.283 in
	constexpr float kWheelCircumference = 6.2831853f; // 2_in * π

	walkTargetDisplay = Target;  // 供屏幕显示当前段目标

	// --- 可调参数 ---
	constexpr uint32_t kRampTimeMs = 80;   // 软启动斜坡时长（毫秒）
	constexpr float    kRampStart  = 0.8f;  // 斜坡起始功率比例
	constexpr float    kMinPower   = 0.15f; // 最小功率保底（克服静摩擦，需实测标定）
	constexpr float    kStictionDegPerTick = 0.5f; // 堵转判定：每 10ms 编码器变化 < 此值=卡死

	// 记录起始编码器角度（度），不复位，不干扰里程计
	float startAngle = static_cast<float>(
		horizontalEncoder.getAngle().convert(deg));

	float err = 0, lastErr = 0, accErr = 0, out = 0;
	float absTarget = fabs(Target);
	uint32_t startTime = pros::millis();

	// 堵转检测用：上一拍的编码器角度
	float lastEncAngle = startAngle;

	// 运动中用 BRAKE 模式，到位后 brake() 有阻力不会溜车
	left_motors.setBrakeMode(lemlib::BrakeMode::BRAKE);
	right_motors.setBrakeMode(lemlib::BrakeMode::BRAKE);

	while (pros::millis() - startTime < FullTime) {
		// --- 更新当前位移（竖直定位轮变化量）---
		float curAngle = static_cast<float>(
			horizontalEncoder.getAngle().convert(deg));
		float deltaAngle = curAngle - startAngle;        // 编码器角度变化
		float curDist = deltaAngle / 360.0f * kWheelCircumference; // 转为英寸
		walkDistDisplay = curDist;  // 供屏幕显示实时位移

		// --- 误差计算 ---
		// Target > 0 前进，Target < 0 后退
		// curDist 前进时为正（编码器正转），后退时为负
		err = Target - curDist;

		// --- 分段积分（误差绝对值小于目标 10% 时积分，防止 windup）---
		if (fabs(err) < absTarget * 0.1f) {
			accErr += err;
		}

		// --- PID 输出 ---
		out = pid.output(err, lastErr, accErr);

		// --- 软启动斜坡 ---
		uint32_t elapsed = pros::millis() - startTime;
		if (elapsed < kRampTimeMs) {
			out *= (elapsed / static_cast<float>(kRampTimeMs)) * kRampStart;
		}

		// --- 功率限幅 ---
		if (fabs(out) > Power) {
			out = (Target > 0) ? Power : -Power;
		}

		// --- 静摩擦补偿：编码器不动 + PID 还要推 + 没到位 → 堵转，加力 ---
		float encDelta = fabs(curAngle - lastEncAngle);
		bool isStuck = (encDelta < kStictionDegPerTick);
		if (isStuck && fabs(out) > 0.001f && fabs(out) < kMinPower && fabs(err) > 0.3f) {
			out = (out > 0) ? kMinPower : -kMinPower;
		}

		// --- 驱动底盘 ---
		left_motors.move(out);
		right_motors.move(out);

		// --- 退出条件：到位（误差 < 0.3 inch ≈ 7.6mm）---
		if (fabs(err) < 0.3f) {

			break;
		}

		// --- 更新 ---
		lastErr = err;
		lastEncAngle = curAngle;
		pros::delay(10);
	}

	// 刹车

	left_motors.brake();
	right_motors.brake();
}

// ============================================================
// GoForWardCurve — 功率-距离曲线前进/后退（摩擦不敏感版，无 PID）
// ============================================================
void GoForWardCurve(float Power, float Target, float FullTime, float DecelDist) {
	// 竖直定位轮：horizontalEncoder 测量前进/后退（端口见 sensor.cpp）
	// 轮径 2 英寸，周长 = π × 2 ≈ 6.283 in
	constexpr float kWheelCircumference = 6.2831853f; // 2_in * π
	// 保底功率：实测该场地“刚能起动”的功率，摩擦大调高
	constexpr float kBreakawayPower = 0.15f;
	constexpr uint32_t kRampTimeMs = 100;  // 软启动斜坡时长（毫秒），减少起步打滑

	walkTargetDisplay = Target;  // 供屏幕显示当前段目标

	// 记录起始编码器角度（度），不复位，不干扰里程计
	float startAngle = static_cast<float>(horizontalEncoder.getAngle().convert(deg));
	uint32_t startTime = pros::millis();

	// 运动中用 BRAKE 模式，到位后 brake() 有阻力不会溜车
	left_motors.setBrakeMode(lemlib::BrakeMode::BRAKE);
	right_motors.setBrakeMode(lemlib::BrakeMode::BRAKE);

	const float dir       = (Target > 0) ? 1.0f : -1.0f;
	const float decel     = fabs(DecelDist); // 减速区长度（英寸）

	while (pros::millis() - startTime < FullTime) {
		// --- 更新当前位移（竖直定位轮变化量）---
		float curAngle = static_cast<float>(horizontalEncoder.getAngle().convert(deg));
		float curDist  = (curAngle - startAngle) / 360.0f * kWheelCircumference;
		walkDistDisplay = curDist;  // 供屏幕显示实时位移

		// --- 剩余距离 ---
		float absErr = fabs(Target - curDist);

		// --- 到位：直接刹停 ---
		if (absErr < 0.3f) {
			break;
		}

		// --- 功率-距离曲线 ---
		float out;
		if (absErr > decel) {
			out = dir * Power;  // 巡航段：满功率
		} else {
			// 减速段：按剩余距离线性降功率，但保留保底功率克服静摩擦
			out = dir * (Power * (absErr / decel));
			if (fabs(out) < kBreakawayPower) out = dir * kBreakawayPower;
		}

		// --- 软启动斜坡（前 kRampTimeMs 从 0 线性升到目标功率）---
		uint32_t elapsed = pros::millis() - startTime;
		if (elapsed < kRampTimeMs) {
			out *= (elapsed / static_cast<float>(kRampTimeMs));
		}

		// --- 驱动底盘 ---
		left_motors.move(out);
		right_motors.move(out);

		pros::delay(10);
	}

	// 刹车
	left_motors.brake();
	right_motors.brake();
}

// ============================================================
// TurnCurve — IMU 航向功率-角度曲线转向（无 PID，摩擦不敏感版）
// ============================================================
void TurnCurve(float Power, float Target, float FullTime, float DecelDeg) {
	// 相对转角：以进入函数时的航向为基准（与 GoForWardCurve 的"从起点算距离"语义一致），正=逆时针
	const float startAngle = static_cast<float>(imu.getRotation().convert(deg));
	const float target = startAngle + Target;

	// 转向方向系数：+1 = 左负右正（与 LemLib turnTo 一致，正角速度=逆时针）
	// 若实测转向方向反了，把这里改成 -1.0f 即可
	constexpr float kTurnSign = -1.0f;

	// 保底功率：实测"刚好能转动"的功率，摩擦力大调高
	constexpr float kBreakawayPower = 0.15f;
	constexpr uint32_t kRampTimeMs = 100;  // 软启动斜坡时长（毫秒），减少起步打滑

	// 减速区角度（度），防 0 值除零
	const float decel = (fabs(DecelDeg) > 0.0f) ? fabs(DecelDeg) : 1.0f;

	uint32_t startTime = pros::millis();
	uint32_t lastPrint = 0;

	// 调试输出：第 5 行显示陀螺仪（IMU 航向，度，精确到 0.1），与第 3/4 行 Dist/Target 同时显示
	pros::lcd::print(5, "IMU:%.1f", startAngle);

	// 转向过程中用 BRAKE 模式，到位后 brake() 有阻力不会滑过
	left_motors.setBrakeMode(lemlib::BrakeMode::BRAKE);
	right_motors.setBrakeMode(lemlib::BrakeMode::BRAKE);

	while (pros::millis() - startTime < FullTime) {
		// 当前航向角 + 环绕误差（0/359 取最短路径）
		float cur = static_cast<float>(imu.getRotation().convert(deg));
		float e = target - cur;
		if (e > 180.0f)  e -= 360.0f;
		if (e < -180.0f) e += 360.0f;
		float absErr = fabs(e);
		// 方向按当前误差实时决定：过冲后自动反向回正
		const float dir = (e >= 0.0f) ? 1.0f : -1.0f;

		// 功率-角度曲线
		float out;
		if (absErr > decel) {
			out = dir * Power;  // 巡航段：满功率
		} else {
			// 减速段：按剩余角度线性降功率，但保留保底功率克服静摩擦
			out = dir * (Power * (absErr / decel));
			if (fabs(out) < kBreakawayPower) out = dir * kBreakawayPower;
		}

		// 软启动斜坡（前 kRampTimeMs 从 0 线性升到目标功率）
		uint32_t elapsed = pros::millis() - startTime;
		if (elapsed < kRampTimeMs) {
			out *= (elapsed / static_cast<float>(kRampTimeMs));
		}

		// 调试输出：每 100ms 刷新一次第 5 行陀螺仪读数
		if (pros::millis() - lastPrint >= 100) {
			pros::lcd::print(5, "IMU:%.1f", cur);
			lastPrint = pros::millis();
		}

		// 到位：误差 < 0.5° 直接刹停
		if (absErr < 0.5f) {
			break;
		}

		// 转向：左右轮反向（与 LemLib turnTo 一致：正角速度=逆时针=左负右正）
		left_motors.move(kTurnSign * -out);
		right_motors.move(kTurnSign * out);

		pros::delay(10);
	}

	// 刹停
	left_motors.brake();
	right_motors.brake();
}
