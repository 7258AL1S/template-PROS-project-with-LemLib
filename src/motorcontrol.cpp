#include "main.h"
#include "motorcontrol.h"
#include "sensor.h"
#include <cmath>



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
	if(Power == 0){
		lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		lift1.brake();
		lift2.brake();
	}
	else{
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


// ============================================================
// 爪子定时控制（单键 L1 切换夹紧/松开）
// 夹紧：全功率 200ms → 低功率保持
// 松开：全功率 200ms → HOLD 刹车
// ============================================================
constexpr uint32_t kPulseMs = 259;  // 全功率时长
constexpr int      kFull    = 100;  // 全功率
constexpr int      kHold    = 20;   // 保持功率
void Claw_control_time(int BtnPressed) {
	static uint32_t pulseStart = 0;     // 脉冲起始时间
	static bool     lastBtn     = false; // 上次按钮状态
	static bool     closed      = false; // 当前：夹紧/松开
	static bool     holdd      = true;  // 是否保持夹紧（松开后不保持）
	

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

	Claw_Rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw_Rot.brake();
}





void ClawOpen(){
	Claw.move(kFull);
	pros::delay(kPulseMs);
	Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw.brake();
}

void ClawClose(){
	Claw.move(-kFull);
	pros::delay(kPulseMs);
	Claw.move(-kHold);
}












const int kDeadzone = 10; // 摇杆死区阈值
//底盘电机控制
void drive(int dir,int turn){
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	
	if(fabs(dir) < kDeadzone && fabs(turn) < kDeadzone){// 前后死区 转向死区 ±10
		left_mg.brake();
		right_mg.brake();
	} else {
		left_mg.move(dir + turn);      // 设置左电机电压
		right_mg.move(dir - turn);     // 设置右电机电压
	} 
}