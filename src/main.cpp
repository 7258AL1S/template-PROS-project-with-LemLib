#include "main.h"
#include "motorcontrol.h"
#include "sensor.h"
#include "api.h"
#include "lemlib/lemlib.hpp"
#include "auto.h"

/**
 * @brief LLEMU 中央按钮的回调函数
 *
 * 当此回调被触发时，将第 2 行 LCD 文本在 "I was pressed!" 和空白之间切换。
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * 运行初始化代码，在程序启动后立即执行。
 *
 * 初始化期间所有其他竞赛模式被阻塞；建议执行时间控制在数秒内。
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "AL-1S");

	pros::lcd::register_btn1_cb(on_center_button);
	left_motors.setBrakeMode(lemlib::BrakeMode::COAST);
	right_motors.setBrakeMode(lemlib::BrakeMode::COAST);


	lemLibInit(); // 初始化 LemLib（IMU 校准 + 里程计启动）
}

/**
 * 在机器人被禁用时运行（竞赛模式中 autonomous 或 opcontrol 之后）。
 * 当机器人被重新启用时，此 task 将退出。
 */
void disabled() {}

/**
 * 在 initialize() 之后、autonomous 之前运行（仅竞赛模式）。
 * 用于竞赛特定初始化，如 LCD 自动选择器。
 *
 * 机器人被启用并进入 autonomous 或 opcontrol 时，此 task 退出。
 */
void competition_initialize() {}



int auton = 1;// 选择自动程序（1、2、3...）
/**
 * 运行用户自动代码。此函数在独立 task 中以默认优先级和栈大小启动，
 * 每当机器人通过 FMS 或 VEX 竞赛开关在自动模式下被启用时调用。
 * 也可在 initialize 或 opcontrol 中调用以进行非竞赛测试。
 *
 * 如果机器人被禁用或通信中断，autonomous task 将被停止。
 * 重新启用将重启 task，而非从中断处恢复。
 */
void autonomous() {
	switch (auton) {
		case 1:
			auto1();
			break;
		default:
			break;
	}
}

/**
 * 运行操作手控代码。此函数在独立 task 中以默认优先级和栈大小启动，
 * 每当机器人通过 FMS 或 VEX 竞赛开关在手动模式下被启用时调用。
 *
 * 如未连接竞赛控制，此函数将在 initialize() 之后立即运行。
 *
 * 如果机器人被禁用或通信中断，opcontrol task 将被停止。
 * 重新启用将重启 task，而非从中断处恢复。
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	left_motors.setBrakeMode(lemlib::BrakeMode::COAST);
	right_motors.setBrakeMode(lemlib::BrakeMode::COAST);
//	left_motors.move(0);
//	right_motors.move(0);
	// 重置升降和夹爪水机状态，清除 autonomous 残留
	lift1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw_Rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw_return.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//lift1.move(0);
	//lift2.move(0);
//	Claw.move(0);
//	Claw_Rot.move(0);
//	Claw_return.move(0);


	while (true) {
		int dir   = master.get_analog(ANALOG_LEFT_Y);
		int turn  = master.get_analog(ANALOG_LEFT_X);
		int chR_Y = master.get_analog(ANALOG_RIGHT_Y);
		int chR_X = master.get_analog(ANALOG_RIGHT_X);
		int BtnL1 = master.get_digital(DIGITAL_L1);
		int BtnL2 = master.get_digital(DIGITAL_L2);
		int BtnR1 = master.get_digital(DIGITAL_R1);
		int BtnR2 = master.get_digital(DIGITAL_R2);
		int BtnA = master.get_digital(DIGITAL_A);
		int BtnX = master.get_digital(DIGITAL_X);
		int BtnB  = master.get_digital(DIGITAL_B);
		int BtnY  = master.get_digital(DIGITAL_Y);

		drive(dir, turn);


		int absX = abs(chR_X);
		int absY = abs(chR_Y);
		constexpr int kStickDead = 70;
		bool tuggleActive = (absX > kStickDead && absX >= absY);
		Piston_tuggle.set_value(tuggleActive);
		Piston_tuggle2.set_value(tuggleActive);
		PickControl(tuggleActive);
/*
		// 半自动宏
		static bool clawAt90 = false;
		static bool aWasActive = false;
		bool aActive = a_macro(BtnA, clawAt90);
		if (aWasActive && !aActive) { clawAt90 = true; }
		aWasActive = aActive;

		static bool xWasActive = false;
		bool xActive = x_macro(BtnX);
		if (xWasActive && !xActive) { clawAt90 = false; }
		xWasActive = xActive;
*/
		bool anyMacro = 0;

		// 升降（tuggle 激活时抑制升降输入，宏激活时由宏内部控制）
		if (!anyMacro) {
			Lift_simple(tuggleActive ? 0 : chR_Y);
		}

		// 爪子（宏激活时跳过）
		if (!anyMacro) {
			static bool clawOpen = false;  // 默认关闭
			static bool l1Prev   = false;
			bool l1Rising = (!l1Prev && BtnL1);
			l1Prev = BtnL1;
			if (l1Rising) clawOpen = !clawOpen;
			if (clawOpen) ClawOpenSimple();
			else          ClawCloseSimple();
		}

		//爪子Intake
		ClawControl(BtnR1, BtnY, BtnL1);
/*
		// 俯仰轴旋转（宏激活时跳过）
		if (!anyMacro) {
			static bool bPrev = false;
			bool bRising = (!bPrev && BtnB);
			bPrev = BtnB;
			if (bRising) clawAt90 = !clawAt90;
			turn_claw(clawAt90);
		}

		// 滚转轴（L2，不受宏影响）
		Claw_Return180_time(BtnL2);

		// 吸球（宏激活时跳过）
		if (!anyMacro) {
			IntakeControl(BtnR1, BtnR2, BtnY);
		}
*/
		pros::delay(20);
	}
}
