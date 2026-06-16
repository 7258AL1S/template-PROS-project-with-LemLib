#include "main.h"
#include "motorcontrol.h"
#include "sensor.h"
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

/**
 * 运行用户自动代码。此函数在独立 task 中以默认优先级和栈大小启动，
 * 每当机器人通过 FMS 或 VEX 竞赛开关在自动模式下被启用时调用。
 * 也可在 initialize 或 opcontrol 中调用以进行非竞赛测试。
 *
 * 如果机器人被禁用或通信中断，autonomous task 将被停止。
 * 重新启用将重启 task，而非从中断处恢复。
 */
void autonomous() {}

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
	


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // 打印模拟屏幕 LCD 按键状态

		// Arcade 操控模式
		int dir = master.get_analog(ANALOG_LEFT_Y);    // 从左摇杆获取前进/后退量
		int turn = master.get_analog(ANALOG_LEFT_X);   // 从左摇杆获取左转/右转量
		left_mg.move(dir + turn);                      // 设置左电机电压
		right_mg.move(dir - turn);   
		                  // 设置右电机电压
		int joystickValue = master.get_analog(ANALOG_RIGHT_Y);       // 从右摇杆获取升降控制量
		Lift_simple(joystickValue);       // 右摇杆推上→上升，拉下→下降
		
		int BtnPressed = master.get_digital(DIGITAL_L1);  // L1 按下→夹紧，松开→放松
		Claw_control(BtnPressed);  // L1 按下→夹紧，松开→放松
		pros::delay(20);                               // 等待 20ms 后进入下一帧
	}
}