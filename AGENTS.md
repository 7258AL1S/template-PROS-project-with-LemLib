# AL-1S — VEX V5 竞赛机器人

Team 7258A · PROS Kernel 4.2.2 · C++ gnu++26 · ARM Cortex-A9

## 构建

通过 VS Code 中 **PROS 扩展的 Build 按钮**编译。不要在终端手动运行 make。若 PROS 扩展不可用、Build 按钮失败或构建环境损坏，停止并报告环境错误；不要尝试手动 `make` 或 `pros conduct fetch` 作为回退方案。

输出产物：
- `bin/cold.package.bin` — 冷固件（用户代码）
- `bin/hot.package.bin` — 热固件（可动态更新）
- `bin/monolith.bin` — 完整固件

## 入口点 & 生命周期

[`src/main.cpp`](src/main.cpp) 包含 PROS 标准生命周期函数，按执行顺序：

| 函数 | 调用时机 | 说明 |
|------|---------|------|
| `initialize()` | 开机后立即执行 | 传感器/屏幕初始化，需在数秒内完成 |
| `competition_initialize()` | 仅比赛模式，autonomous 前 | 自动选择器等 |
| `autonomous()` | 自动阶段（15秒） | **独立 task 运行**，可被禁用中断 |
| `opcontrol()` | 手动控制阶段 | **独立 task 运行**，主循环在此 |
| `disabled()` | 机器人禁用时 | 清理/状态显示 |

⚠️ `autonomous()` 和 `opcontrol()` 各自运行在独立 task 中。被禁用后重新启用会**重启 task**，而非恢复。

## 架构层次

```
main.cpp (生命周期入口)
  ├── lemlib/       运动控制库（PID、里程计、路径跟踪）
  │     ├── motions/    moveToPoint, moveToPose, turnTo, follow
  │     └── tracking/   TrackingWheelOdom（追踪轮里程计）
  ├── hardware/     硬件抽象层（Motor, MotorGroup, Encoder, IMU）
  ├── units/        编译期量纲检查（Length, Angle, Time 等）
  └── PROS Kernel   底层系统 API（motor, IMU, RTOS, screen 等）
```

## 关键约定

### 电机端口表示法
端口号前加负号表示反转方向：
```cpp
pros::MotorGroup left_mg({1, -2, 3});   // 端口 2 反转
pros::MotorGroup right_mg({-4, 5, -6}); // 端口 4、6 反转
```

### 单位系统（编译期类型安全）
```cpp
using namespace units;
Length dist = 10_in;           // 英寸
Angle angle = 90_stDeg;        // 标准度（模360）
AngularVelocity vel = 360_rpm; // 转速
Time timeout = 500_msec;       // 毫秒
```
在本项目中，只有传给 LemLib 运动/里程计/PID 函数的参数，以及所有 `units::Length / Angle / AngularVelocity / Time` 类型变量，必须使用单位类型；普通 if 条件、数组下标、枚举值和非 LemLib 辅助函数的数值可以保持为原生整数或浮点。若某个值要传给 LemLib，请先转换为对应的 `units::` 类型。

### 命名空间别名（已预定义）
```cpp
namespace ll = lemlib;
namespace mh = lemlib::motion_handler;
```

### 硬件对象统一定义在 `sensor.cpp`

所有需要直接访问 VEX 端口的全局对象（pros::Motor、pros::Imu、pros::Rotation、pros::ADIEncoder、lemlib::V5InertialSensor 等）必须只在 src/sensor.cpp 中定义；其他 .cpp 文件只能通过 #include "sensor.h" 访问这些对象，禁止在任何其他 .cpp 中声明或定义它们。

```cpp
// sensor.h — 声明
extern pros::Motor       Claw;
extern lemlib::V5InertialSensor imu;

// sensor.cpp — 定义
pros::Motor              Claw(11);
lemlib::V5InertialSensor imu(15);
```

其他文件通过 `#include "sensor.h"` 引用，**禁止**在其他 `.cpp` 中重复定义同一端口的硬件对象。理由：
- 避免**端口冲突**（同端口被两个对象初始化导致运行时错误）
- 避免**静态初始化顺序问题**（跨编译单元的全局对象构造顺序未定义）
- 统一管理所有端口，易于接线变更

若实际机器人接线或端口映射与当前代码不同，停止并索要验证过的端口映射表，不要猜测端口号或传感器类型。

### 优化级别
`-Os`（体积优化），ARM Cortex-A9 + NEON FPU + hard float ABI。

## ⚠️ 必须遵守的规则

按优先级排列：

1. **构建方式** — 通过 VS Code 中 PROS 扩展的 Build 按钮编译。不要在终端手动运行 make。若 PROS 扩展不可用、Build 按钮失败或构建环境损坏，停止并报告环境错误；不要尝试手动 `make` 或 `pros conduct fetch` 作为回退方案。

2. **禁止阻塞循环** — `autonomous()` 和 `opcontrol()` 中的函数会被主循环反复调用。在 motorcontrol 相关代码中绝对不要使用 `while`/`for` 阻塞循环。使用非阻塞状态机 + `static` 变量 + timer。

3. **硬件对象统一定义在 `sensor.cpp`** — 所有需要直接访问 VEX 端口的全局对象（pros::Motor、pros::Imu、pros::Rotation、pros::ADIEncoder、lemlib::V5InertialSensor 等）必须只在 src/sensor.cpp 中定义；其他 .cpp 文件只能通过 #include "sensor.h" 访问这些对象，禁止在任何其他 .cpp 中声明或定义它们。若实际机器人接线或端口映射与当前代码不同，停止并索要验证过的端口映射表，不要猜测端口号或传感器类型。

4. **使用单位系统** — 在本项目中，只有传给 LemLib 运动/里程计/PID 函数的参数，以及所有 `units::Length / Angle / AngularVelocity / Time` 类型变量，必须使用单位类型；普通 if 条件、数组下标、枚举值和非 LemLib 辅助函数的数值可以保持为原生整数或浮点。若某个值要传给 LemLib，请先转换为对应的 `units::` 类型。

5. **端口约束** — SmartPort 端口号不能为 0；`lemlib::SmartPort` 是 `consteval` 编译期求值，只接受 1-21 的有效端口。占位符也必须用有效值，标注 TODO 即可。电机端口负号在花括号内：`{1, -2, 3}`。

### 其他重要约束

6. **PROS 内核版本固定** — 内核 4.2.2，不要升级 `api.h` / `main.h`，它们会被内核升级覆盖。

7. **全局作用域 lambda 不能使用捕获默认** — `[&]` / `[=]` 在文件作用域不合法，改用显式捕获或（对 `static` 变量）空捕获直接访问。

8. **`static` 存储期变量不能被 lambda 捕获** — 静态变量直接访问即可，无需写在 `[]` 里。

9. **LemLib 里程计对象必须延迟初始化** — `TrackingWheel` 构造时会调用 `encoder->getAngle()`，所以编码器必须先于 TrackingWheel 构造。将 TrackingWheel/Odom 放在 `lemLibInit()` 函数内用 `static` 局部变量延迟构造，由 `initialize()` 调用，可避免跨编译单元的静态初始化顺序问题。

10. **PROS CLI 与 Python 3.14 不兼容** — `pkg_resources` 已在 Python 3.14 中移除。`pros conduct fetch` 等 CLI 命令不可用。通过 VS Code 中 PROS 扩展的 Build 按钮编译。

## 附录：设计模式

### 脉冲时序控制（爪子）
```cpp
// 按钮上升沿触发 → 全功率运行 N ms → 切换到低功率保持或刹车
void Claw_control_time(int BtnPressed) {
    static uint32_t pulseStart = 0;
    static bool lastBtn = false;
    static bool closed  = false;

    if (!lastBtn && BtnPressed) { closed = !closed; pulseStart = pros::millis(); }
    lastBtn = BtnPressed;

    if (pros::millis() - pulseStart < kPulseMs) {
        Claw.move(closed ? kFull : -kFull);
    } else {
        Claw.move(closed ? kHold : 0);  // 或 HOLD 刹车
    }
}
```
单按钮切换夹紧/松开，全功率脉冲避免持续堵转烧电机。

### 六段式 PID（升降机构）
上升/下降各 3 段，通过 `distFromBottom` 分段切换 PID 参数。上升段和下降段各自独立积分，清零对侧积分防止 windup 跨越。

### 摇杆死区 + 刹车（底盘）
```cpp
if (fabs(dir) < kDeadzone && fabs(turn) < kDeadzone) {
    motors.brake();  // 死区内刹车
} else {
    motors.move(dir + turn);  // Arcade drive
}
```

## 附录：Git 工作流

| 远程 | URL | 用途 |
|------|-----|------|
| `origin` | `7258AL1S/template-PROS-project-with-LemLib` | 发布 |
| `upstream` | `shaonianhuge/template-PROS-project-with-LemLib` | 模板同步 |

同步上游：
```bash
git fetch upstream dev
git reset --hard upstream/dev
git push -f origin main
```
## 依赖库

| 库 | 位置 | 用途 |
|---|---|---|
| LemLib | `src/lemlib/` + `include/lemlib/` | 里程计、PID、运动控制（**已内嵌，非 submodule**） |
| LVGL 9.2.0 | `include/liblvgl/` | 脑屏 GUI（当前未在 main.cpp 中使用） |
| PROS Kernel 4.2.2 | firmware/ 预编译 | 系统内核 |

## 更多信息

- [README.md](README.md) — 硬件要求、许可证
- [PROS 官方文档](https://pros.cs.purdue.edu/)
- [LemLib 文档](https://github.com/LemLib/LemLib)
