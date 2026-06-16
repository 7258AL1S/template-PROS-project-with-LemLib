# AL-1S — VEX V5 竞赛机器人

Team 7258A · PROS Kernel 4.2.2 · C++ gnu++26 · ARM Cortex-A9

## 构建

通过 VS Code 中 **PROS 扩展的 Build 按钮**编译。不要在终端手动运行 make。

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
所有 LemLib API 使用此单位系统，不要混用裸 `double`。

### 命名空间别名（已预定义）
```cpp
namespace ll = lemlib;
namespace mh = lemlib::motion_handler;
```

### 优化级别
`-Os`（体积优化），ARM Cortex-A9 + NEON FPU + hard float ABI。

## ⚠️ 关键陷阱

1. **禁止阻塞循环** — `autonomous()` 和 `opcontrol()` 中的函数会被主循环反复调用。在 motorcontrol 相关代码中绝对不要使用 `while`/`for` 阻塞循环。使用非阻塞状态机 + `static` 变量 + timer。
2. **不要混用裸数值与单位** — LemLib API 全部使用 units 系统。传入裸 `double` 会导致编译错误。
3. **电机端口语法** — 负号在花括号内：`{1, -2, 3}`，不是 `{-1, 2, -3}`（虽然效果等价，但惯例是正号端口在左、负号在右时用负号标记反转）。
4. **PROS 内核版本固定** — 内核 4.2.2，不要升级 `api.h` / `main.h`，它们会被内核升级覆盖。

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
