# AL-1S — VEX V5 竞赛机器人

Team 7258A · PROS Kernel 4.2.2 · C++ gnu++26 · ARM Cortex-A9

## 自主执行、澄清与任务完成

- 用户提出实现、修复或审阅请求，即授权完成该任务所需的常规检查、分析和可逆修改；仅审阅请求默认不修改文件。已有授权与已提供的信息无需重复确认。
- 信息不足时，先检查代码、文档和已有上下文。只有缺失信息会实质影响正确性、硬件接入或操作授权时才询问，且只询问所需的最少信息；继续完成不依赖答案的部分。
- 常规实现细节和下文标明的默认偏好，由 Codex 根据任务自行判断并在必要时说明原因，无需申请例外；不得据此绕过明确的构建、接线或 Git 保障。
- 除明确要求整个任务暂停的规则外，“停止”仅作用于被阻塞的步骤。完成所有已授权且不依赖该步骤的工作，并具体说明阻塞原因和剩余事项。
- 交付时分别说明修改、验证、提交和推送结果。未执行或未成功的步骤必须明确列出，不得把部分完成表述为全部完成。纯文档修改检查内容和差异即可，无需构建机器人固件。

## 构建

构建只能通过 VS Code 中 **PROS 扩展的 Build 按钮**进行。本次修改导致的源代码编译错误，应自行修复并通过同一入口重试；不要把普通源代码报错直接视为构建环境损坏。

若扩展不可用、Build 无法启动或确认构建环境损坏，停止构建尝试并报告具体原因；可继续只读诊断及不依赖构建的工作，明确标记编译验证未完成。不要在终端手动运行 `make`，也不要尝试 `pros conduct fetch` 作为回退方案。

输出产物：
- `bin/cold.package.bin` — 冷固件（用户代码）
- `bin/hot.package.bin` — 热固件（可动态更新）
- `bin/monolith.bin` — 完整固件

### 工具链兼容性记录

2026-09-08 构建故障及修复过程：

1. 构建最初使用 `/usr/local/Cellar/arm-gcc-bin@10/...` 的 ARM GCC 10.3 链接器，但预编译库并非同一版本：`hardware.a`、`lemlog.a` 使用 GCC 14，`libpros.a`、`liblvgl.a` 使用 GCC 13。由此产生 `std::__throw_bad_array_new_length()` 和 `std::__cxx11::basic_string<...>::_M_replace_cold()` 未定义符号。
2. `.vscode/c_cpp_properties.json` 中的 `compilerPath` 只影响 IntelliSense，不控制 PROS 实际构建。PROS 扩展原有工具链路径已不存在，修改该路径本身不能解决链接错误。
3. GCC 10.3 还不支持 `--no-warn-rwx-segments`，该参数从 `common.mk` 的 `LNK_FLAGS` 中移除。
4. `common.mk` 的 `ARCHTUPLE` 已固定为 `/usr/local/opt/arm-gcc-bin@14/bin/arm-none-eabi-`，并将 `.vscode/c_cpp_properties.json` 的 `compilerPath` 同步到 GCC 14。
5. 修改后通过 VS Code 中 PROS 扩展的 **Build** 按钮成功构建。以后遇到相同的未定义 C++ 运行时符号，先检查实际链接器版本，不要只修改 IntelliSense 配置；构建仍必须使用 PROS **Build** 按钮。

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
传给 LemLib 运动/里程计/PID 函数的有量纲参数，按函数签名转换为对应的 `units::` 类型；所有 `units::Length / Angle / AngularVelocity / Time` 类型变量也必须使用相应单位类型。布尔值、计数、枚举及其他无量纲参数遵循函数签名，不强行转换为单位类型。普通 if 条件、数组下标和非 LemLib 辅助函数的数值可以保持原生整数或浮点。

### 命名空间别名（已预定义）
```cpp
namespace ll = lemlib;
namespace mh = lemlib::motion_handler;
```

### 硬件对象统一定义在 `sensor.cpp`

所有需要直接访问 VEX 端口的全局对象（pros::Motor、pros::Imu、pros::Rotation、pros::ADIEncoder、lemlib::V5InertialSensor 等）必须只在 `src/sensor.cpp` 中定义，在 `include/sensor.h` 中统一声明。其他 `.cpp` 文件通过 `#include "sensor.h"` 引入声明并访问这些对象，禁止手写重复声明或定义。

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

当本次修改必须依赖缺失或相互矛盾的接线信息时，仅暂停依赖该信息的部分，并询问涉及设备所需的最少信息；不要猜测端口号或传感器类型。用户本次已提供的明确、验证过的映射无需重复确认，也无需为单个设备的变更索要完整映射表。代码审查可基于现有映射继续，但不得声称已验证实物接线；无法验证实物本身不等于已发现接线冲突。

### 已知硬件事实（记忆）

- **`horizontalEncoder` 其实是竖直定位轮**：LemLib 定位硬件中唯一实际存在的物理追踪轮，测量前进/后退（`GoForWard` 也只用它做闭环）。端口见 `src/sensor.cpp`。
- **`verticalEncoder` 没有对应物理设备**：仅为代码中保留的对象，无实际追踪轮。
- **所有端口 21 的设备对象均为废弃对象**：`Claw_Rot`、`Claw_return`、`ClawRotation`、`IntakeFront`、`IntakeBack` 已废弃，不要使用、不要依赖其行为。

### 工作流约定（记忆）

- 修改前检查工作区、当前分支及远端同步状态，区分本任务改动和用户已有改动。仅提交本任务相关且已获授权的改动；保留无关改动，不擅自纳入提交、覆盖或回滚。
- **涉及代码重构或新增函数时，保留远端备份前置要求**：先将相关、已获授权的现有改动提交并普通推送，确认相关基线已同步后再开始代码修改。没有待提交改动且相关基线已同步时，无需空提交或重复推送。只读审阅和纯文档修改不受此前置要求约束。
- 前置推送因网络、认证等原因失败时，暂停依赖该备份的代码修改，继续只读分析及其他不依赖该步骤的工作，报告具体原因；不得擅自通过重置或强推绕过。
- 完成本任务修改及适用检查后，提交并普通推送；重构或新增函数也必须执行此收尾步骤。检查失败时先修复本次修改导致的问题；验证受环境阻塞时，如实记录未验证项，不得将提交或推送成功等同于验证通过。
- 提交信息由 Codex 自行撰写（中文，说明改动目的与要点）。上述范围内的提交和普通推送已有授权，无需逐次确认；推送失败时保留本地提交并报告远端同步未完成。

### 函数控制写法偏好（记忆）

以下为现有单实例、每帧调用的控制函数的默认模式。任务需要显式复位、共享状态或不同保持行为时，可自行选择合适实现并说明原因，无需申请偏好例外；仍须遵守下文的阻塞边界和硬件约束。

- **入参传原始状态，内部状态默认放在函数内 `static` 变量里**：函数接收按键布尔/摇杆原始值（`BtnPressed`、`joystickValue`），由主循环每帧调用，非阻塞；需要阶段复位时应明确处理状态重置。
- **上升沿做切换**：统一 `if (prevBtn == 0 && btn == 1)` / `if (!lastBtn && BtnPressed)` 风格，`prev` 存上一帧。
- **时间控制用 `pros::millis()` 差值 + 命名常量**：`constexpr uint32_t kXxxMs` 写在函数顶部，到点切换功率档位。
- **刹车模式分层**：运动中 BRAKE、到位 HOLD 锁死、松开 COAST；堵转后用 `move_voltage` 恒定电压顶住限位，避开 `move()` 速度指令触发 V5 内部堵转报警。
- **堵转检测统一为"编码器增量 + 滞回"**：每 250ms 比较编码器位置，位移 <3° 判定堵转，>8° 才清除（3~8° 保持原状态防抖）。
- **PID 以 PD 为主**（`kI` 基本为 0）：大误差清积分防 windup；输出钳位 ±100；到位死区 + HOLD；0/359 环绕做误差修正与 D 项保护；保留最低输出克服静摩擦。
- **分段增益调度**：升降六段式为代表——按 `distFromBottom` 分上/下各 3 段换 PID 参数，上升/下降积分互相清零。
- **互斥的多按键默认用 else-if 优先级链**，最后 `else` 按机构需求归零、刹车或维持已定义的保持状态。
- **阻塞边界**：统一遵守下文“控制函数与阻塞边界”规则。
- **注释密度高、意图先行**：中文注释解释"为什么"；常量 `kXxx` 命名；参数名统一（`BtnPressed`、`joystickValue`、`Power`、`Target`、`FullTime`）。

### 优化级别
`-Os`（体积优化），ARM Cortex-A9 + NEON FPU + hard float ABI。

## ⚠️ 必须遵守的规则

以下为强制约束，优先于默认写法偏好；引用的章节是对应规则的唯一规范版本。

1. **构建方式** — 遵守上文“构建”章节，包括构建入口、源代码错误修复和环境故障处理。

2. **控制函数与阻塞边界** — 会被 `opcontrol()` 每帧调用的函数，以及自动/手动共享的控制更新函数，必须使用非阻塞状态机和计时器。仅 `autonomous()` 调用的流程函数允许有超时、可响应禁用且适当让出执行时间的等待，并注明“仅 autonomous 使用，不得在 opcontrol 循环内调用”。有限遍历数据的 `for`/`while` 循环不因语法本身被禁止；不得在控制更新函数内循环等待按键、时间或机构到位。

3. **硬件对象与接线** — 遵守上文“硬件对象统一定义在 `sensor.cpp`”及“已知硬件事实”章节，包括统一声明/定义和接线信息缺失时的处理范围。

4. **使用单位系统** — 遵守上文“单位系统（编译期类型安全）”章节，按函数签名区分有量纲和无量纲参数。

5. **端口约束** — SmartPort 端口号不能为 0；`lemlib::SmartPort` 是 `consteval` 编译期求值，只接受 1-21 的有效端口。禁止为了通过编译而给硬件对象分配任意有效端口，`TODO` 不能替代验证过的映射。已有废弃对象保持原映射且不得新增依赖；新增设备必须使用验证过的映射。映射缺失时，继续完成不依赖端口的部分，并明确列出尚未完成的硬件接入。电机组中反转端口的写法为 `{1, -2, 3}`。

### 其他重要约束

6. **PROS 内核版本固定** — 内核 4.2.2，不要升级 `api.h` / `main.h`，它们会被内核升级覆盖。

7. **全局作用域 lambda 不能使用捕获默认** — `[&]` / `[=]` 在文件作用域不合法，改用显式捕获或（对 `static` 变量）空捕获直接访问。

8. **`static` 存储期变量不能被 lambda 捕获** — 静态变量直接访问即可，无需写在 `[]` 里。

9. **LemLib 里程计对象必须延迟初始化** — `TrackingWheel` 构造时会调用 `encoder->getAngle()`，所以编码器必须先于 TrackingWheel 构造。将 TrackingWheel/Odom 放在 `lemLibInit()` 函数内用 `static` 局部变量延迟构造，由 `initialize()` 调用，可避免跨编译单元的静态初始化顺序问题。

10. **PROS CLI 环境兼容性记录** — 当前环境记录中，PROS CLI 在 Python 3.14 下因 `pkg_resources` 缺失而失败。该记录用于环境诊断，不应据此推断所有 CLI 子命令均不可用；构建方式和禁止的回退操作仍以上文“构建”章节为准。

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

普通上游同步先 `fetch` 并比较差异，采用保留项目改动的合并方式。先检查本地远程配置；若 `upstream` 未配置，核对本节模板仓库地址后可按任务需要添加，无需仅因缺少该别名而暂停。

下列命令仅为**用 `upstream/dev` 覆盖当前分支及远端历史**的示例，不是默认同步流程。仅在用户明确授权该覆盖操作时执行，先核对目标分支并备份可能丢失的工作；已获得针对该操作的明确授权时，无需重复确认。普通同步不得默认执行重置或强推。

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

## 新增自动程序模板（记忆）

新建 `src/auto/auto2.cpp` 时，文件本身只需要 `#include "auto.h"`，因为 `auto.h` 已经包含 `auto_common.h` 和所需控制头文件。

```cpp
#include "auto.h"

void auto2() {
    startAutoBackgroundTasks();

    // 这里写 auto2 流程，可访问 liftCmd / liftGo / ClawUP / clawGo / xMacroGo / autoActive / autoBusy

    autoActive = false;
}
```

接入步骤：

1. 在 `include/auto.h` 增加 `void auto2();`。
2. 在 `src/main.cpp` 的 `autonomous()` switch 中增加 `case 2: auto2(); break;`。

注意：`startAutoBackgroundTasks()` 当前沿用 `static pros::Task`，后台任务只在程序生命周期内首次调用时真正创建。比赛时一次只跑一个 auto 没问题；若要在同一次运行中连续切换 `auto1`/`auto2`，需进一步改成可重启任务管理。
