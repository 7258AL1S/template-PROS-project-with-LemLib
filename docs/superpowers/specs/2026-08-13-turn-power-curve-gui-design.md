# turnTo 电机功率曲线 GUI 工具 — 设计说明

日期：2026-08-13

## 目标

在现有 `tools/turn_power_curve.py` 基础上增加 GUI 模式，用弹出窗口中的数值框
实时调整并渲染 `turnTo` 的 `motorPower` 曲线。

## 交互范围

GUI 提供以下可编辑数值框：

| 字段 | 默认值 |
|---|---|
| 目标角度 | -90 deg |
| FullTime | 900 ms |
| 最大功率 maxSpeed | 1 |
| kP | 1.32 |
| kI | 0 |
| kD | 0.1 |

初始角度固定为 `0 deg`。其余内部参数固定：

- `dt=10 ms`
- `max_omega=360 deg/s`
- `min_speed=0`
- `slew=0`
- `exit_range=2 deg`
- `exit_time=250 ms`
- `early_exit_range=0`

## 运行方式

```powershell
E:\conda\envs\pytorch\python.exe tools\turn_power_curve.py
```

无参数时默认打开 GUI。旧 CLI 模式通过 `--cli` 保留。

## 交互与渲染

- 使用 Tkinter `StringVar.trace` 监听输入变化。
- 约 150 ms 防抖后自动重新计算并更新图表。
- 输入非法时保留上一张有效曲线，并在状态栏提示错误。
- 图表使用 `FigureCanvasTkAgg` 嵌入 Tkinter 窗口。

## 代码结构

- 修改 `tools/turn_power_curve.py`。
- 保留 `simulate_turn_power` 等核心函数和现有测试不变。
- 新增 `run_gui()`、`build_gui_fields()`、`render_curve()` 等 GUI 函数。
- `main()` 无参数时调用 `run_gui()`，`--cli` 时走原命令行逻辑。

## 验证

- 运行 `tools/test_turn_power_curve.py`，14 个测试仍全部通过。
- 启动 GUI，修改任一数值框后曲线应自动刷新。
- 非法输入不崩溃，状态栏给出明确提示。
