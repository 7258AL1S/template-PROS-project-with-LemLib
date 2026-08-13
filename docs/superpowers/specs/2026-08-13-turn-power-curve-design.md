# turnTo 电机功率曲线工具 — 设计说明

日期：2026-08-13

## 目标

制作一个小工具，输入 LemLib `turnTo` 使用的 PID 三个数值，输出指定
`timeout`（即项目里的 FullTime）时间内的电机输出功率曲线。

输出值为 `turnTo` 循环内最终写到底盘电机的 `motorPower`，范围为 `-1..1`，
不包含真实电机惯性或角速度物理模型。

## 默认测试场景

对应 `src/auto/auto1.cpp` 第 106–107 行：

```cpp
turnSettings.angularPID = lemlib::PID(1.32, 0.0, 0.1);
lemlib::turnTo(-90_stDeg, 900_msec, turnParams, turnSettings);
```

| 参数 | 值 |
|---|---|
| kP / kI / kD | 1.32 / 0.0 / 0.1 |
| 初始航向 | 0 deg |
| 目标航向 | -90 deg |
| FullTime / timeout | 900 ms |
| maxSpeed | 1 |
| minSpeed | 0 |
| slew | 0 |
| 退出条件 | 2 deg / 250 ms |
| PID windupRange | 0 |
| PID signFlipReset | false |

## 文件与运行方式

- 文件：`tools/turn_power_curve.py`
- 解释器：`E:\conda\envs\pytorch\python.exe`
- 依赖：Python 3.12、matplotlib 3.11

命令行示例：

```text
E:\conda\envs\pytorch\python.exe tools\turn_power_curve.py 1.32 0.0 0.1
```

默认参数即上面的测试场景。所有场景参数均可通过命令行覆盖。

## 输入参数

位置参数：

- `kp`、`ki`、`kd`：LemLib angular PID 增益，按弧度输入。

可选参数：

- `--initial`：初始航向，单位度，默认 `0`
- `--target`：目标航向，单位度，默认 `-90`
- `--fulltime`：总时间，单位毫秒，默认 `900`
- `--dt`：仿真步长，单位毫秒，默认 `10`
- `--max-speed`：最大功率绝对值，默认 `1`
- `--min-speed`：最小功率绝对值，默认 `0`
- `--slew`：功率每秒最大变化率，默认 `0`
- `--max-omega`：内部推进航向的最大角速度，单位 deg/s，默认 `360`
- `--exit-range`：退出误差阈值，单位度，默认 `2`
- `--exit-time`：误差在阈值内持续多久退出，单位毫秒，默认 `250`
- `--windup-range`：PID 积分清零误差范围，默认 `0`
- `--sign-flip-reset`：误差变号时是否清零积分，默认 `false`
- `--save`：保存 PNG 的路径，可选

## 仿真逻辑

脚本按固定步长 `dt` 模拟 `turnTo` 的循环，逐点复刻以下逻辑：

1. 使用 AUTO 模式 `angleError` 计算最短角误差，范围为 `[-180, 180]` 度。
2. 将角误差转换为弧度，调用 LemLib 同款 PID 公式：

   ```text
   output = error * kP + integral * kI + derivative * kD
   derivative = (error - previousError) / dt
   integral += error * dt
   ```

   若 `signFlipReset` 为真且误差变号，或 `abs(error) > windupRange`，则积分清零。

3. 使用 `settling` 标志检测误差符号翻转；非 settling 阶段按 `slew` 限制输出变化率。
4. 使用 `constrainPower` 先处理最小功率，再限制在 `[-maxSpeed, maxSpeed]`。
5. 用 `heading += motorPower * maxOmega * dt` 推进内部航向，产生下一帧误差。
6. 若退出条件满足，则将后续时间点的功率置为 `0`，模拟 `turnTo` 末尾的 `brake()`。
7. 横轴固定覆盖 `0..fulltime`，即使提前退出也保持到 FullTime 结束。

## 输出

- 使用 matplotlib 绘制 `motorPower` 对时间的曲线。
- 曲线标题和轴标签使用中文或英文均可，至少标明 PID 增益和峰值功率。
- 默认弹出窗口显示；指定 `--save` 时同时保存 PNG。

## 错误处理

- `fulltime`、`dt`、`max-speed` 等数值非法时打印明确错误并退出。
- `dt <= 0`、`fulltime <= 0`、`max-speed < 0`、`min-speed < 0` 视为非法。
- PID 增益不限制范围，允许用户测试发散参数。

## 验证

实现完成后，用默认测试场景运行：

```text
E:\conda\envs\pytorch\python.exe tools\turn_power_curve.py 1.32 0.0 0.1
```

检查：

- 初始功率约为 `kP * error_rad` 经过 max/min 约束后的值。
- 曲线先随误差减小而下降，退出条件触发后功率为 `0`。
- 输出范围始终在 `[-maxSpeed, maxSpeed]`。
