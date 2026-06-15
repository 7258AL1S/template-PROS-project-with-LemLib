# AL-1S

-from TEAM 7258A

VEX V5 竞赛机器人程序，基于 [PROS](https://pros.cs.purdue.edu/) 内核构建。

## 依赖

| 库 | 版本 | 用途 |
|---|---|---|
| PROS Kernel | 4.2.2 | 系统内核 |
| LVGL | 9.2.0 | 图形界面 |
| [LemLib](https://github.com/LemLib/LemLib) | latest | 里程计、PID 控制、路径跟踪 |

## 功能

- 里程计定位（Tracking Wheel Odometry）
- PID 运动控制
- 路径跟随（path following）
- 定点移动（move to point / pose）
- 转向控制（turn to）

## 构建

通过 VS Code 中 PROS 扩展的 **Build** 按钮编译。

## 硬件要求

- VEX V5 Brain
- 惯性传感器（IMU）
- 编码器 / 追踪轮
- 底盘电机组

## 许可证

MIT License — 详见 [LICENSE](./LICENSE)
