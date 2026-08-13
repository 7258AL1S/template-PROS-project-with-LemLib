# turnTo 电机功率曲线 GUI 工具 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `tools/turn_power_curve.py` 默认打开 Tkinter 数值框窗口，并实时渲染 motorPower 曲线。

**Architecture:** 复用现有 `simulate_turn_power` 核心，在文件末尾增加 Tkinter GUI。`main()` 无参数时启动 GUI，`--cli` 保留原命令行模式。

**Tech Stack:** Python 3.12，Tkinter，matplotlib `FigureCanvasTkAgg`。

**解释器:** `E:\conda\envs\pytorch\python.exe`

---

## Task 1: 修改 `tools/turn_power_curve.py`

**Files:**
- Modify: `tools/turn_power_curve.py`

- [ ] **Step 1: 增加 GUI 导入和 `run_gui` 函数**

在文件顶部导入 `tkinter` 和 `FigureCanvasTkAgg`。保留原 CLI 函数。

- [ ] **Step 2: 调整 `main()` 分发逻辑**

```python
def main(argv: list[str] | None = None) -> int:
    if argv is None:
        argv = []
    if "--cli" in argv:
        parser = build_parser()
        args = parser.parse_args([a for a in argv if a != "--cli"])
        validate_args(args, parser)
        return run_cli(args)
    if argv and any(a.startswith("-") or not a.startswith("-") for a in argv):
        # 没有参数时进入 GUI；有参数时仍尝试 CLI，便于老用户继续使用。
        parser = build_parser()
        args = parser.parse_args(argv)
        validate_args(args, parser)
        return run_cli(args)
    return run_gui()
```

> 注意：无参数 `argv == []` 时进入 GUI；显式传 PID 或 `--target` 等仍走 CLI。

- [ ] **Step 3: 实现 GUI 数值框与实时渲染**

GUI 字段：目标角度、FullTime、maxSpeed、kP、kI、kD。初始角度固定 0。使用 `StringVar.trace` 和 `after(150, render)` 防抖；非法输入显示状态文本，保留旧图。

- [ ] **Step 4: 提交**

```bash
git add tools/turn_power_curve.py
git commit -m "feat: turnTo 功率曲线工具增加 GUI 实时渲染"
```

---

## Task 2: 验证

- [ ] **Step 1: 运行单元测试**

Run: `E:\conda\envs\pytorch\python.exe tools\test_turn_power_curve.py`
Expected: 14 tests pass。

- [ ] **Step 2: 冒烟测试 GUI 启动**

Run: `E:\conda\envs\pytorch\python.exe -c "import importlib.util; ... run_gui?` 不阻塞的方式验证函数可导入，或直接运行 GUI 手动检查。

- [ ] **Step 3: 提交验证产物（如有）并推送**

```bash
git push origin main
```
