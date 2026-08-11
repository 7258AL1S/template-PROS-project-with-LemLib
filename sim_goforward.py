"""
GoForWard PD 仿真 — 纯 PD（kI=0）+ 静摩擦模型 + 最小功率保底
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.family'] = 'Microsoft YaHei'
matplotlib.rcParams['axes.unicode_minus'] = False

# ============================================================
# 全局参数
# ============================================================
Power       = 1.0       # 最大功率 [0, 1.0]
Target      = 24.0      # 目标距离（英寸）
FullTime    = 3000      # 超时时间（毫秒）
kRampTimeMs = 200       # 软启动斜坡时长（毫秒）
kRampStart  = 0.6       # 斜坡起始功率比例
kMinPower   = 0.15      # 最小功率保底（与 C++ 一致，需 > 静摩擦阈值）

max_velocity   = 40.0   # 满功率最大速度 (inch/s)
mass_inertia   = 0.15   # 惯性系数
dt             = 0.010  # 仿真步长 10ms

# 摩擦参数
viscous_friction  = 1.8    # 粘性摩擦（与速度成正比）
static_friction   = 0.12   # 静摩擦阈值（功率低于此值=推不动）
stiction_velocity = 0.3    # 低于此速度(inch/s)视为"静止"，静摩擦生效

# ============================================================
# PD 参数组（kI 全部为 0）
# ============================================================
PD_GROUPS = [
    ("kP=0.20 kD=0.10",  0.20, 0.0, 0.10),
]

# ============================================================
# 仿真核心（含静摩擦 + 最小功率保底）
# ============================================================
def simulate(kP, kI, kD, friction_coef, enable_anti_stiction=True):
    steps     = int(FullTime / (dt * 1000))
    curDist   = 0.0
    velocity  = 0.0
    err       = 0.0
    lastErr   = 0.0
    accErr    = 0.0
    absTarget = abs(Target)

    time_arr = np.zeros(steps)
    dist_arr = np.zeros(steps)
    out_arr  = np.zeros(steps)
    err_arr  = np.zeros(steps)
    vel_arr  = np.zeros(steps)
    raw_out_arr = np.zeros(steps)  # 保底前的原始 PID 输出

    for i in range(steps):
        t_ms = i * dt * 1000

        err = Target - curDist

        if abs(err) < absTarget * 0.1:
            accErr += err

        # PD 输出
        out = kP * err + kI * accErr + kD * (err - lastErr)

        # 软启动斜坡
        if t_ms < kRampTimeMs:
            out *= (t_ms / kRampTimeMs) * kRampStart

        # 功率限幅
        if abs(out) > Power:
            out = Power if Target > 0 else -Power

        # 记录原始输出
        raw_out = out

        # 静摩擦补偿：最小功率保底
        if enable_anti_stiction:
            if abs(out) > 0.001 and abs(out) < kMinPower and abs(err) > 0.3:
                out = kMinPower if out > 0 else -kMinPower

        # 底盘动力学（含静摩擦）
        target_vel = out * max_velocity

        # 静摩擦：速度极低 且 驱动力不足以克服静摩擦 → 不动
        if abs(velocity) < stiction_velocity and abs(out) < static_friction:
            acceleration = 0.0
            if abs(out) < 0.001:
                velocity = 0.0  # 彻底静止
        else:
            acceleration = (target_vel - velocity) / mass_inertia - friction_coef * velocity

        velocity += acceleration * dt
        curDist += velocity * dt

        time_arr[i] = t_ms
        dist_arr[i] = curDist
        out_arr[i]  = out
        err_arr[i]  = err
        vel_arr[i]  = velocity
        raw_out_arr[i] = raw_out

        if abs(err) < 0.3:
            time_arr  = time_arr[:i+1]
            dist_arr  = dist_arr[:i+1]
            out_arr   = out_arr[:i+1]
            err_arr   = err_arr[:i+1]
            vel_arr   = vel_arr[:i+1]
            raw_out_arr = raw_out_arr[:i+1]
            break

        lastErr = err

    return time_arr, dist_arr, out_arr, err_arr, vel_arr, raw_out_arr


# ============================================================
# 对比：有/无 静摩擦补偿
# ============================================================
kP_test, kI_test, kD_test = PD_GROUPS[0][1], PD_GROUPS[0][2], PD_GROUPS[0][3]

print("=" * 80)
print("静摩擦补偿对比（kP=0.55, kD=0.20, kI=0）")
print("=" * 80)

t_on,  d_on,  o_on,  e_on,  v_on,  r_on  = simulate(kP_test, kI_test, kD_test, viscous_friction, True)
t_off, d_off, o_off, e_off, v_off, r_off = simulate(kP_test, kI_test, kD_test, viscous_friction, False)

for label, t, d, e, v in [("有保底", t_on, d_on, e_on, v_on),
                            ("无保底", t_off, d_off, e_off, v_off)]:
    avg_spd = d[-1] / (t[-1] / 1000.0) if t[-1] > 0 else 0
    print(f"  {label}:  到达={d[-1]:.2f}in  误差={e[-1]:.3f}in  "
          f"耗时={t[-1]:.0f}ms  均速={avg_spd:.1f}in/s")


# ============================================================
# 不同摩擦场景（有保底）
# ============================================================
print(f"\n{'='*80}")
print("不同摩擦场景下的表现（有保底）")
print(f"{'='*80}")

for label, vf_mult in [("光滑地砖", 0.6), ("标称", 1.0), ("泡沫垫", 1.5), ("地毯", 2.2)]:
    fc = viscous_friction * vf_mult
    t, d, o, e, v, r = simulate(kP_test, kI_test, kD_test, fc, True)
    avg_spd = d[-1] / (t[-1] / 1000.0) if t[-1] > 0 else 0
    reached = "✓ 完成" if t[-1] < FullTime - 10 else "✗ 超时"
    print(f"  {label} (x{vf_mult:.1f}):  到达={d[-1]:.2f}in  偏差={d[-1]-Target:+.2f}in  "
          f"耗时={t[-1]:.0f}ms  均速={avg_spd:.1f}in/s  {reached}")


# ============================================================
# 绘图
# ============================================================
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# --- 图 A: 有/无保底 — 距离对比 ---
ax1 = axes[0, 0]
ax1.plot(t_on,  d_on,  '#4CAF50', linewidth=2, label=f'有保底 [{d_on[-1]:.2f}in]')
ax1.plot(t_off, d_off, '#FF5722', linewidth=2, label=f'无保底 [{d_off[-1]:.2f}in]')
ax1.axhline(y=Target, color='black', linestyle='--', linewidth=0.8)
ax1.fill_between([0, FullTime], Target-0.3, Target+0.3, alpha=0.06, color='green')
ax1.set_ylabel('距离 (inch)')
ax1.set_title('静摩擦补偿 — 距离对比')
ax1.legend(loc='lower right')
ax1.grid(True, alpha=0.3)

# --- 图 B: 有/无保底 — 功率对比 ---
ax2 = axes[0, 1]
ax2.plot(t_on,  o_on,  '#4CAF50', linewidth=1.5, label='有保底（实际输出）')
ax2.plot(t_on,  r_on,  '#4CAF50', linewidth=0.8, linestyle=':', alpha=0.5, label='有保底（原始PD）')
ax2.plot(t_off, o_off, '#FF5722', linewidth=1.5, label='无保底')
ax2.axhline(y=kMinPower, color='blue', linestyle=':', linewidth=0.8,
            label=f'kMinPower={kMinPower}')
ax2.axhline(y=static_friction, color='red', linestyle='--', linewidth=0.8,
            label=f'静摩擦阈值={static_friction}')
ax2.set_ylabel('电机输出')
ax2.set_title('静摩擦补偿 — 功率对比')
ax2.legend(fontsize=7, loc='upper right')
ax2.grid(True, alpha=0.3)

# --- 图 C: 有/无保底 — 速度对比 ---
ax3 = axes[1, 0]
ax3.plot(t_on,  v_on,  '#4CAF50', linewidth=1.5, label='有保底')
ax3.plot(t_off, v_off, '#FF5722', linewidth=1.5, label='无保底')
ax3.axhline(y=stiction_velocity, color='gray', linestyle=':', linewidth=0.8,
            label=f'静摩擦速度阈值={stiction_velocity} in/s')
ax3.set_ylabel('速度 (inch/s)')
ax3.set_xlabel('时间 (ms)')
ax3.set_title('静摩擦补偿 — 速度对比')
ax3.legend(loc='upper right')
ax3.grid(True, alpha=0.3)

# --- 图 D: 末端放大 — 最后 0.5 英寸的功率细节 ---
ax4 = axes[1, 1]
# 找到距离 > Target-1.0 的区域
mask_on  = d_on  > Target - 1.0
mask_off = d_off > Target - 1.0
if any(mask_on):
    ax4.plot(t_on[mask_on],  o_on[mask_on],  '#4CAF50', linewidth=2, label='有保底')
if any(mask_off):
    ax4.plot(t_off[mask_off], o_off[mask_off], '#FF5722', linewidth=2, label='无保底')
ax4.axhline(y=kMinPower,      color='blue', linestyle=':', linewidth=0.8)
ax4.axhline(y=static_friction, color='red', linestyle='--', linewidth=0.8)
ax4.axhline(y=0, color='gray', linewidth=0.5)
ax4.set_ylabel('电机输出')
ax4.set_xlabel('时间 (ms)')
ax4.set_title('末端放大 — 最后 1 inch 的功率细节')
ax4.legend(loc='upper right')
ax4.grid(True, alpha=0.3)

plt.tight_layout()
out_path = r'c:\Users\zzx100\Desktop\7258OverRide\AL-1S\sim_goforward.png'
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"\n图片已保存: {out_path}")
