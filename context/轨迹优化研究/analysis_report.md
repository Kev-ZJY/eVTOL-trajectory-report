# 六旋翼 eVTOL 轨迹优化 · 代码诊断报告

## 一、问题概述

原始 MATLAB (GPOPS2 + SNOPT) 脚本存在两大问题：
1. **运行极慢** — SNOPT 跑了 25,820 QP 迭代 / 14,596 主迭代仍未收敛
2. **控制输入振荡严重** — 求解后的 omega1~6 上下震荡，不符合实际

## 二、根本原因分析

### 🔴 Issue 1: 电池模型奇异性 (CRITICAL)

```matlab
SOC = 1 - Qb / Q_max;          % Q_max = 3.3
Uoc = c1 * log(SOC) + ...      % log(0) = -inf 当 Qb → 3.3
a   = -R_battery - k*Q_max/(Q_max - Qb);  % → -inf 当 Qb → 3.3
Ib  = (-b + sqrt(b^2 - 4*a*c))/(2*a);     % 判别式可能为负
```

- 目标函数 `(xf_5(1)/3.3 - 1)` 试图让 Qb → 3.3
- 但 Qb → 3.3 时电池模型奇异性导致梯度爆炸
- **结果：优化器在靠近奇点附近反复震荡，永远无法收敛**

### 🔴 Issue 2: 爬升梯度路径约束奇异性

```matlab
path2 = -dz ./ (dx + eps);    % eps 极小值
```

- VTOL/悬停阶段 dx ≈ 0，导致除法近奇异
- eps 无法解决本质问题，只是让数值从 NaN 变成极大值
- 此约束在 Phase 3-5 中持续施加，限制了求解空间

### 🟠 Issue 3: 状态变量尺度不匹配

| 状态 | 量级 | 问题 |
|------|------|------|
| Qb | O(1) | 正常 |
| p,q,r | O(0.01~0.1) | 极小 |
| u | O(10~40) | 中等 |
| x | O(1000~5000) | 极大 |
| z | O(-500~-30) | 极大 |

- 未做任何缩放 → Hessian 矩阵条件数极差
- SNOPT 的 Limited Memory Hessian (仅10次更新) 无法处理
- **结果：优化器每一步都在"猜"，震荡是必然的**

### 🟠 Issue 4: 阶段间链接约束过于严格

```matlab
bounds.eventgroup(1).lower = [zeros(1, n_x), 0];
bounds.eventgroup(1).upper = [zeros(1, n_x), 0];
```

- 5 个阶段之间，13个状态+时间全部严格相等
- 加上 tight bounds，可行域极窄
- **结果：初始解稍有偏差就导致可行性问题**

### 🟡 Issue 5: 目标函数设计问题

```matlab
output.objective = (xf_5(1)/3.3 - 1) + rotation_penalty;
```

- `(Qb_final/3.3 - 1)` 的范围是 [-1, 0]，当 Qb=0 时 = -1，Qb=3.3时 = 0
- `rotation_penalty` = ∫(dp²+dq²+dr²)dt ≥ 0
- **问题：目标值区间不匹配** — 电池项 O(1)，旋转惩罚项可能 O(10~100+)
- 惩罚项占主导，目标函数对电池消耗不敏感

### 🟡 Issue 6: 角速度状态边界过于严格

```matlab
x_min_1 = [0; -0.1; -0.1; -0.1; ...];  % p,q,r 范围 ±0.1 rad/s
```

- VTOL 起飞时角速度变化剧烈，±0.1 rad/s 太紧
- 导致优化器必须用控制输入剧烈变化来"卡"住角速度边界

### 🟡 Issue 7: SNOPT 设置不合理

从 SNOPT info 提取：
- Limited Memory Hessian：仅 10 次更新（用于 14,596 变量的问题！）
- Hessian frequency：99,999,999（几乎从不重新计算）
- Reduced Hessian dim：2,000（对于 14,584 非线性变量远远不够）
- **结果：Hessian 信息极差 → QP 子问题求解效率低下 → 收敛极慢**

## 三、SNOPT 日志关键证据

```
Column counts:
  Total columns: 14,596
  Nonlinear constraints: 11,218
  Nonlinear variables: 14,584

迭代过程:
  Itn 14,596: 目标值在 2.0 ~ 2.57 之间反复震荡
  绝大多数变量状态: "N SBS" (Nonbasic Superbasic) 或 "N FR" (Nonbasic Free)
  ➡ 优化器在所有自由度上都处于"游离"状态，没有约束边界在起作用

时间统计:
  Constraint function evaluation: ***** (>9999秒)
  Solving problem: -2.00秒 (溢出)
```

## 四、修复策略

### 4.1 电池模型修复
- 将 Qb 搜索范围限制在 [0, 3.2]（留安全余量，远离 Q_max=3.3）
- 在 SOC 接近 0 时对对数项做 clamp/clip
- 电池电流计算增加判别式保护

### 4.2 增加状态缩放
- 使用对角线缩放矩阵，使所有状态在 O(1) 量级
- 各状态的缩放因子：
  - Qb: ×1, p/q/r: ×10, u/v/w: ×0.1
  - phi/theta/psi: ×10, x: ×0.01, y: ×0.1, z: ×0.01

### 4.3 爬升梯度约束改进
- 改用 `atan2(-dz, dx)` 或速度倾角公式替代除法形式
- 或者直接约束 `-dz ≥ gradient_min * dx` 避免除法

### 4.4 目标函数重构
- 合并为统一量纲：能量消耗 + 姿态平滑度
- 归一化权重：ω₁×(Qb/3.3) + ω₂×(rotation_penalty/max_penalty)

### 4.5 阶段链接松弛
- 对接时间和位置状态宽松约束
- 角速度/姿态角允许微小跳变（物理上合理）

### 4.6 求解器迁移
- 从 SNOPT 迁移到 **IPOPT**（通过 CasADi）
- IPOPT 使用完整 Hessian 信息（或 BFGS 更新）
- 更好的处理非凸约束和非线性
