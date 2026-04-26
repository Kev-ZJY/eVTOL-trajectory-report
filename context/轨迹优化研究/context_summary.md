# eVTOL 轨迹优化项目 · 上下文总结

## 项目概况

- **项目**: 六旋翼 eVTOL 轨迹优化（毕业设计）
- **路径**: `/Users/kevin_zjy/Documents/毕业设计/六旋翼eVTOL模型/正常情况/轨迹优化/0418/`
- **用户**: Kevin (kevin_zjy) — 个人投资者 + 开发者 + PM 求职中
- **时间**: 2026-04-26 全面分析 + 调试

---

## 一、核心问题链

### 1.1 初始状态错误（P0）
`ocp_solver.py` 的初始状态 `x0_phys[12] = -31.5`（高度31.5m），MATLAB 原版用 `eps*ones(13,1)` 从地面开始。这跳过了地面→悬停的转速斜坡，是电机振荡的**根因**。

**修复**: 改为 z=0，通过递进式求解收敛。

### 1.2 Phase 结构不符合 EASA 规定
EASA MOC-2 SC-VTOL 规定起飞流程应包含：
- 垂直起飞段（z=0→h1=3m）— 需 FATO 约束
- 转换段（z=3m→h2=30.5m）— 需 12.5% 梯度约束
- 爬升段 — 需 V>=VTOSS=10/VFTO=16

当前5阶段定义中缺少 h1=3m 航点约束，Phase 1 无路径约束。

### 1.3 y 方向漂移（已修复）
无惩罚项时优化器利用 y 方向体轴耦合降低电池消耗，y 属于 [-4.98, +2.95]m。

**修复**: 目标函数加 y2 软惩罚项（`y_penalty_weight=0.01`），|y|max 降至 0.11m（43.6x 改善）。

### 1.4 电机 w 振荡（已约束）
原始轨迹 w 最高 1216 rad/s2（Phase 3），远超物理极限。

**修复**: 递进式 w 约束（500 to 200 rad/s2），最终全阶段 w <= 83。

---

## 二、求解器方法

### 技术栈
- **框架**: CasADi + IPOPT
- **离散**: 直接配点法（Direct Collocation）
- **网格**: (12, 18, 16, 14, 16) — 小网格稳定

### 收敛策略
```
Step 0: 人工初猜 (init_guess_manual.py) -> 无 w 约束
    | 热启动
Step 2a: w <= 500 rad/s2 (57 iter, 10s)
    | 热启动
Step 2b: w <= 200 + y2 penalty (1000 iter, 195s)
```

### 关键原则
- **小步验证-数据闭环**: 每步独立验证再交付
- **递进式约束**: 从宽松到严格，热启动
- **软惩罚 > 硬边界**: y2 软惩罚（而非收紧 y 边界）防止发散
- **中间解持久化**: 每步保存 .mat 文件用于热启动和追溯

---

## 三、物理基准

| 指标 | 值 | 评级 |
|------|-----|------|
| w <= 50 rad/s2 | - | 平滑飞行 |
| w <= 150 rad/s2 | - | 正常机动 |
| w <= 300 rad/s2 | - | 紧急机动 |
| w > 300 rad/s2 | - | 不可行 |
| 边界跳变 > 10 rad/s | - | 不可接受 |

### 最终轨迹质量
| 参数 | 值 | 状态 |
|------|-----|------|
| 初始高度 | z=0.0m | OK |
| 终端位置 | x=500m, z=305m | OK |
| SOC | 97.2% | OK |
| 总时间 | 41.0s | OK |
| w <=200 | P1=83, P2=27, P3=16, P4=5, P5=21 | OK |
| 边界跳变 | 1-11 rad/s | OK |
| Vmax | 19.6m/s | OK |
| Ib_max | 3471A | OK |
| y 摆幅 | 0.16m | OK |

---

## 四、用户工作风格与偏好

### 沟通偏好
- **语言**: 中文沟通，简洁直接
- **交付**: 重交付 > 重解释
- **反馈**: 直接指出问题，要求数据闭环
- **接受格式**: 表格输出、对比数据

### 工作伦理
- **小步验证**: 不要直接跑 15-20 分钟的脚本而不先确认中间结果
- **数据闭环**: 每次修改后必须跑 solver 确认再声称"已修复"
- **量化比较**: Before/After 必须有数字对比
- **独立验证**: 创建独立 test 脚本验证每步收敛

### 技术偏好
- **Python + CasADi/IPOPT**: 轨迹优化求解器
- **Matplotlib**: 时域绘图
- **Scipy.io (.mat)**: 中间解持久化
- **递进式约束**: 从宽松约束到严格约束的阶梯策略

---

## 五、EASA 合规清单（待办）

| 优先级 | 任务 | 状态 |
|--------|------|------|
| P0 | Python 初始状态 z=0 | 已完成 |
| P0 | 增加 h1=3m 航点约束 | 待办 |
| P0 | Phase 1a 增加 FATO 约束 | 待办 |
| P1 | 修正 draw.m x_vert=15.2m | 待办 |
| P1 | 统一速度阈值 VTOSS=10/VFTO=16 | 待办 |
| P2 | 增加 Y 方向空域约束 | 待办 |
| P2 | 区分悬停/爬升梯度公式 | 待办 |

---

## 六、文件结构

### 求解器
- `ocp_solver.py` - 主求解器（CasADi/IPOPT）
- `dynamics_py.py` - 六旋翼动力学模型
- `init_guess_manual.py` - 人工初始猜测
- `solve_step0.py` - Step 0: 无 w 约束求解
- `solve_step2.py` - Step 2: 递进式约束求解
- `test_step2a.py` - 独立 Step 2a 验证
- `test_step2b.py` - 独立 Step 2b 验证（含 y2 penalty）

### 分析与合规
- `easa_regulation_analysis.md` - EASA 合规性分析报告
- `analyze_oscillation.py` - 电机振荡分析
- `analyze_physical_benchmark.py` - 物理可行性基准分析

### 绘图
- `plot_solution.py` - 8 张时域图生成
- `plot_y_comparison.py` - y 方向 Before/After 对比
- `fig1_motor_speeds.png` ~ `fig8_wdot_summary.png` - 生成图
- `fig_y_comparison.png` - y 方向对比图

### 数据
- `solution_casadi.mat` - 最终标准解
- `step2a_test.mat`, `step2b_test.mat` - 中间验证解
- `step0_solution.mat`, `step1_solution.mat`, `step2a_solution.mat` - 递进解

---

## 七、优化模型数学表述

### 7.1 状态向量与控制向量

**状态向量** (13 维)：

$$x = [Q_b,\ p,\ q,\ r,\ u,\ v,\ w,\ \phi,\ \theta,\ \psi,\ x,\ y,\ z]^\text{T}$$

- $Q_b$: 电池电荷量 (Ah)
- $p, q, r$: 体轴角速率 (rad/s)
- $u, v, w$: 体轴线速度 (m/s)
- $\phi, \theta, \psi$: 欧拉角 (滚转、俯仰、偏航, rad)
- $x, y, z$: 惯性系位置 (m, NED坐标系, z=0地面, z<0高度)

**控制向量** (6 维)：

$$u = [\omega_1,\ \omega_2,\ \omega_3,\ \omega_4,\ \omega_5,\ \omega_6]^\text{T}$$

六旋翼转速 (rad/s)，约束范围 $[10, 350]$ rad/s。

### 7.2 动力学方程 (ODEs)

**刚体运动学-动力学** (13 个一阶 ODE)：

$$
\begin{aligned}
\dot{Q}_b &= I_b / 3600 \\
\begin{bmatrix}\dot{p} \\ \dot{q} \\ \dot{r}\end{bmatrix} &= J^{-1} \left(\begin{bmatrix}L \\ M \\ N\end{bmatrix} - \begin{bmatrix}p \\ q \\ r\end{bmatrix} \times J \begin{bmatrix}p \\ q \\ r\end{bmatrix}\right) \\
\begin{bmatrix}\dot{u} \\ \dot{v} \\ \dot{w}\end{bmatrix} &= -\begin{bmatrix}p \\ q \\ r\end{bmatrix} \times \begin{bmatrix}u \\ v \\ w\end{bmatrix} + g \begin{bmatrix}-\sin\theta \\ \sin\phi\cos\theta \\ \cos\phi\cos\theta\end{bmatrix} + \frac{1}{M} \begin{bmatrix}F_x \\ F_y \\ F_z\end{bmatrix} \\
\begin{bmatrix}\dot{\phi} \\ \dot{\theta} \\ \dot{\psi}\end{bmatrix} &= \begin{bmatrix}1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta\end{bmatrix} \begin{bmatrix}p \\ q \\ r\end{bmatrix} \\
\begin{bmatrix}\dot{x} \\ \dot{y} \\ \dot{z}\end{bmatrix} &= R_b^i(\phi,\theta,\psi)^\text{T} \begin{bmatrix}u \\ v \\ w\end{bmatrix}
\end{aligned}
$$

**旋转矩阵** (体轴→惯性系)：

$$
R_b^i = \begin{bmatrix}
c\theta c\psi & s\phi s\theta c\psi - c\phi s\psi & c\phi s\theta c\psi + s\phi s\psi \\
c\theta s\psi & s\phi s\theta s\psi + c\phi c\psi & c\phi s\theta s\psi - s\phi c\psi \\
-s\theta & s\phi c\theta & c\phi c\theta
\end{bmatrix}
$$

### 7.3 六旋翼力/力矩模型

**旋翼拉力系数与前进比**：

$$J_i = \frac{2\pi V}{\omega_i D_m}, \quad V = \sqrt{u^2 + v^2 + w^2}$$

$$c_{t,i}(J_i) = -0.1039 J_i^2 - 0.0763 J_i + 0.0848$$
$$c_{m,i}(J_i) = -0.0154 J_i^2 + 0.0042 J_i + 0.0042$$

**六旋翼合推力与力矩**：

$$T = \sum_{i=1}^6 T_i, \quad T_i = \frac{\rho D_m^4}{4\pi^2} c_{t,i} \omega_i^2$$
$$L = -\sum T_i L_{y,i}, \quad M = \sum T_i L_{x,i}$$
$$N = \frac{\rho D_m^5}{4\pi^2} \sum c_{m,i} \sigma_i \omega_i^2, \quad \sigma = [-1,1,-1,1,-1,1]$$

**气动阻力** (简化模型)：

$$F_x = -\frac{1}{2}\rho S_{yoz} c_d |u|u, \quad 
F_y = -\frac{1}{2}\rho S_{xoz} c_d |v|v, \quad 
F_z = \frac{1}{2}\rho S_{xoy} c_d |w|w - T$$

### 7.4 电池模型

**SOC-开路电压**：

$$SOC = 1 - Q_b / Q_{max}, \quad Q_{max} = 3.3 \text{Ah}$$

$$U_{oc} = c_1 \ln(SOC) + e^{-c_2 SOC} + c_3 SOC^3 + c_4$$

**电机功率 → 电池电流** (二次方程)：

$$P_b = \sum_{i=1}^6 I_{m,i} U_{m,i}, \quad 
I_m = M_m / k_t, \quad 
U_m = \frac{30}{\pi k_v}\omega + R_m I_m$$

$$-R_{batt}I_b^2 + (U_{oc} - k\frac{Q_{max}Q_b}{Q_{max}-Q_b} + Ae^{-BQ_b})I_b - \frac{P_b}{N_s N_p} = 0$$

### 7.5 最优控制问题 (OCP) 表述

**目标函数**：

$$\min\ J = \frac{Q_{b,f}}{Q_{max}} + 0.3\frac{\sum T_p}{T_{max}} + 0.005\frac{1}{N_{total}}\sum_{k} (\dot{p}_k^2 + \dot{q}_k^2 + \dot{r}_k^2) + 0.1\frac{\sum (\Delta\omega)^2}{N\cdot 6\cdot 150^2} + w_y \frac{\sum y_k^2}{N_{total}}$$

其中 $w_y$ 为 y² 软惩罚权重（最终解 $w_y = 0.01$）。

**约束条件**：

| 约束类型 | 数学表达 | 说明 |
|---------|---------|------|
| 动力学 | $\dot{x} = f(x, u)$ | RK4 离散时间配点 |
| 初始状态 | $x(0) = \mathbf{0}$ | 地面悬停 |
| 终端条件 | $x_f \geq 500\text{m},\ z_f = -305\text{m}$ | 水平+高度目标 |
| 状态边界 | $x_{lb} \leq x \leq x_{ub}$ | 每阶段独立边界 |
| 控制边界 | $10 \leq \omega_i \leq 350$ rad/s | 安全转速范围 |
| 电机加速度 | $|\dot{\omega}_i| \leq \dot{\omega}_{max}$ | 递进: 500→200 rad/s² |
| 阶段跳变 | $|\omega_{p+1,0} - \omega_{p,end}| \leq 20$ rad/s | 阶段间连续 |
| 空域 | $28.6 + 0.125x + z \leq 0$ | Phase 1-2 空域边界 |
| 爬升梯度 | Phase 2: $\geq 4.5\%$, Phase 4: $\geq 2.5\%$ | 安全爬升 |
| 最小速度 | Phase 2-3: $V \geq 10$, Phase 4: $V \geq 16$ m/s | VTOSS/VFTO |

### 7.6 求解方法

**离散化**: 直接多重打靶法 (Direct Multiple Shooting)

- 5 阶段分割，每阶段均匀配点 $(N_1,...,N_5) = (12, 18, 16, 14, 16)$
- RK4 积分作为连续约束
- 控制输入在每段内零阶保持 (ZOH)

**缩放策略**: 状态和控制统一缩放到 $\mathcal{O}(1)$ 尺度，避免 IPOPT 条件数问题。

**递进式热启动**:

```
Step 0: 人工初猜 -> 无 ω̇ 约束（建立大致轨迹形状）
    ↓ 热启动
Step 2a: ω̇ ≤ 500 rad/s² (57 iter, 10s)
    ↓ 热启动
Step 2b: ω̇ ≤ 200 + y² 软惩罚 (1000 iter, 195s)
```

**求解器**: CasADi (符号自动微分) + IPOPT (内点法)，有限内存拟牛顿近似。

---

*文档生成日期: 2026-04-26*
*用于上下文记忆和研究汇报网页构建*
