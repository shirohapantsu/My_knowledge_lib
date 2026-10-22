---
tags:
  - 机器人控制
  - 最优控制
created: 2026-10-19
up: "[[机器人控制]]"
status:
---

# MPC 控制算法

> [!abstract]  一句话总结MPC
> **“看未来 $N$ 步，算最优 $N$ 步，只走当下第 $1$ 步，下一步重新看。”** 
> 就像人类下棋或开车：眼睛看着前方几十米的路况（**预测时域**），脑子里盘算出一套未来几秒打方向盘和踩油门的动作序列（**优化求解**），但手脚只执行当前这一瞬间的动作；下一秒车往前开了一点，根据最新位置重新观察并规划（**滚动优化**）。

传统的 [[PID  控制算法]] 是“看过去”（根据过去的误差累积来调整当前输入），[[LQR 控制算法]] 是“算全局”（在无约束线性假设下离线算出一个固定反馈增益矩阵 $K$），而 **MPC 的杀手锏在于它能显式处理多变量耦合与物理约束（如方向盘最大转角、电机最大扭矩、安全避障边界）**。

---

## 1. 三大核心概念和基本控制流程

> [!theorem] 预测模型（Predictive Model）
> 用描述系统物理规律的数学方程（如[[状态空间方程]]），根据当前状态和候选的未来控制序列，预测未来 $N$ 步的系统状态。

> [!theorem] 滚动优化（Receding Horizon Optimization）
> 在每个采样时刻 $k$，结合系统约束，最小化包含状态跟踪误差和控制能量消耗的代价函数，求出未来 $N$ 步的最优控制序列 $U^* = \{u^*_{k}, u^*_{k+1}, \dots, u^*_{k+N-1}\}$，并随采样时刻推进不断重新求解。

> [!theorem] 反馈校正（Feedback Correction）
> **只将最优序列的第一个控制量 $u^*_k$ 作用于真实系统**；到 $k+1$ 时刻，测量或估计真实状态 $x_{k+1}$，以此更新预测的起点，将预测窗口向前推进一步并重新优化，从而减小模型失配和外部扰动带来的影响。

### 基本控制流程

```mermaid
flowchart LR
    A[测量或估计当前状态] --> B[更新参考轨迹与约束]
    B --> C[利用模型预测并求解优化问题]
    C --> D[只执行最优序列的第一个控制量]
    D --> E[真实系统运行一个采样周期]
    E --> A
```

---

## 2. 线性MPC

当系统动态是线性的（或围绕工作点/参考轨迹进行了雅可比线性化），且代价函数为二次型、约束为线性不等式时，该问题被称为 **线性 MPC** ，具体步骤如下

### 2.1 建立离散线性系统模型

利用[[拉格朗日力学]]建立系统的动力学方程组，解出自由变量的二阶导，建立向量 $\dot x$ 和 $x$ 、$u$ 的方程，求出矩阵 $A$ 、$B$ ；然后建立 **连续线性[[状态空间方程]]** $\dot x = Ax+Bu$ ；对连续线性状态空间方程进行离散化，得到离散线性状态空间方程 $x_{k+1} = Ax_k + Bu_k$ 

### 2.2 定义MPC优化问题

统一采用“绝对时刻 $\mid$ 当前规划时刻”的预测记号：$k$ 是当前采样时刻，$i$ 是向前预测的步数，$x_{k+i|k}$ 和 $u_{k+i|k}$ 分别是在时刻 $k$ 规划的、时刻 $k+i$ 的状态和控制量；$x_{k+i|k}^{\mathrm{ref}}$ 和 $u_{k+i|k}^{\mathrm{ref}}$ 是本次规划使用的对应参考值，$x_k$ 是当前测量或估计的状态。

这里有几个需要人为规定好的量：

- 预测时域 $N$：向前预测的步数
- 状态跟踪误差权重 $Q$
- 控制输入权重 $R$
- 终端状态权重 $P$

在此基础上，定义当前时刻的 MPC 代价函数：

$$
\begin{aligned}
J_k={}&\underbrace{\sum_{i=1}^{N-1}
(x_{k+i|k}-x_{k+i|k}^{\mathrm{ref}})^T Q
(x_{k+i|k}-x_{k+i|k}^{\mathrm{ref}})}_{\text{状态跟踪误差代价}}\\
&+\underbrace{(x_{k+N|k}-x_{k+N|k}^{\mathrm{ref}})^T P
(x_{k+N|k}-x_{k+N|k}^{\mathrm{ref}})}_{\text{终端代价}}\\
&+\underbrace{\sum_{i=0}^{N-1}
(u_{k+i|k}-u_{k+i|k}^{\mathrm{ref}})^T R
(u_{k+i|k}-u_{k+i|k}^{\mathrm{ref}})}_{\text{控制量代价}}
\end{aligned}
$$

状态跟踪误差从未来第 $1$ 步开始计算，第 $N$ 步单独使用终端权重 $P$；控制量代价仍从当前第 $0$ 步开始计算。当前状态 $x_k$ 已知，其误差代价与待优化的控制序列无关，因此不纳入目标函数。当 $N=1$ 时，状态跟踪误差的求和为空，只保留终端状态代价和当前控制量代价。

再定义输入、状态和输入变化量约束（向量不等式按分量理解）：

$$
\begin{aligned}
u_{\min}&\leq u_{k+i|k}\leq u_{\max},
&&i=0,\ldots,N-1\\
x_{\min}&\leq x_{k+i|k}\leq x_{\max},
&&i=1,\ldots,N\\
\Delta u_{\min}&\leq \Delta u_{k+i|k}\leq \Delta u_{\max},
&&i=0,\ldots,N-1
\end{aligned}
$$

其中，$\Delta u_{k+i|k}=u_{k+i|k}-u_{k+i-1|k}$。规定 $u_{k-1|k}=u_{k-1}$，即上一时刻实际施加的控制量，因此第一步的输入变化量为 $\Delta u_{k|k}=u_{k|k}-u_{k-1}$。

于是得到线性 MPC 的基本优化形式：

$$
\begin{aligned}
\min_{u_{k|k},\ldots,u_{k+N-1|k}}\quad
&J_k\\
\text{s.t.}\quad
&x_{k|k}=x_k,\quad u_{k-1|k}=u_{k-1}\\
&x_{k+i+1|k}=Ax_{k+i|k}+Bu_{k+i|k},
&&i=0,\ldots,N-1\\
&x_{\min}\leq x_{k+i|k}\leq x_{\max},
&&i=1,\ldots,N\\
&u_{\min}\leq u_{k+i|k}\leq u_{\max},
&&i=0,\ldots,N-1\\
&\Delta u_{\min}\leq u_{k+i|k}-u_{k+i-1|k}\leq\Delta u_{\max},
&&i=0,\ldots,N-1
\end{aligned}
$$

### 2.3 构造标准QP

二次规划（Quadratic Programming，QP）要求**目标函数是决策变量的二次函数，约束是线性的**。下面利用预测模型消去未来状态，将整个控制序列作为决策变量，把 2.2 的问题整理成矩阵形式。

#### （1）堆叠预测状态和控制量

设状态维数为 $n$，控制量维数为 $m$。定义：

$$
X_k=
\begin{bmatrix}
x_{k+1|k}\\
x_{k+2|k}\\
\vdots\\
x_{k+N|k}
\end{bmatrix}\in\mathbb{R}^{Nn},
\qquad
U_k=
\begin{bmatrix}
u_{k|k}\\
u_{k+1|k}\\
\vdots\\
u_{k+N-1|k}
\end{bmatrix}\in\mathbb{R}^{Nm}
$$

其中 $U_k$ 是待求的决策变量；$X_k$ 堆叠未来 $N$ 步状态，共 $N$ 个状态块。当前状态 $x_k$ 是预测的已知起点，不放入 $X_k$。按照相同顺序堆叠参考值：

$$
X_k^{\mathrm{ref}}=
\begin{bmatrix}
x_{k+1|k}^{\mathrm{ref}}\\
x_{k+2|k}^{\mathrm{ref}}\\
\vdots\\
x_{k+N|k}^{\mathrm{ref}}
\end{bmatrix},
\qquad
U_k^{\mathrm{ref}}=
\begin{bmatrix}
u_{k|k}^{\mathrm{ref}}\\
u_{k+1|k}^{\mathrm{ref}}\\
\vdots\\
u_{k+N-1|k}^{\mathrm{ref}}
\end{bmatrix}
$$

将离散模型递推展开：

$$
\begin{aligned}
x_{k+1|k}&=Ax_k+Bu_{k|k}\\
x_{k+2|k}&=A^2x_k+ABu_{k|k}+Bu_{k+1|k}\\
&\ \vdots\\
x_{k+N|k}&=A^Nx_k+\sum_{j=0}^{N-1}A^{N-1-j}Bu_{k+j|k}
\end{aligned}
$$

于是可以统一写成：

$$
\boxed{X_k=\Phi x_k+\Gamma U_k}
$$

其中：

$$
\Phi=
\begin{bmatrix}
A\\A^2\\\vdots\\A^N
\end{bmatrix},
\qquad
\Gamma=
\begin{bmatrix}
B&0&\cdots&0\\
AB&B&\cdots&0\\
\vdots&\vdots&\ddots&\vdots\\
A^{N-1}B&A^{N-2}B&\cdots&B
\end{bmatrix}
$$

$\Phi\in\mathbb{R}^{Nn\times n}$ 描述当前状态对未来状态的影响，$\Gamma\in\mathbb{R}^{Nn\times Nm}$ 描述控制序列对未来状态的影响。

#### （2）将代价函数整理成标准二次型

按照 $X_k$ 和 $U_k$ 的堆叠顺序，构造块对角权重矩阵：

$$
\bar Q=\operatorname{diag}(\underbrace{Q,\ldots,Q}_{N-1\text{ 个}},P),
\qquad
\bar R=\operatorname{diag}(\underbrace{R,\ldots,R}_{N\text{ 个}})
$$

这里的 $\operatorname{diag}$ 表示按矩阵块构成块对角矩阵。$\bar Q$ 的前 $N-1$ 块 $Q$ 对应 $x_{k+1|k},\ldots,x_{k+N-1|k}$，最后一块 $P$ 对应终端状态 $x_{k+N|k}$。因此 $\bar Q\in\mathbb{R}^{Nn\times Nn}$，$\bar R\in\mathbb{R}^{Nm\times Nm}$。当 $N=1$ 时，$\bar Q=P$。

将矩阵形式的原代价函数、$e_k$ 的定义和预测方程连立：

$$
\left\{
\begin{aligned}
J_k&=(X_k-X_k^{\mathrm{ref}})^T\bar Q(X_k-X_k^{\mathrm{ref}})
+(U_k-U_k^{\mathrm{ref}})^T\bar R(U_k-U_k^{\mathrm{ref}})\\
e_k&=\Phi x_k-X_k^{\mathrm{ref}}\\
X_k&=\Phi x_k+\Gamma U_k
\end{aligned}
\right.
$$

其中，$\Phi x_k$ 是未来控制输入全部为零时，系统从当前状态自然演化得到的预测轨迹；$e_k$ 是该轨迹与参考轨迹的误差。当前时刻的 $x_k$ 和参考轨迹已知，因此 $e_k$ 相对于决策变量 $U_k$ 是固定的。

由后两式得到完整的预测跟踪误差 $X_k-X_k^{\mathrm{ref}}=e_k+\Gamma U_k$。将其代入第一式，利用权重矩阵的对称性展开，并按 $U_k$ 的次数整理：

$$
\begin{aligned}
J_k
&=(e_k+\Gamma U_k)^T\bar Q(e_k+\Gamma U_k)
+(U_k-U_k^{\mathrm{ref}})^T\bar R(U_k-U_k^{\mathrm{ref}})\\
&=U_k^T(\Gamma^T\bar Q\Gamma+\bar R)U_k
+2(\Gamma^T\bar Qe_k-\bar R U_k^{\mathrm{ref}})^T U_k\\
&\quad+e_k^T\bar Qe_k+(U_k^{\mathrm{ref}})^T\bar R U_k^{\mathrm{ref}}\\
&=\frac12 U_k^T H U_k+g_k^T U_k+c_k
\end{aligned}
$$

与标准二次型逐项对照，得到：

$$
\begin{aligned}
H&=2(\Gamma^T\bar Q\Gamma+\bar R)\\
g_k&=2\Gamma^T\bar Qe_k-2\bar R U_k^{\mathrm{ref}}\\
c_k&=e_k^T\bar Qe_k+(U_k^{\mathrm{ref}})^T\bar R U_k^{\mathrm{ref}}
\end{aligned}
$$

$c_k$ 是展开后与 $U_k$ 无关的常数项，不影响最优控制序列，求解时可省略。这里假设 $Q,P$ 对称半正定、$R$ 对称正定，因此 $H$ 正定；若约束可行，该 QP 有唯一最优解。

#### （3）将三类约束整理成矩阵形式

将各步的上下界按对应顺序重复堆叠，记为 $X_{\min},X_{\max}$、$U_{\min},U_{\max}$ 和 $\Delta U_{\min},\Delta U_{\max}$。其中状态上下界仍对应 2.2 的未来第 $1$ 至第 $N$ 步，因此 $X_{\min},X_{\max}\in\mathbb{R}^{Nn}$。例如，$U_{\min}=\mathbf{1}_N\otimes u_{\min}$，其中 $\otimes$ 是 Kronecker 积。

**控制输入约束**可以直接写成：

$$
U_{\min}\leq U_k\leq U_{\max}
$$

**状态约束**为 $X_{\min}\leq X_k\leq X_{\max}$，代入预测方程后得到：

$$
X_{\min}-\Phi x_k\leq\Gamma U_k\leq X_{\max}-\Phi x_k
$$

**输入变化量约束**需要考虑第一步与上一时刻实际输入的差。定义块差分矩阵 $D\in\mathbb{R}^{Nm\times Nm}$：

$$
D=
\begin{bmatrix}
I_m&0&\cdots&0\\
-I_m&I_m&\cdots&0\\
0&-I_m&\ddots&\vdots\\
\vdots&\ddots&\ddots&I_m
\end{bmatrix},
\qquad
d_k=
\begin{bmatrix}
u_{k-1}\\0\\\vdots\\0
\end{bmatrix}
$$

$D$ 的主对角块为 $I_m$，紧邻主对角线下方的块为 $-I_m$，其余块为零。于是：

$$
\Delta U_k=DU_k-d_k
=\begin{bmatrix}
u_{k|k}-u_{k-1}\\
u_{k+1|k}-u_{k|k}\\
\vdots\\
u_{k+N-1|k}-u_{k+N-2|k}
\end{bmatrix}
$$

相应约束为：

$$
\Delta U_{\min}+d_k\leq DU_k\leq\Delta U_{\max}+d_k
$$

#### （4）得到可交给求解器的标准 QP

将上述约束纵向合并：

$$
C=\begin{bmatrix}I_{Nm}\\\Gamma\\D\end{bmatrix},
\qquad
\ell_k=\begin{bmatrix}
U_{\min}\\X_{\min}-\Phi x_k\\\Delta U_{\min}+d_k
\end{bmatrix},
\qquad
h_k=\begin{bmatrix}
U_{\max}\\X_{\max}-\Phi x_k\\\Delta U_{\max}+d_k
\end{bmatrix}
$$

最终的双边约束标准形式为：

$$
\boxed{
\begin{aligned}
\min_{U_k}\quad&\frac12 U_k^T H U_k+g_k^T U_k\\
\text{s.t.}\quad&\ell_k\leq CU_k\leq h_k
\end{aligned}}
$$

若求解器要求单边不等式，可等价改写为：

$$
\begin{bmatrix}C\\-C\end{bmatrix}U_k
\leq\begin{bmatrix}h_k\\-\ell_k\end{bmatrix}
$$

当模型、预测时域和权重固定时，$\Phi,\Gamma,H,C$ 可以预先计算；每个采样周期根据最新状态、参考轨迹和上一时刻输入，更新 $g_k,\ell_k,h_k$，再进入 2.4 的 QP 求解步骤。

### 2.4 调用QP求解器求解

将 $H,g_k,C,\ell_k,h_k$ 传入 QP 求解器，求得满足约束的最优控制序列：

$$
U_k^*=\begin{bmatrix}
u_{k|k}^*\\
u_{k+1|k}^*\\
\vdots\\
u_{k+N-1|k}^*
\end{bmatrix}
$$

### 2.5 输出最优控制量

**只执行最优序列的第一个控制量**：

$$
u_k=u_{k|k}^*
$$

将其施加到真实系统并保持一个采样周期 $T_s$。到下一时刻，重新测量或估计状态 $x_{k+1}$，更新参考轨迹与约束，再次求解 QP，形成“预测—优化—执行—反馈”的滚动闭环。

---

## 3. 非线性NMPC

**非线性模型预测控制（Nonlinear Model Predictive Control，NMPC）** 的计算过程可以按以下思路理解：

$$
\boxed{\text{构建 OCP}}
\xrightarrow{\text{DMS 离散转写}}
\boxed{\text{有限维 NLP}}
\xrightarrow{\text{SQP / RTI}}
\boxed{\text{控制序列}}
$$

OCP（Optimal Control Problem，最优控制问题）描述要优化的运动与控制目标；DMS（Direct Multiple Shooting，直接多重打靶法）将其转写为有限维 NLP（Nonlinear Programming，非线性规划）；SQP 或 RTI 再对这个 NLP 进行数值求解。

### 3.1 构建连续时间OCP

当前采样时刻为 $t_k$，采样周期为 $T_s$，预测时长为 $T=NT_s$。首先利用[[拉格朗日力学]]等方法建立非线性[[状态空间方程]] $\dot x=f_c(x,u)$，并定义运行代价和终端代价：

$$
\begin{aligned}
\ell(x,u,t)
&=(x-x^{\mathrm{ref}}(t))^TQ_c(x-x^{\mathrm{ref}}(t))\\
&\quad+(u-u^{\mathrm{ref}}(t))^TR_c(u-u^{\mathrm{ref}}(t))\\
V_f(x,t)
&=(x-x^{\mathrm{ref}}(t))^TP(x-x^{\mathrm{ref}}(t))
\end{aligned}
$$

其中 $Q_c,R_c$ 是连续时间运行代价的权重，$P$ 是终端权重。于是得到每个采样时刻需要求解的 OCP：

$$
\begin{aligned}
\min_{x(\cdot),u(\cdot)}\quad
&J_k^c=\int_{t_k}^{t_k+T}\ell(x(t),u(t),t)\,\mathrm dt
+V_f(x(t_k+T),t_k+T)\\
\text{s.t.}\quad
&x(t_k)=x_k\\
&\dot x(t)=f_c(x(t),u(t))\\
&x_{\min}\leq x(t)\leq x_{\max}\\
&u_{\min}\leq u(t)\leq u_{\max}
\end{aligned}
$$

约束作用于预测区间，$x_k$ 是当前测量或估计的状态。**此时待求的是整段状态函数和控制函数**，需要经过离散化，才能交给有限维优化求解器。

### 3.2 用DMS将OCP转化为NLP

> [!abstract] 直接多重打靶（DMS）
> DMS（Direct Multiple Shooting）是一种将连续时间最优控制问题（OCP）转化为有限维非线性规划问题（NLP）的直接法，本质上是对OCP问题的离散化

#### （1）划分时域，设置节点变量

令 $t_{k+i}=t_k+iT_s$，将预测区间分为 $N$ 段。在第 $i$ 段采用常值控制：

$$
u(t)=u_{k+i|k},\qquad
t\in[t_{k+i},t_{k+i+1}),\quad i=0,\ldots,N-1
$$

在各节点设置独立的状态变量 $x_{k+i|k}$，与各段控制量共同优化。DMS 通过每段积分及段间连续性等式连接这些节点。[CasADi 官方文档](https://web.casadi.org/docs/#direct-multiple-shooting)

沿用未来 $N$ 步状态和 $N$ 步控制量的堆叠：

$$
X_k=\begin{bmatrix}
x_{k+1|k}\\\vdots\\x_{k+N|k}
\end{bmatrix}\in\mathbb R^{Nn},
\qquad
U_k=\begin{bmatrix}
u_{k|k}\\\vdots\\u_{k+N-1|k}
\end{bmatrix}\in\mathbb R^{Nm}
$$

为便于 RTI 在收到新测量后更新初值约束，单独保留初始节点 $x_{k|k}$，定义完整决策向量：

$$
z_k=\begin{bmatrix}x_{k|k}\\X_k\\U_k\end{bmatrix}
\in\mathbb R^{(N+1)n+Nm}
$$

$x_{k|k}$ 是优化变量，$x_k$ 是已知的测量或估计值，二者通过 $x_{k|k}=x_k$ 连接。

#### （2）分段积分，建立连续性等式

每段从自己的节点状态 $x_{k+i|k}$ 出发，在常值输入 $u_{k+i|k}$ 下积分 $T_s$，将所得末端状态记为 $f_d(x_{k+i|k},u_{k+i|k})$。例如可用 Runge–Kutta 方法计算该离散映射，并要求：

$$
x_{k+i+1|k}-f_d(x_{k+i|k},u_{k+i|k})=0,
\qquad i=0,\ldots,N-1
$$

求解过程中的节点猜测可以暂时不连续，收敛时通过这些等式将各段轨迹连接起来。每段等式只涉及相邻状态和本段输入，因此约束雅可比矩阵具有稀疏结构。

#### （3）离散化代价函数

对运行代价采用左矩形求积，令 $Q=T_sQ_c$、$R=T_sR_c$，将采样周期吸收到离散权重中。参考值取本次规划在对应节点的参考轨迹。由于初值等式固定了 $x_{k|k}=x_k$，可省略不影响最优解的当前状态误差项，得到与 2.2 一致的定义：

$$
\begin{aligned}
J_k={}&\sum_{i=1}^{N-1}
(x_{k+i|k}-x_{k+i|k}^{\mathrm{ref}})^TQ
(x_{k+i|k}-x_{k+i|k}^{\mathrm{ref}})\\
&+(x_{k+N|k}-x_{k+N|k}^{\mathrm{ref}})^TP
(x_{k+N|k}-x_{k+N|k}^{\mathrm{ref}})\\
&+\sum_{i=0}^{N-1}
(u_{k+i|k}-u_{k+i|k}^{\mathrm{ref}})^TR
(u_{k+i|k}-u_{k+i|k}^{\mathrm{ref}})
\end{aligned}
$$

沿用 2.3 中的 $\bar Q=\operatorname{diag}(Q,\ldots,Q,P)$（含 $N-1$ 个 $Q$）和 $\bar R$，写成矩阵形式：

$$
J_k(z_k)=(X_k-X_k^{\mathrm{ref}})^T\bar Q(X_k-X_k^{\mathrm{ref}})
+(U_k-U_k^{\mathrm{ref}})^T\bar R(U_k-U_k^{\mathrm{ref}})
$$

#### （4）组装有限维NLP

将初值约束和各段动力学约束堆叠为：

$$
r_k(z_k)=\begin{bmatrix}
x_{k|k}-x_k\\
x_{k+1|k}-f_d(x_{k|k},u_{k|k})\\
x_{k+2|k}-f_d(x_{k+1|k},u_{k+1|k})\\
\vdots\\
x_{k+N|k}-f_d(x_{k+N-1|k},u_{k+N-1|k})
\end{bmatrix}=0
$$

**此时目标函数对 $z_k$ 仍是二次型，但动力学等式约束是非线性的，因此整体仍是 NLP。**

在未来第 $1$ 至第 $N$ 个节点施加状态上下界，在各段施加输入上下界，并补入相邻采样输入的变化量约束。各上下界、差分矩阵 $D$、已知向量 $d_k$ 沿用 2.3，其中第一步变化量为 $u_{k|k}-u_{k-1}$。节点上的状态约束是对连续路径约束的离散近似。按照 $z_k$ 的堆叠顺序构造：

$$
C_z=\begin{bmatrix}
0&0&I_{Nm}\\
0&I_{Nn}&0\\
0&0&D
\end{bmatrix},
\qquad
\ell_k=\begin{bmatrix}
U_{\min}\\X_{\min}\\\Delta U_{\min}+d_k
\end{bmatrix},
\qquad
h_k=\begin{bmatrix}
U_{\max}\\X_{\max}\\\Delta U_{\max}+d_k
\end{bmatrix}
$$

$C_z$ 的三组块列分别对应 $x_{k|k},X_k,U_k$。于是得到：

$$
\boxed{
\begin{aligned}
\min_{z_k}\quad&J_k(z_k)\\
\text{s.t.}\quad&r_k(z_k)=0\\
&\ell_k\leq C_z z_k\leq h_k
\end{aligned}}
$$

到这里，待求对象已从连续函数变成有限维向量 $z_k$，微分方程已变成代数等式约束。**DMS 完成的是 OCP 到 NLP 的转写，接下来才进行数值优化。**

### 3.3 使用SQP或RTI求解NLP

#### （1）SQP：序列二次规划

**SQP（Sequential Quadratic Programming）在同一采样时刻反复构造和求解局部 QP，逐步修正状态与控制轨迹。** 用 $j$ 表示当前采样时刻内的优化迭代次数，$z_k^{(j)}$ 表示当前猜测。

在该点线性化等式约束，并构造目标函数的局部二次模型，求解增量 $\Delta z$：

$$
\begin{aligned}
\min_{\Delta z}\quad
&\frac12\Delta z^T\mathcal H^{(j)}\Delta z
+\nabla J_k(z_k^{(j)})^T\Delta z\\
\text{s.t.}\quad
&r_k(z_k^{(j)})+\mathcal A^{(j)}\Delta z=0\\
&\ell_k-C_z z_k^{(j)}\leq C_z\Delta z
\leq h_k-C_z z_k^{(j)}
\end{aligned}
$$

其中 $\mathcal A^{(j)}=\left.\dfrac{\partial r_k}{\partial z_k}\right|_{z_k^{(j)}}$，$\mathcal H^{(j)}$ 是拉格朗日函数 Hessian 的近似。对本节的二次跟踪代价，可采用 Gauss–Newton 近似：

$$
\mathcal H^{(j)}=2\operatorname{diag}(0_{n\times n},\bar Q,\bar R)
$$

该近似保留目标函数的曲率，忽略非线性动力学约束对拉格朗日函数 Hessian 的贡献。求得 QP 解 $\Delta z^{(j)}$ 后，更新：

$$
z_k^{(j+1)}=z_k^{(j)}+\alpha_j\Delta z^{(j)},
\qquad 0<\alpha_j\leq1
$$

步长可通过线搜索确定。重复“线性化—解 QP—更新轨迹”，直到最优性和约束残差达到容差，或达到迭代与时间限制；通常只能寻找局部解。[SQP 与 NMPC 教学资料](https://www.syscop.de/files/2015ss/numopt/e9_SQP.pdf)

##### 回溯步长（Backtracking Line Search）

**QP 给出修改轨迹的方向，回溯决定沿该方向走多远。** 由于 QP 使用局部近似，直接取完整步长 $\alpha_j=1$，可能使原问题的非线性动力学残差增大。因此先试完整步长，若效果不够好，就逐次缩小步长。

对约束优化，接受一步时需要综合考虑目标代价与约束违反程度。一种常见选择是 $\ell_1$ 评价函数（merit function）：

$$
\begin{aligned}
\theta_k(z)
&=\|r_k(z)\|_1
+\|[\ell_k-C_z z]_+\|_1
+\|[C_z z-h_k]_+\|_1\\
\phi_\rho(z)&=J_k(z)+\rho\,\theta_k(z),\qquad \rho>0
\end{aligned}
$$

其中 $[v]_+$ 表示对向量逐分量取 $\max(v_i,0)$，$\theta_k$ 汇总等式残差和上下界违反量，$\rho$ 是罚权重。在一次回溯中固定 $\rho$ 和 QP 方向 $\Delta z^{(j)}$。记 $d_j=\phi_\rho'(z_k^{(j)};\Delta z^{(j)})$ 为评价函数沿该方向的单侧方向导数；进行下面的下降线搜索前，应使 $d_j<0$。

从 $\alpha=1$ 开始，计算试探点 $z_{\mathrm{trial}}=z_k^{(j)}+\alpha\Delta z^{(j)}$，用**原非线性函数**重新计算代价与约束残差，并检验 **Armijo 充分下降条件**：

$$
\boxed{
\phi_\rho\!\left(z_k^{(j)}+\alpha\Delta z^{(j)}\right)
\leq\phi_\rho\!\left(z_k^{(j)}\right)+c_1\alpha d_j
},\qquad 0<c_1<1
$$

由于 $d_j<0$，右侧要求评价函数获得与步长相称的下降。若满足，接受 $\alpha_j=\alpha$ 并更新轨迹；否则令 $\alpha\leftarrow\beta\alpha$（$0<\beta<1$），继续试探。例如取 $\beta=0.5$，候选步长依次为 $1,\frac12,\frac14,\ldots$。**同一次回溯只调整步长，不重新求解 QP。** acados 提供基于评价函数的回溯及 Armijo 条件选项。[acados 官方文档](https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp_options.AcadosOcpOptions.globalization)

若尚未收敛却没有下降方向，应检查罚权重、Hessian 正则化和 QP 求解状态；若步长小于设定下限，或回溯次数、计算时间达到上限，应报告线搜索失败，不能将其视为收敛。回溯改善迭代的稳健性，但中间轨迹仍可能存在非线性约束残差。

#### （2）RTI：实时迭代

**RTI（Real-Time Iteration）基于 SQP，每个采样周期通常只进行一次 SQP 更新，即构造并求解一个 QP，不等待当前 NLP 完全收敛。** 下一周期用更新后的轨迹继续迭代。[RTI 研究论文](https://arxiv.org/abs/2403.07101)

为减少测量到控制输出之间的延迟，计算可分为两个阶段：

1. **准备阶段**：移位上一周期的状态和控制轨迹，补齐末端节点；在该猜测处进行各段积分、计算导数并构造 QP，必要时进行凝聚以缩小 QP 规模。
2. **反馈阶段**：收到最新状态 $x_k$ 后，更新初值等式的右端，求解准备好的 QP，更新一次轨迹并立即取出首个控制量。[acados 官方文档](https://docs.acados.org/features/#algorithmic-features-and-solver-options)

由于将初始节点保留为变量，最新测量在本节模型中仅通过 $x_{k|k}-x_k=0$ 进入 NLP；对应的增量约束为：

$$
\Delta x_{k|k}=x_k-x_{k|k}^{(0)}
$$

基本 RTI 通常采用完整步长，得到本周期的近似解：

$$
\widehat z_k=z_k^{(0)}+\Delta z_k
$$

这里“一次迭代”指一次 SQP 更新，QP 求解器内部仍可进行多次迭代。RTI 依赖较好的初始猜测，单次更新不保证原非线性约束已精确满足，因此需要检查原问题的约束残差与求解状态。

| 对比项 | SQP | RTI |
| --- | --- | --- |
| 每周期的 SQP 更新 | 多次，按容差或计算预算停止 | 基本方案一次 |
| 计算安排 | 在当前问题上持续迭代 | 准备与反馈分阶段进行 |
| 本周期结果 | 通常更接近当前 NLP 的局部解 | 用一次更新得到近似控制轨迹 |

### 3.4 执行控制并滚动更新

将本周期求解或更新后的控制序列记为 $\widehat U_k$，**只执行其第一个控制量**：

$$
u_k=\widehat u_{k|k}
$$

将其施加到真实系统并保持一个采样周期 $T_s$。随后移位状态与控制轨迹，作为下一周期的初始猜测；收到新状态 $x_{k+1}$ 后，按 SQP 或 RTI 流程继续更新，形成滚动闭环。
