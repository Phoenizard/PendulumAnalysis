## Model Framework

### Model Assumption

考虑到实际大摆锤和海盗船的差异，相比于海盗船的摆动，大摆锤在运动幅度以及规模上都远大于摆锤。同时摆锤的客舱普遍都是圆盘状，我们会在其受到的阻力和分析上进行进一步的简化。因此我们保留刚体抽象、质点简化和无相对运动三个假设，删除线形阻尼这一条件并且添加：

- 不记阻力：我们忽略空气阻力和电机的内部阻尼，尽管事实上电机的阻尼是较大的，我们认为电机的作用力足够大并且与阻尼抵消，达到我们期望的模拟现象。
- 常数施力：在海盗船模型中提出了在滚轮位置平滑施力，从0到最大值平滑给力。在大摆锤模型中，考虑到电力马力足够大以及在数学层面方便建模的两方面角度，我们认为每次电机给力都是固定常数。

### 摆锤运动模型

与海盗船模型类似，我们依然从牛顿第二定律出发，由于刚体假设，游客的运动状态角速度和摆锤的摆动角速度是耦合的，我们将目标放在构建摆动角速度的运动方程上，同时从直觉上，如果电机赋予系统动力，那么可以看作是扭动了摆臂，因此我们可以从力矩的角度得到牛顿第二定律的力矩形式：
$$
\begin{align}
\mathbf{F}_{\text{net}}&=\mathbf{a} m \\
\mathbf{L} \times\mathbf{F}_{\text{net}} &= \mathbf{L}\times \mathbf{a} m \\
\sum \mathbf{\tau}_i &= \mathbf{L} \times \frac{d\mathbf{v}}{dt} m \\
\sum \mathbf{\tau}_i &= \frac{d \omega}{dt} \cdot mL^2 := \frac{d \omega}{dt} \cdot I
\end{align}
$$
其中$I = mL^2$ 为转动惯量，在模型假设下，摆锤的合外力仅包含重力和电机的扭力。为了简单描述 ，我们将上表达式用标量形式写出：
$$
I\cdot\frac{d^2\theta}{dt^2} = I\cdot\frac{d\omega}{dt} = \tau_{m} - \|\mathbf{G}\times\mathbf{L}\| = \tau_{m} - mgL\sin\theta
$$
其中$\tau_m$ 代表电机给的常数力矩大小。通过求解ODE with initial condion: $\omega(\theta_{\text{in}}) = w_0$我们得到单次摆动周期中在给定初始角速度大小 $\omega_0$的角速度方程：
$$
\omega^2(\theta) = \omega_0^2 + \frac{2\tau_m}{I}(\theta - \theta_{\text{in}}) + \frac{2g}{L} [\cos(\theta) - \cos(\theta_{\text{in}})]
$$
由于该式为平方式，具体角速度的符号需要根据实际运动状况来定。

### 驱动策略和运行阶段

通过在摆动最低位置附近施加恒定力矩，该策略确保能量输入与摆锤运动产生共振效应，在保证有效的振幅增长时限制机械应力和过度加速度。游戏开始时系统角速度为0.

#### 启动阶段

当位移角度满足$|\theta | \leq \Delta\theta$ 时电机给予恒定力矩，其余时刻不给予连续力矩驱动，即：
$$
\tau_m = 
\begin{cases}
    \tau & |\theta(t)| \le \Delta\theta \\
    0 & \text{otherwise}
\end{cases}
$$
在一开始游客登上客舱时，为了方便计算，我们令摆锤向正方向偏移$20\degree$ 确保在第一次摆动就能经历一个完成的加速区间。当摆锤达到理想摆角$\theta_0 \in [10\degree, 180\degree]$ 时，启动阶段结束，注意如果theta_0设置为180度时，意味着摆锤可以360度旋转，此时我们还会设置摆到最高点的速度，通常该速度为0。

对于$\tau$的选择，首先需要满足加速度最大约束，注意到：
$$
\tau = I\cdot\frac{d\omega}{dt}+ mgL\sin\theta \leq Ia^{\max}_{\omega} + mgLsin\theta \quad \text{for } \forall \theta \in [-\Delta\theta, \Delta\theta]
$$
所以力矩的上限为：

$$
\tau \leq I a_{\omega}^{\max}
$$

为了符合现实，我们也不希望摆锤仅仅摆动几次就达到最大摆角，我们还添加约束，希望这个力矩使得摆锤至少摆动到速度为0点8次以上。在这两个约束条件下，我们总能找到一个最大的$\tau$用于启动阶段。

#### 稳定阶段

在该阶段摆锤会进行5次完整的周期运动，此时在模型假设下，系统只收到重力作用，无外力做功。具体运动模式取决于$\theta_0$:

- $\theta_0 < \pi$（常规摆动）:做大角度振动，在重力作用下满足方程：
  $$
  \frac{d\omega}{dt} = -\frac{g}{L} \sin \theta \Rightarrow \omega^2 = \frac{2g}{L} \left(\cos \theta - \cos \theta_0\right)
  $$
  根据周期定义，从速度最高点出发经历半周期到达对面最高点，再回来为一个周期。

- $\theta_0 = \pi$ (360度摆锤)：在重力作用下同样满足上述方程，只是做完整的圆周运动。由于摆臂为刚体，过最高点速度可以为0，定义从最高点出发经过整个圆周运动为一个完整周期。

#### 制动阶段

与海盗船保持一致。在摆动底部附近施加与瞬时速度相反的恒定力矩，逐渐降低系统的动能。一旦振幅降至小角度以下，就启动主动制动器使船体完全停止。对于力矩的选择，我们会统计启动阶段和稳定阶段的运行时间并且根据期望游玩时长计算制动阶段的可用时间，并且使用二分法得到在规定时间内最合适的（尽可能恰好用尽时间的）力矩大小。如果得到的力矩大小超过加速度安全限制，则使用理论安全最大的力矩进行制动，忽略在时间内这一约束。

### 游客惯性系运动分析

建立以乘客为中心的惯性坐标系 $S = (x, y, z)$，其中：

- **z轴（径向）**：乘客头顶方向，定义头顶向上为正方向
- **x轴（切向）**：垂直于摆臂，沿摆动方向的切线方向，定义正方向为偏右方向
- **y轴（法向）**：垂直于摆动平面，满足右手定则$\hat{\mathbf{y} }= \hat{\mathbf{z}} \times\hat{\mathbf{x}}$，定义游客左手侧为正方向

#### 固定座位下惯性系分析

对于固定座位下的运动学分析，游客的固有运动状态与摆锤耦合，两者共用相同角速度。通过对摆锤的运动，我们可以把数据传承到游客身上，并由游客到运动状态反推其加速系的情况。游客在沿摆臂方向(+z轴）体验到向心加速度，在切线方向上(x轴)体验到由角速度变化带来的线加速度。具体的，我们可以构造一个函数: $\mathbf{a}_S:R\rightarrow R^3$
$$
\mathbf{a}_S(t) = 
\begin{pmatrix} 
a_{s,x}(t)\\ a_{s,y}(t)\\ a_{s,z}(t)
\end{pmatrix} = 
\begin{pmatrix} 
\dot{\omega_p}L\\ 0\\ \omega_p^2L
\end{pmatrix}
$$

#### 客舱自转下惯性系分析

进一步我们为客舱添加自转机制，我们提出假设：客舱自转半径r远小于摆臂半径L, 且自转围绕摆臂和客舱的连接点，自转可以看作匀速圆周运动，自转角速度为$\omega_s$方向向上指向支点，即逆时针转动。易得对于匀速圆周运动，乘客惯性系包含向心加速度为$w_s^2 r$。

在自转条件下，注意到此时**客舱-乘客系统**构成在摆锤摆动速度为$v_p$的情况下的旋转系，会产生Coriolis Effect(科里奥利效应)，即在旋转参考系中，科里奥利效应会产生一个额外的加速度矢量：
$$
\mathbf{a}_{cor} = - 2\mathbf{\omega}_s \times \mathbf{v}_p\ ;\ a_{cor} = - 2 \|\omega_s\| \cdot\|\mathbf{v}_p\|
$$
根据右手定则，该加速度始终平行于客舱平面并垂直于摆锤摆动平面。同时于固定座位下一致，整个旋转系在摆动的时候还具有摆动方向上的加速度$\dot{\omega_p}L$  和向心加速度 $\omega_p^2L$, 注意这里假设了$r << L$, 在计算向心加速度的时候近似于在中心位置计算，不做偏移处理。

<img src="/Users/shay/Workplace/Pendulum/doc/selfRotationAnalysisAcc.png" alt="selfRotationAnalysisAcc" style="zoom:25%;" />



因此对于**乘客惯性系**，是客舱自转产生的向心加速度、摆锤摆动产生的线加速度和向心加速度、和Coriolis Effect三者的叠加。由于乘客会随着时间变化拥有一个相位角$\phi$：
$$
\phi(t) = \omega_s t
$$
我们对上述加速度进行矢量分解和叠加，就能得到有自转情况下的乘客惯性系解析式如下。
$$
a_c =  |v_p||\omega_s|
$$

$$
\mathbf{a}_S(t) = 
\begin{pmatrix} 
a_{s,x}(t)\\ a_{s,y}(t)\\ a_{s,z}(t)
\end{pmatrix} = 
\begin{pmatrix} 
-\omega_s^2r +\dot{\omega}_pL \cos{\phi} - a_c \sin{\phi}\\ -\dot{\omega}_pL\sin{\phi} - a_c\cos{\phi}\\ \omega_p^2L
\end{pmatrix}
$$

#### 游客真实体验分析

在本文对游客加速度的感知中，我们已经确认了是对支持力的测量。现在对游客进行受力分析，显然游客仅收到重力和支持里影响。无论是否加入自转情况，根据假设r<<L, 我们始终认为游客在摆锤位于$\theta$摆角时，重力在游客坐标系的作用力为 $\mathbf{g}(\theta) = [-mg\sin{\theta}, 0, -mg\cos{\theta}]^T$, 因此对于时刻t的支持力，我们有：
$$
\mathbf{N}_S(t) = m\cdot
\begin{pmatrix} 
\omega_s^2r +\dot{\omega}_p(t)L\cos{\phi(t)} + a_c(t) \sin{\phi(t)} + g\sin\theta(t)\\
-\dot{\omega}_p(t)L\sin{\phi(t)} + a_c(t)\cos{\phi(t)}\\ 
\omega_p(t)^2L + g\cos\theta(t)
\end{pmatrix}
$$
进而游客感受到的加速度等价于：
$$
\mathbf{a}_{S, \text{perceived}}(t) = 
\begin{pmatrix} 
\omega_s^2r +\dot{\omega}_p(t)L\cos{\phi(t)} + a_c(t) \sin{\phi(t)} + g\sin\theta(t)\\
-\dot{\omega}_p(t)L\sin{\phi(t)} + a_c(t)\cos{\phi(t)}\\ 
\omega_p(t)^2L + g\cos\theta(t)
\end{pmatrix}
$$

用于后续的兴奋程度分析。



### Passenger Inertial Frame Motion Analysis

We establish a passenger-centered inertial coordinate system $S = (x, y, z)$, where:
- **z-axis (radial direction)**: Passenger's head direction, with upward defined as positive
- **x-axis (tangential direction)**: Perpendicular to the swing arm, along the tangential direction of swing motion, with rightward defined as positive
- **y-axis (normal direction)**: Perpendicular to the swing plane, satisfying the right-hand rule $\hat{\mathbf{y}} = \hat{\mathbf{z}} \times \hat{\mathbf{x}}$, with the passenger's left side defined as positive

#### Inertial Frame Analysis for Fixed Seating

For kinematic analysis under fixed seating conditions, the passenger's inherent motion state couples with the pendulum, sharing identical angular velocity. Through analysis of the pendulum's motion, we can transfer data to the passenger and infer the acceleration frame from the passenger's motion state. The passenger experiences centripetal acceleration along the swing arm direction (+z-axis) and linear acceleration in the tangential direction (x-axis) resulting from angular velocity variations. Specifically, we construct a function: $\mathbf{a}_S: \mathbb{R} \rightarrow \mathbb{R}^3$

$$
\mathbf{a}_S(t) = 
\begin{pmatrix} 
a_{s,x}(t)\\ a_{s,y}(t)\\ a_{s,z}(t)
\end{pmatrix} = 
\begin{pmatrix} 
\dot{\omega}_p L\\ 0\\ \omega_p^2 L
\end{pmatrix}
$$

#### Inertial Frame Analysis with Cabin Self-Rotation

Furthermore, we introduce a self-rotation mechanism for the cabin with the following assumptions: the cabin self-rotation radius $r$ is much smaller than the swing arm radius $L$, and self-rotation occurs about the connection point between the swing arm and cabin. The self-rotation can be treated as uniform circular motion with angular velocity $\omega_s$ directed upward toward the pivot, corresponding to counterclockwise rotation. For uniform circular motion, the passenger's inertial frame contains centripetal acceleration $\omega_s^2 r$.

Under self-rotation conditions, the **cabin-passenger system** constitutes a rotating frame with pendulum swing velocity $v_p$, producing the Coriolis Effect. In the rotating reference frame, the Coriolis effect generates an additional acceleration vector:

$$
\mathbf{a}_{\text{cor}} = -2\boldsymbol{\omega}_s \times \mathbf{v}_p; \quad a_{\text{cor}} = -2 \|\boldsymbol{\omega}_s\| \cdot \|\mathbf{v}_p\|
$$

According to the right-hand rule, this acceleration remains parallel to the cabin plane and perpendicular to the pendulum swing plane. Consistent with fixed seating conditions, the entire rotating system during swinging also exhibits acceleration $\dot{\omega}_p L$ in the swing direction and centripetal acceleration $\omega_p^2 L$. Note that we assume $r \ll L$, and when calculating centripetal acceleration, we approximate at the center position without offset correction.

<img src="/Users/shay/Workplace/Pendulum/doc/selfRotationAnalysisAcc.png" alt="selfRotationAnalysisAcc" style="zoom:25%;" />

Therefore, for the **passenger inertial frame**, it represents the superposition of three contributions: centripetal acceleration from cabin self-rotation, linear and centripetal accelerations from pendulum swing, and the Coriolis Effect. Since the passenger undergoes a time-varying phase angle $\phi$:

$$
\phi(t) = \omega_s t
$$

We perform vector decomposition and superposition of the above accelerations to obtain the analytical expression for the passenger inertial frame under self-rotation conditions:

$$
\mathbf{a}_S(t) = 
\begin{pmatrix} 
a_{s,x}(t)\\ a_{s,y}(t)\\ a_{s,z}(t)
\end{pmatrix} = 
\begin{pmatrix} 
\omega_s^2 r + \dot{\omega}_p L \cos{\phi} + a_c \sin{\phi}\\ 
-\dot{\omega}_p L \sin{\phi} + a_c \cos{\phi}\\ 
\omega_p^2 L
\end{pmatrix}
$$

#### Analysis of Passenger's Actual Experience

In our analysis of passenger acceleration perception, we have established that measurements are based on support force. Now performing force analysis on the passenger, clearly the passenger is only subject to gravity and support force. Regardless of whether self-rotation is included, based on the assumption $r \ll L$, we consistently consider that when the pendulum is at swing angle $\theta$, the gravitational force in the passenger coordinate system is $\mathbf{g}(\theta) = [-mg\sin{\theta}, 0, -mg\cos{\theta}]^T$. Therefore, for the support force at time $t$, we have:

$$
\mathbf{N}_S(t) = m \cdot
\begin{pmatrix} 
\omega_s^2 r + \dot{\omega}_p(t) L \cos{\phi(t)} + a_c(t) \sin{\phi(t)} + g\sin\theta(t)\\
-\dot{\omega}_p(t) L \sin{\phi(t)} + a_c(t) \cos{\phi(t)}\\ 
\omega_p(t)^2 L + g\cos\theta(t)
\end{pmatrix}
$$

Consequently, the acceleration perceived by the passenger is equivalent to:

$$
\mathbf{a}_{S, \text{perceived}}(t) = 
\begin{pmatrix} 
\omega_s^2 r + \dot{\omega}_p(t) L \cos{\phi(t)} + a_c(t) \sin{\phi(t)} + g\sin\theta(t)\\
-\dot{\omega}_p(t) L \sin{\phi(t)} + a_c(t) \cos{\phi(t)}\\ 
\omega_p(t)^2 L + g\cos\theta(t)
\end{pmatrix}
$$



### Simulation Results

在模拟阶段，我们纵向2s最大加速度限制6g，根据真实游乐园考察，我们发现当摆锤位于360度最高点时，摆锤速度可以几乎看作为0，这时根据动能定理，在最低点的纵向加速度为5g，始终在安全范围之内。因此我们为摆锤在最高点设置一个小速度($v=0.01m/s$)并标记惯性驱使的动态方向。考虑最大速度限制100km/h (27.78m/s)，从最高点为0落下速度为$\sqrt{4gL} \leq 27.78$, 那么摆锤半径$L <= 19.66m$, 我们规定$L = 18m$ 符合限制条件。在加速阶段，根据公式(29)乘客最大能承受力矩为2648700N·m，这里力矩值满足目标可以在4次摆动周期内到达最高点且小于最大力矩限制; 刹车阶段力矩为满足目标可以在3次摆动周期内到达小角度且小于最大力矩限制的值; 摆锤初始摆角设置为$20\degree$.接下来我们对其进行模拟，更多参数见下表。

| 参数       | Value         | Note             |
| ---------- | ------------- | ---------------- |
| L          | 18m           | 摆锤半径         |
| $\theta_0$ | $\pi$         | 期望最终摆角     |
| m          | 5000kg        | 系统总质量       |
| T          | 120s          | 运行时间上限     |
| $\tau$     | 817684.72 N·m | 加速阶段施力力矩 |
| $\Delta t$ | $10\degree$   | 施力角度区间     |
| $\tau_{b}$ | 911223.04 N·m | 减速阶段施力力矩 |
| $\omega_s$ | 0.4rad/s      | 自转匀速角速度   |

#### 摆锤运动状态

![PendulumMotion](/Users/shay/Workplace/Pendulum/doc/PendulumMotion.png)

从图中可以观察到摆锤系统的运动特征完全符合设计预期，展现出三阶段运动模式。在加速阶段（约0-40s），摆锤振幅从初始20°经过约3个摆动周期逐步增长至最高点，角速度峰值和线速度持续增大，红色脉冲显示力矩τ仅在摆锤通过最低点附近施加力矩使速度产生突变；进入稳定阶段（约40-80s）后，驱动力矩停止施加，系统在重力作用下进行圆周运动，最大向心加速度为4g；减速阶段（80-120s）施加反向制动力矩, 振幅递减，经过约3个摆动周期平稳减速至接近0°，角速度峰值随之递减至0。

#### 非自转下Rider感受的加速度

![RiderAccelerationsNon-RotatingFrame](/Users/shay/Workplace/Pendulum/doc/RiderAccelerationsNon-RotatingFrame.png)

这是乘客收到的支持力等效的加速度。我们可以看到y方向上（红色）始终为0，因为游客固定在非自转客舱中，x和z方向和摆锤的切向和径向完全一致。在x和z方向，乘客感受到的加速度波动和摆锤系统在两个方向上的加速度波动一致，只是收到重力分量的影响产生了偏移。

#### 自转下Rider感受的加速度

<img src="/Users/shay/Workplace/Pendulum/doc/RiderAccelerationsSelf-RotatingFrame.png" alt="RiderAccelerationsSelf-RotatingFrame" style="zoom:72%;" />

上图为自转情况下Rider的三方向的加速度感受情况。可以看到除了Z方向与原先保持一致(L>>r的假设)，非自转下的x方向加速度被分解到了自转下的y和x方向，并且被Coriolis效应进一步增强。下图为Coriolis效应的加速度范数大小图，我们可以看到这符合定义式$2|\omega_S||v_p|$ ，和摆锤的摆动速度图像变换规律一致。





================== Excitement Value Statistics without SelfRotate=================
Max Excitement Value: 4.259
Min Excitement Value: 0.212
Total Excitement Value (Integral): 152.099
Average Excitement Value: 1.290
Average Total Acceleration: 31.973 m/s²
Max Total Acceleration: 81.159 m/s²
Average Scale Height: 21.648 m
Max Scale Height: 36.000 m
Average Jerk: 8.035 m/s³
Max Jerk: 24.910 m/s³

================== Excitement Value Statistics with SelfRotate=================
Max Excitement Value: 4.488
Min Excitement Value: 0.364
Total Excitement Value (Integral): 165.770
Average Excitement Value: 1.406
Average Total Acceleration: 33.229 m/s²
Max Total Acceleration: 85.712 m/s²
Average Scale Height: 21.648 m
Max Scale Height: 36.000 m
Average Jerk: 10.643 m/s³
Max Jerk: 24.910 m/s³
