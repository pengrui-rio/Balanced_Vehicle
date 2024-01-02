# doubleWheel_Vehicle

### System overview & demo

**Systematic scheme:**

![image](https://github.com/pengrui-rio/Balanced_Vehicle/blob/main/1-Mechanics/0-frame.png)

**Demo:**

![image](https://github.com/pengrui-rio/Balanced_Vehicle/blob/main/1-Mechanics/balance%20demo.gif)


# 双轮平衡小车

### 1-嵌入式开发

**1.1 小车整体框架**

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled.png)

**1.2 MCU选型及外设设计**

- 芯片：stm32f1c8t6
- TIM3 - TIM4：小车电机编码器读取
- TIM2： 定时1ms，计算小车电机转度

### 2-IMU滤波融合算法

**2.1 确定IMU坐标系及旋转方式**

这里的IMU坐标系确定和小车的坐标系保持一致，为东北天坐标系ENU（X轴向东，Y轴向北，Z轴竖直向上），同时定义每个轴的旋转为：

- 绕Z轴旋转：yaw，转动y角度
- 绕Y轴旋转：pitch, 转动p角度
- 绕X轴旋转：roll，转动r角度

在初始时刻，IMU坐标系和大地坐标系重合，然后依次**绕自身Z-Y-X轴**进行旋转，此时旋转方式为内旋，其三次连续绕轴旋转如下图所示：

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%201.png)

除此之外有很多种欧拉角的旋转方式，但是请记住并熟练应用上述旋转定义和旋转方式，即内旋Z-Y-X旋转，分别对应yaw，pitch，roll角。

参考：[https://blog.51cto.com/u_15262460/2883172](https://blog.51cto.com/u_15262460/2883172)

**2.2 基于绕轴旋转角的旋转矩阵**

基于上述旋转定义，根据单次绕轴旋转的知识，可知分别绕X轴，Y轴，Z轴的旋转矩阵为：

$$
\mathbf{R}_x = \left[
\begin{matrix}
  1 & 0 & 0 \\ 0 & \cos r & \sin r \\ 0 & -\sin r & \cos r
\end{matrix}
\right] \\                                                        \mathbf{R}_y = \left[
\begin{matrix}
\cos p & 0 & -\sin p \\           0 & 1 & 0 \\                                              \sin p & 0 & \cos p
\end{matrix}
\right] \\                                                 \mathbf{R}_z = \left[
\begin{matrix}
\cos y & \sin y & 0 \\                                                        -\sin y & \cos y & 0 \\                                                   0 & 0 & 1
\end{matrix}
\right] 
$$

另外，绕每个轴的**逆时针**方向为该轴旋转的正方向。

参考：[https://blog.51cto.com/u_15262460/2883172](https://blog.51cto.com/u_15262460/2883172)

**2.3 加速度计解算欧拉角**

加速度计测量的是其感受到的加速度，水平静止时，其感受到的加速度与重力加速度正好相反，即读到的数据是竖直向上的，从而我们可以根据倾斜时在三轴上的加速度分量计算姿态：

- 当加速度计水平放置时，即Z轴竖直向上，Z轴读到 1g 的数值 （g为重力加速度标量），X轴和Y轴两个方向的数值为0，可以记作（0，0，g）。
- 当加速度计旋转到一定姿态时，重力加速度会在三轴上产生相应的分量，其本质是大地坐标系下的（0，0，g）在新的加速度计自身坐标系下的坐标，从而加速度计读到的三个数值就是（0，0，g）向量的新坐标。

按照上述ZYX顺序的旋转方式，通过向量（0，0，g）先左乘$\mathbf{R}_z$再左乘$\mathbf{R}_y$最后左乘$\mathbf{R}_x$的方式得到：

$$
\left[
\begin{matrix}
  a_x \\ a_y \\ a_z\\
\end{matrix}
\right]                                                                         =                                                                            \mathbf{R}_x \cdot \mathbf{R}_y \cdot \mathbf{R}_z \cdot                          \left[
\begin{matrix}
  0 \\ 0 \\ g \\
\end{matrix}
\right]                                                                                                             \\=                                  \left[
\begin{matrix}
  \cos p \cos y & \cos p \sin y & -\sin p \\                                             \cos y \sin p \sin r - \cos r \sin y & \cos r \cos y + \sin p \sin r \sin y & \cos p \sin r \\       \sin r \sin y + \cos r \cos y \sin p & \cos r \sin p \sin y - \cos y \sin r & \cos p \cos r
\end{matrix}
\right] \cdot                          \left[
\begin{matrix}
  0 \\ 0 \\ g \\
\end{matrix}
\right]                                \\=                                                                     \left[
\begin{matrix}
  -\sin p \\ \cos p \sin r \\ \cos p \cos r\\
\end{matrix}
\right] \cdot g                                                    
$$

通过求解上述方程，可以得到roll和pitch角，因为绕Z轴旋转没有产生重力加速度的分量，所以加速度计无法计算yaw角：

$$
r = \arctan(a_y / a_z) \\               p = -\arctan(a_x / \sqrt{a_y^2 + a_z^2})
$$

其中三次旋转过程的分解流程如下：

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%202.png)

参考：[https://blog.51cto.com/u_15262460/2883172](https://blog.51cto.com/u_15262460/2883172)

**2.4 欧拉角转四元数**

定义一个单位四元数：

$$
\bm q = \cos\frac{\theta}{2} + \bm u \sin\frac{\theta}{2}
$$

其中以$\bm u$ 作为旋转轴，旋转了$\theta$角度。

按照旋转顺序: ZYX，可以得到以下关于各自四元数的信息：

| 旋转角 | 旋转轴 | 旋转轴单位向量 | 对应四元数 |
| --- | --- | --- | --- |
| 偏航角 y | Z | (0, 0, 1) | $\bm {q}_{y} = \cos \frac{y}{2}+ 0 \bm i + 0 \bm j + \sin \frac{y}{2} \bm k$ |
| 俯仰角 p | Y | (0, 1, 0) | $\bm {q}_{p} = \cos \frac{p}{2}+ 0 \bm i + \sin \frac{p}{2} \bm j + 0 \bm k$ |
| 滚转角 r | X | (1, 0, 0) | $\bm {q}_{r} = \cos \frac{r}{2} + \sin \frac{r}{2}  \bm i + 0 \bm j + 0 \bm k$ |

根据上述旋转顺序，可以得到三次旋转之后的四元数：

$$
\bm q^I_W = q_0 + q_1 \bm i + q_2 \bm j + q_3 \bm k = \bm q_r \bm q_p \bm q_y
$$

该四元数表示的是从大地坐标系$W$ 到IMU坐标系$I$ 的旋转，进而可得：

$$
\begin{aligned}                                                         q_0 &= \cos \frac{y}{2} \cos \frac{p}{2} \cos \frac{r}{2} - \sin \frac{y}{2} \sin \frac{p}{2} \sin \frac{r}{2}                        \\                                  q_1 &= \cos \frac{y}{2} \cos \frac{p}{2} \sin \frac{r}{2} + \sin \frac{y}{2} \sin \frac{p}{2} \cos \frac{r}{2}                        \\                               q_2 &= \cos \frac{y}{2} \sin \frac{p}{2} \cos \frac{r}{2} - \sin \frac{y}{2} \cos\frac{p}{2} \sin \frac{r}{2}                           \\                                   q_3 &= \sin \frac{y}{2} \cos \frac{p}{2} \cos \frac{r}{2} + \cos \frac{y}{2} \sin \frac{p}{2} \sin \frac{r}{2}                        \\                                                                                              \end{aligned}                                                                                                    
$$

参考： [https://www.sohu.com/a/158723341_464087](https://www.sohu.com/a/158723341_464087)

另外，后续会用到四元数和旋转矩阵的转换，一般来说四元数转到旋转矩阵更多：

$$
\bm R^I_W =                        \left[ \begin{matrix}                                       		 		 q_0^2 + q_1^2 - q_2^2 - q_3^2 & 	 2 \cdot q_1 \cdot q_2 - 2 \cdot q_0 \cdot q_3 &						 2 \cdot q_1 \cdot q_3 + 2 \cdot q_0 \cdot q_2 \\
 2 \cdot q_1 \cdot q_2 + 2 \cdot q_0 \cdot q_3 &               q_0^2 - q_1^2 + q_2^2 - q_3^2  &  	 2 \cdot q_2 \cdot q_3 - 2 \cdot q_0 \cdot q_1 
\\  2 \cdot q_1 \cdot q_3 - 2 \cdot q_0 \cdot q_2 & 						  2 \cdot q_2 \cdot q_3 + 2 \cdot q_0 \cdot q_1 &							 q_0^2 - q_1^2 - q_2^2 + q_3^2 \\                                 \end{matrix}\right]                                  
$$

**2.5 IMU数据预处理**

- 陀螺仪测量角速度

陀螺仪的测量模型为：

$$
\bm \omega_m = \bm \omega + \bm \omega_{bias}
$$

其中$\bm \omega_m$为三轴角速度测量向量，由理想测量向量$\bm \omega$和测量误差$\bm \omega_{bias}$构成，其中每个轴的角速度正负号尤其关键，同样地，绕轴的逆时针为正方向，如图所示：

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%203.png)

- 加速度计测量加速度分量

加速度计测量模型为：

$$
\bm \alpha_m = \bm R(\bm \alpha - \bm g) + \bm \alpha_{bias}
$$

其中$\bm \alpha_m = [a_x, a_y, a_z]$为三轴角速度测量向量，由理想测量向量$\bm \alpha_m$和测量误差$\bm \alpha_{bias}$构成，$\bm R$ 为从大地坐标系到IMU自身坐标系的旋转矩阵，$|\bm g| = 9.81$。

要确定$a_x,a_y,a_z$的正负号，那么就观察 $-\bm g$ 在imu坐标系发生旋转后的坐标系中的投影向量方向。下图展示了$a_x,a_y,a_z$全部为正的情况：

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%204.png)

2**.6 IMU数据校准**

- 陀螺仪的角速度测量校准

静置IMU模块，存下一系列角速度的测量值（300个），然后求平均得到角速度的bias，后续再减去这个bias就基本能消除零偏

- 加速度计测量校准

偏差校准 (bias calibration, or offset calibration)
这里介绍 acc offset 校准方法。以 z 轴为例，设备正面朝上静止放置在水平面上时，z 测量值计为Z_up; 设备正面朝下静止放置在水平面上时， z 测量值计为 Z_dwn; 理想情况下 Z_up = +9.81 m/s^2, Z_dwn = -9.81 m/s^2。当然实际上由于 offset 的存在，Z_up 和 Z_dwn都会有偏差。offset的计算公式就是：

```
/* 偏差 = （向上 + 向下）/ 2 */
Z_offset = (Z_up + Z_dwn)/2.

/* 稍微精确一些，例如取100个Z_up, 求均值 Z_up_mean, 取100个 Z_dwn，求均值Z_dwn_mean.*/
Z_offset = (Z_up_mean + Z_dwn_mean) / 2

```

得到Z轴的offset之后同样的对X轴和Y轴做offset校准即可。

2**.7 IMU数据融合**

这里先定义$I$ 为IMU坐标系，$W$为大地坐标系。Mahony滤波器是一种考虑四元数空间中流形变换的互补滤波器，Mahony滤波器的总体思想是通过融合/组合陀螺仪测量值和加速度计测量值获得的方向的姿态估计值来估计姿态/角度/方位，所以用来估计IMU的姿态。

**算法流程如下：**

1. 获取传感器测量值：$^{I}\bm \omega_t$和$^{I}\bm \alpha_t$ 分别指代$t$时刻的角速度和加速度测量值，$^{I}\bm {\hat{\alpha}}_t$为归一化后的加速度。规定初始时刻的四元数$\bm q_0$由加速度得到的欧拉角计算获得。
2. 计算由上一时刻估计的四元数得到的重力加速度分量：

$$
\bm v({}^I_W \hat{\bm q}_{est,t-1}) = {}^I_W \bm  R \cdot \bm {\hat g} = \left[
\begin{matrix}
2(q_1q_3 + q_0q_2) \\                               2(q_2q_3 - q_0q_1)   \\                (q^2_0 - q^2_1 - q^2_2 + q^2_3)                                                                                    
\end{matrix}
\right] 
$$

1. 计算加速度计测量值和第二步估计的加速度分量的差值：

$$
\bm e_{t} = ^{I}\bm {\hat{\alpha}}_t \times \bm v({}^I_W \hat{\bm q}_{est,t-1})
$$

这里的差值由两个三维向量的叉乘表示，同时也表示旋转估计的偏差，这里的偏差几乎很小。

1. 计算第三步的误差积分：

$$
\bm e_{i, t} = \bm e_{i, t-1} + \bm e_t \Delta t
$$

其中$\Delta t$为采样时间。

1. 利用PI控制器修正陀螺仪的角速度测量值：

$$
{}^I \bm \omega_{corr, t} = {}^I \bm \omega_t + \bm K_p \bm e_t + \bm K_i \bm e_{i,t}
$$

1. 得到修正后四元数的增量，即四元数求导：

$$
{}^I_W \dot{\bm q}_{\bm \omega,t}     = \frac{1}{2} \cdot {}^I_W \hat{\bm q}_{est,t-1} \otimes [0, {}^I \bm \omega_{corr, t}]^\top
$$

1. 利用四元数增量，在采样时间内累积后得到姿态的修正量，从而更新姿态：

$$
{}^I_W {\bm q}_{est,t} = {}^I_W \hat{\bm q}_{est,t-1} + {}^I_W \dot{\bm q}_{\bm \omega,t} \cdot \Delta t
$$

最后在每一个时刻重复迭代1 - 7步骤，得到融合滤波后的四元数姿态。

这里的$\bm K_p$ 和$\bm K_i$ 是经验调参后的参数，需要通过姿态曲线反复修正。

参考：[https://nitinjsanket.github.io/tutorials/attitudeest/mahony#mahonyfilt](https://nitinjsanket.github.io/tutorials/attitudeest/mahony#mahonyfilt)

### 3-物理建模模型

双轮平衡小车由车体和双轮两部分组成，可以看成一个移动的倒立摆，分别对车轮和车体进行力学分析，建立动力学模型，最后，通过对两者的分析给出系统的状态空间表达式。

首先给出物理变量：

| 物理量 | 描述 | 单位 |
| --- | --- | --- |
| $m$ | 车轮的质量 | $kg$ |
| r | 车轮的半径 | $m$ |
| $ x_R$, $ x_L$ | 右轮的水平位移 | $m$ |
| $H_{fR}$, $H_{fL}$ | 右轮受到地面的摩擦力的大小 | $N$ |
| $H_R$, $H_L$ | 右轮受到车体作用力的水平分力的大小 | $N$ |
| $T_R$, $T_L$ | 右轮电机输出的转矩的大小 | $N \cdot m$ |
| $I$ | 车轮的转动惯量 | $kg \cdot m^2$ |
| $\omega_R$, $\omega_L$ | 右轮的角速度的大小 | $rad / s$ |

**3.1 车轮模型**

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%205.png)

将车轮分解为平动和转动，

由牛顿第二定律可得：

$$
m \ddot{{x}}_R = H_{fR} - {H}_R \quad (1)
$$

由刚体定轴转动定律可得：

$$
I\dot{\omega}_R = T_R - H_{fR} \cdot r \quad (2)
$$

联立上述二式可得：

$$
m \ddot{x}_R = (T_R - I\dot{\omega}_R) / r - H_R \quad (3)
$$

假设车轮不打滑，车轮移动速度大小和转动速度大小成比例关系，即：

$$
\omega_R = \dot{x}_R / r \\                              \dot{\omega}_R = \ddot{x}_R / r 
$$

于是可得：

$$
(m + 1/r^2)\ddot{x}_R = T_R/r - H_R \quad (4)
$$

左右轮参数相同，于是对左轮有：

$$
(m + 1/r^2)\ddot{x}_L = T_L/r - H_L \quad (5)
$$

**3.2 小车运动模型**

小车的正向运动可以分解为前向运动和绕车体质心P的相对转动（俯仰）。小车底盘中心O的水平位移为$x_O = (x_L + x_R) / 2 \quad (6)$

将方程(4)(5)想加，等式两边除以$2$ 可得:

$$
(m+I/r^2)(\ddot{x}_L + \ddot{x}_R)/2 = (T_L+T_R)/2r - (H_L + H_R)/2 \quad (7)
$$

将(6)带入(7)可得：

$$
(m+I/r^2)\ddot{x} = (T_L+T_R)/2r - (H_L + H_R)/2 \quad (8)
$$

其正向运动模型为：

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%206.png)

对车体，应用牛顿第二定律可得，

在水平方向上，有

$$
M\frac{d(x+l\sin\theta_P)}{dt^2} = H_L+H_R  \quad (9)
$$

在竖直方向上，有

$$
M\frac{d(l\cos\theta_P)}{dt^2} = V_L+V_R - Mg  \quad (10)
$$

对车体，由刚体定轴转动定律可得：

$$
J_P\ddot{\theta}_P = (V_L+V_R)l\sin\theta_P - (H_L+H_R)l\cos\theta_P - (T_L+T_R)  \quad (11)
$$

其中，

| 物理量 | 描述 | 单位 |
| --- | --- | --- |
| $M$ | 车体的质量 | $kg$ |
| $l$ | 质心距底盘中心的距离 | $m$ |
| $J_P$ | 车体绕质心转动时的转动惯量 | $kg\cdot m^2$ |
| $\theta_P$ | 车体与竖直方向所成的夹角 | $rad$ |
| $V_L$, $V_R$ | 车体受到车轮作用力的竖直分力的大小 | $N$ |

### 4-平衡控制算法

这里用的是串级PID算法，先定义平衡车的状态空间：

$$
\bm X^* = \left[\begin{matrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{matrix} \right]
$$

其中,

$$
x = (l_L+l_R) / 2 \\                                                                       \dot{x} = (v_L+v_R) / 2 \\            \theta = \theta_P  \\                      \dot{\theta} = \omega_P
$$

定义位置误差为：

$$
e_x = x_d - x
$$

位置环的PI控制器为：

$$
\theta_d = K_{x,p} \cdot e_x + K_{x,i} \int e_x
$$

定义角度环的角度误差为：

$$
e_{\theta} = \theta_d - \theta
$$

那么角度环的比例控制器为：

$$
\omega_d = K_{\theta,p} \cdot e_\theta 
$$

定义角速度环误差：

$$
e_{\omega} = \omega_d - \omega
$$

角速度环的PI控制器为：

$$
v_d = K_{\omega,p} \cdot e_\omega + K_{\omega,i} \int e_\omega \\        (v_{L,d} + v_{R,d}) / 2 = v_d       \\ v_{L,d} = v_{R,d} = v_d
$$

最后pwm环为：

$$
e_v = v_d - v \\                                                                                 u_{input} = K_{v,p} \cdot e_v + K_{v,i} \int e_v
$$

$u_{input}$输入给定时器的pwm输出通道的占空比，控制电机转速来使平衡车达到期望状态。

### 5 - 元器件购买list

- stm32f1c8t6最小系统板: [https://detail.tmall.com/item.htm?_u=f25u67823715&id=619901805655&spm=a1z09.2.0.0.67002e8dY0q5kk](https://detail.tmall.com/item.htm?_u=f25u67823715&id=619901805655&spm=a1z09.2.0.0.67002e8dY0q5kk)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%207.png)

- stlink程序烧录器：[https://detail.tmall.com/item.htm?_u=f25u67823715&id=619901805655&spm=a1z09.2.0.0.67002e8dY0q5kk](https://detail.tmall.com/item.htm?_u=f25u67823715&id=619901805655&spm=a1z09.2.0.0.67002e8dY0q5kk)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%208.png)

- 降压稳压模块12v→5v：[https://detail.tmall.com/item.htm?_u=f25u67825ba0&id=40142354852&skuId=4087889854295&spm=a1z09.2.0.0.67002e8dY0q5kk](https://detail.tmall.com/item.htm?_u=f25u67825ba0&id=40142354852&skuId=4087889854295&spm=a1z09.2.0.0.67002e8dY0q5kk)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%209.png)

- MPU9250（IMU）：[https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8dY0q5kk&id=642706214609&_u=f25u6782af0e](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8dY0q5kk&id=642706214609&_u=f25u6782af0e)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%2010.png)

- 双轮底盘（两个直流电机）：[https://detail.tmall.com/item.htm?_u=f25u6782d167&id=609074662884&spm=a1z09.2.0.0.67002e8dY0q5kk](https://detail.tmall.com/item.htm?_u=f25u6782d167&id=609074662884&spm=a1z09.2.0.0.67002e8dY0q5kk)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%2011.png)

- 电机驱动模块：[https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8dY0q5kk&id=43963555491&_u=f25u6782598f](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8dY0q5kk&id=43963555491&_u=f25u6782598f)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%2012.png)

- CP2102串口模块：[https://detail.tmall.com/item.htm?_u=f25u678270f1&id=44376093268&spm=a1z09.2.0.0.67002e8dY0q5kk](https://detail.tmall.com/item.htm?_u=f25u678270f1&id=44376093268&spm=a1z09.2.0.0.67002e8dY0q5kk)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%2013.png)

- 杜邦线（尽量长）：[https://detail.tmall.com/item.htm?_u=f25u67825a24&id=14466195609&skuId=3495100573839&spm=a1z09.2.0.0.67002e8dY0q5kk](https://detail.tmall.com/item.htm?_u=f25u67825a24&id=14466195609&skuId=3495100573839&spm=a1z09.2.0.0.67002e8dY0q5kk)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%2014.png)

- 12v航模电池：[https://item.taobao.com/item.htm?spm=a21n57.1.0.0.4efc523c2Zz2j1&id=662762928573&ns=1&abbucket=10#detail](https://item.taobao.com/item.htm?spm=a21n57.1.0.0.4efc523c2Zz2j1&id=662762928573&ns=1&abbucket=10#detail)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%2015.png)

- 电池充电器：[https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15496292193.23.4bfb28c2lTp0ab&id=543497662500](https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15496292193.23.4bfb28c2lTp0ab&id=543497662500)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%2016.png)

- 烙铁+焊锡：[https://detail.tmall.com/item.htm?abbucket=10&id=604287077536&ns=1&skuId=4403291878306&spm=a21n57.1.0.0.2245523cgBxOHu](https://detail.tmall.com/item.htm?abbucket=10&id=604287077536&ns=1&skuId=4403291878306&spm=a21n57.1.0.0.2245523cgBxOHu)

![Untitled](%E5%8F%8C%E8%BD%AE%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%201ad9dd91683f4bce849c50e5ba27facd/Untitled%2017.png)

[list](https://www.notion.so/list-ddf6a6500f72437f9c947743332c849a?pvs=21)
