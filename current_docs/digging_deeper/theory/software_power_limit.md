<!-- <script type="text/javascript" src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=default"></script> -->

# 软件底盘功率限制
软件功率限制通过限制电机输出力矩来控制底盘功率，以达到在底盘剧烈运动、实际功率即将超过给定的最大功率时，将实际功率限制到最大功率附近以下的目的。

## 理论
电机的输出功率为：
$$ {eq:p_out}
\begin{equation}
    P_{out} = \tau \omega
    \label{eq:p_out}
\end{equation}
$$

电机的输入功率为输出功率和损失功率的和，可表示为：
$$
\begin{equation}
    P_{in} = P_{out} + k_1 \tau^2 + k_2 \omega^2
    \label{eq:p_in}
\end{equation}
$$
其中$k_1$和 $k_2$为常数。

## 实现
$\eqref{eq:p_in}$的常数较难测得，我们近似使用$\eqref{eq:p_out}$。已知：最大功率 $P\_{max}$、各电机当前转速 $\omega\_{real}$ 和电机PID计算出来的原始力矩指令 $\tau_{cmd}$ ，根据原始力矩指令等比例分配，我们可以计算出各个电机输出的最大力矩：
$$
\begin{equation}
    \tau_{max}=
         k \frac{\tau_{cmd}} {\tau_{total}} \frac{P_{max}}{\omega_{real}} 
    \label{eq:tau_max}
\end{equation}
$$
其中$k$安全系数，通过调节$k$的大小可以保守地减小损失功率的影响；$k_1$为所有电机的命令力矩的绝对值之和。
为防止电机转速低时，计算出允许的最大力矩过大，我们引入最低转速$\omega_{min}$当电机转速低于最低转速时，使用最低转速计算最大力矩。结合 \eqref{eq:tau_max} 最终计算电机最大输出力矩的公式为。
$$
\begin{equation}
    \tau_{max}=\left\{\begin{array}{lcr}
         k \frac{\tau_{cmd}} {\tau_{total}} \frac{P_{max}}{\omega_{min}} 
         &,\omega_{real} < \omega_{min}
         \\
         k \frac{\tau_{cmd}  } {\tau_{total}  } \frac{P_{max}}{\omega_{real}}  
         &,\omega_{real} \geq \omega_{min} \\
    \end{array}\right.
    \label{eq:tau_max_min}
\end{equation}
$$
下图展示了步兵机器人在 静止状态下进入高速小陀螺后再进入慢速小陀螺最后在高速小陀螺状态下进行平移的功率（最上）、速度指令（中间）和实际速度（最下）的曲线，可见在高速小陀螺状态下，功率被限制在了红线附近之下，实际速度比速度指令小，进入慢速小陀螺后，功率降低，此时实际速度与速度指令相等，最后在高速小陀螺状态下平移时，功率依然被限制在了青色线(最大功率)附近以下。