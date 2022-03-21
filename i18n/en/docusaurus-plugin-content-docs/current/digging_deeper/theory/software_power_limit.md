---
sidebar_position: 1
---

# Software Chassis Power Limiting

Software power limiting controls chassis power by limiting the motor output torque for the purpose of limiting the actual power to below near the maximum power when the chassis is moving violently and the actual power is about to exceed the given maximum power.

## Theory

The output power of the motor is.

$$
P_{out} = \tau \omega\tag{1}
$$

The input power of the motor is the sum of the output power and the lost power, which can be expressed as

$$
P_{in} = P_{out} + k_1 \tau^2 + k_2 \omega^2\tag{2}
$$

where $$ k_1$$ and $$ k_2$$ are constants.

## Implementation

The constants of equation (2) are harder to measure, and we approximate using equation (1). Knowing: the maximum power $P_{max}$, the current speed of each motor $\omega_{real}$ and the raw torque command $\tau_{cmd}$ calculated by the motor PID, we can calculate the maximum torque output from each motor according to the equiproportional distribution of the raw torque command.

$$
\tau_{max}=k \frac{\tau_{cmd}} {\tau_{total}} \frac{P_{max}}{\omega_{real}}\tag{3}
$$

where $$k$$ safety factor, the effect of lost power can be conservatively reduced by adjusting the magnitude of $$k$$; $$\tau_{total}$$ is the sum of the absolute values of the command moments of all motors.
To prevent the maximum torque allowed from being calculated too large when the motor speed is low, we introduce the minimum speed $\omega_{min}$ to calculate the maximum torque using the minimum speed when the motor speed is lower than the minimum speed. Combining with equation (3) the final formula for calculating the maximum output torque of the motor is:

$$
\tau_{max}=\left\{\begin{array}{lcr}
         k \frac{\tau_{cmd}} {\tau_{total}} \frac{P_{max}}{\omega_{min}}
         &,\omega_{real} < \omega_{min}
         \\\
         k \frac{\tau_{cmd} } {\tau_{total}} \frac{P_{max}}{\omega_{real}}
         &,\omega_{real} \geq \omega_{min} \\
    \end{array}\right.\tag{4}
$$

The following figure shows the curves of power (top), speed command (middle) and actual speed (bottom) for the infantry robot entering the high-speed small gyro at rest and then entering the slow small gyro and finally panning in the high-speed small gyro state. It can be seen that in the high-speed small gyro state, the power is limited below near the red line and the actual speed is smaller than the speed command, and after entering the slow small gyro, the power decreases, at which time the actual speed is equal to the speed command, and finally when panning in the high-speed small gyro state, the power is still limited below near the cyan line (maximum power).
![Software chassis power limit](/img/digging_deeper/software_power_limit.png)
