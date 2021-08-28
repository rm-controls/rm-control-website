# Introduction
In practice, the target to be hit tends to move, and the bullet will be dragged in the air and the launch of the barrel will be delayed. We need to make a barrel angle solver to predict the firing angle of the barrel to achieve more accurate strikes.

Purpose:
- Consider air resistance (proportional to speed)
- Consider target speed
- Consider the launch delay

Difficulties:
- No analytical solution
- The target solution frequency is greater than 1Khz

Two algorithms:
- Iterative method:
  - Advantages: higher accuracy (when there are many iterations)
  - Disadvantages: slow
- Velocity stacking method:
  - Advantages: fast speed (within 20us)
  - Disadvantages: inherent error
<br/>

- Algorithm block diagram:
![BVZR9P.png](https://s1.ax1x.com/2020/10/24/BVZR9P.png)

# Model derivation
Air resistance is proportional to speed, and the target point moves at a constant speed.
2D Model: In the xz plane, assuming that the bullet launching point is the coordinate origin, the actual position of the bullet is equivalent to a vector in the xoz plane.
![BEgFbT.png](https://s1.ax1x.com/2020/10/23/BEgFbT.png)
Use to represent the component of the bullet position vector in the X direction, $\vec{v_x}, \vec{v_z}$ represent the component of the bullet velocity on the x axis and z axis, respectively,and use $\vec{z}$ to represent the component of the bullet position vector in the z direction; $v_0$ represents the initial launch velocity of the bullet, k is the air resistance coefficient, g is the acceleration of gravity, m is the weight of the bullet, and $f_x$ is the position of the bullet. The component of air resistance in the X direction.Based on physical knowledge:
$$\vec{f_x}=-k\vec{v_x}=m\frac{d\vec{v_x}}{dt}\tag{1}$$
Sort (1) to get:
$$-\frac kmdt=\frac{d\vec{v_x}}{v_x}$$
Taking the time of bullet launching as time 0, the component of the initial launch velocity in the x direction is $\vec{v_x}$, Integrate the above formula:
$$\int_0^t-\frac kmdt=\int_{v_{x_0}}^{v_x}\frac{dv_x}{v_x}\tag{2}$$
Solved by (2): 
$$v_x=\frac{dx}{dt}=v_{x_0}e^{-\frac kmt}$$ 
Integrate both sides of the above formula to obtain:
$$x=\frac mk v_{x_0}(1-e^{-\frac kmt})\tag{3}$$
At the same time, the resistance of the bullet in the z-axis direction can be obtained as:
$$\vec{f_z}=-k\vec{v_z}-mg=m\frac{d\vec{v_z}}{dt}$$
Sort out the differential equation:
$$\frac{dv_z}{dt}+\frac kmv_z+g=0\tag{4}$$
Solutions have to:
$$z=\frac km\left(v_{z_0}+\frac{mg}k\right)\left(1-e^{-\frac kmt}\right)-\frac{mg}kt\tag{5}$$
Target point motion model: Suppose the starting position of the target point is $\left(x_{t_0}, z_{t_0}\right)$, The speed of the target point in the x-axis direction is $v_{t_x}$, the speed in the z-axis direction is $v_{t_z}$, the component of the actual position of the target point in the x-axis direction is $x_t$, in the z-axis The component of the direction is $z_t$.
$$x_t=x_{t_0}+v_{t_x}t\\z_t=z_{t_0}+v_{t_z}t$$ 
Based on the derivation of the above 2D model, use y to represent the component of the bullet position vector in the y-axis direction, $v_{y_0}$ represents the component of $v_0$ in the y-axis direction; the actual position of the target point in the y-axis direction component is $y_t $, the y-axis coordinate of the starting position is $y_{t_0}$, and the speed in the y-axis direction is $v_{t_y}$, the movement model of the bullet and the target point in the y direction under the 3D model can be obtained:
$$ y = \frac mkv_{y_0}(1-e^{-\frac kmt})$$ $$ y_t = y_{t_0} + v_{t_y}t$$

# Code
## Base class
```cpp
template<typename T>
class BulletSolver {
 public:
  BulletSolver(T resistance_coff, T g, T delay, T dt, T timeout) :
      resistance_coff_(resistance_coff),
      dt_(dt), g_(g), delay_(delay),
      timeout_(timeout) {};
  virtual ~BulletSolver() = default;
  virtual void setTarget(const T *pos, const T *vel) = 0;
  virtual void setBulletSpeed(T speed) { bullet_speed_ = speed; };
  virtual void solve(const T *angle_init) = 0;
  virtual void output(T *angle_solved) = 0;
 protected:
  T bullet_speed_{};
  T resistance_coff_, g_, dt_, timeout_, delay_;
};
```
The BulletSolver class is the base class of all models and algorithms. It defines the algorithm function interface for solving the bullet launch angle, and member variables such as air drag coefficient, gravitational acceleration, launch delay, and bullet velocity.

#### Function description
* setTarget() 

   Set the target point, used to initialize the initial position and speed of the target point

* setBulletSpeed() 

   Set bullet speed, used to initialize bullet launch speed

* solve()

   Calculation, used to calculate the final barrel angle

* output()

   Output, used to output the final barrel angle

#### Variable description
* bullet_speed_(`T`)

   Initial bullet velocity

* resistance_coff_

   Coefficient of air resistance

* g_

   Acceleration of gravity

* dt_

   Loop iteration interval

* timeout_

   The condition to exit the loop, when the calculated time of the bullet flying in the air exceeds this value, it will exit the loop

* delay_

   Barrel launch delay

## Bullet motion model
```cpp
rt_bullet_rho = (1 / this->resistance_coff_) * bullet_v_rho
        * (1 - std::exp(-this->fly_time_ * this->resistance_coff_));

rt_bullet_z = (1 / this->resistance_coff_)
      * (bullet_v_z + this->g_ / this->resistance_coff_)
      * (1 - std::exp(-this->fly_time_ * this->resistance_coff_))
      - this->fly_time_ * this->g_ / this->resistance_coff_;        
```

#### Variable description
* bullet_v_rho(`T`)

   The superposition of bullet velocity $v_x$ and $v_y$

* rt_bullet_rho

   The actual position of the bullet is superimposed on the x-axis component and the y-axis component

* rt_bullet_z

   The actual position of the bullet is on the z-axis component

* fly_time_

   Bullet flight time

## Target point motion model
```cpp
rt_target_x += this->target_dx_ * this->dt_;
rt_target_y += this->target_dy_ * this->dt_;
```

#### Variable description
* rt_target_x(`T`)

   The actual position of the target point on the x-axis component

* rt_target_y

   The actual position of the target point on the y-axis component

* target_dx_

   The target point velocity on the x axis component

* target_dy_

   The target point velocity on the y-axis component

Please refer to the [bullet_solver.cpp]() for the specific implementation of all algorithms.

# Test program
## Include header file
```cpp
#include <iostream>
#include "bullet_solver.h"
```
The header file contains the definitions of all classes and functions.

## Create class object
```cpp
int main(int argc, char **argv) {
  Iter2DSolver<double> iter2d(0.1, 9.8, 0.01, 0.0001, 3.);
  Approx2DSolver<double> approx2d(0.1, 9.8, 0.01, 0.01, 3.);
  Iter3DSolver<double> iter3d(0.1, 9.8, 0.01, 0.0001, 3.);
  Approx3DSolver<double> approx3d(0.1, 9.8, 0.01, 0.0001, 3.);
  ```

  #### Variable description
  * iter2d(`Iter2DSolver`)

     Object instance of iterative algorithm class of 2D model

  * approx2d(`Approx2DSolver`)

     An instance of the velocity superposition algorithm class object of the 2D model

  * iter3d(`Iter3DSolver`)

     The iterative algorithm class object instance of 3D model

  * approx3d(`Approx3DSolver`)

     3D model speed superposition algorithm class object instance

## Set parameters
Take the iterative algorithm of the 3D model as an example
```cpp
  double angle_init[2]{}, angle_solved[2]{};
  double bullet_speed = 18.;
  double pos_3d[3] = {7, 0, 1};
  double vel_3d[3] = {0, 1, 0};
  iter3d.setBulletSpeed(bullet_speed);
  iter3d.setTarget(pos_3d, vel_3d);
  ```

  #### Parameter Description
  * angle_init(`double`)

     Customized initial launch angle

  * angle_solved(`double`)

     Calculated launch angle

  * bullet_speed(`double`)

     Initial velocity of bullet

  * pos_3d(`double[]`)

     Initial coordinates of the target point

  * vel_3d(`double[]`)

     The speed of the target point in the x, y, z direction

## Calculate and output the launch angle
```cpp
  iter3d.solve(angle_init);
  iter3d.output(angle_solved);
  std::cout << "yaw:" << angle_solved[0] << " pitch:" << angle_solved[1] << std::endl;
```