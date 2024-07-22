# Trajectory planning for Automatic Machines and Robots (2008)

## 3.4 Trajectory with Double S Velocity Profile 

A trapezoidal (or triangular) velocity motion profile presents a discontinuous acceleration. For this reason, this trajectory may generate efforts and stresses on the mechanical system that may result detrimental or generate undesired vibrational effects (see Chapter 7 for a more detailed discussion on these aspects). Therefore, a smoother motion profile must be defined, for example by adopting a continuous, linear piece-wise, acceleration profile as shown in Fig. 3.13. In this manner, the resulting velocity is composed by linear segments connected by parabolic blends. The shape of the velocity profile is the reason of the name double $S$ for this trajectory, known also as bell trajectory or seven segments trajectory, because it is composed by seven different tracts with constant jerk. Since the jerk is characterized by a step profile, the stress and the vibrational effects generated on the transmission chain and on the load by this motion profile are reduced with respect to trapezoidal velocity trajectories, characterized by an impulsive jerk profile.

Let us assume that

$$
\mathrm{j}_{\min }=-\mathrm{j}_{\max }, \quad \mathrm{a}_{\min }=-\mathrm{a}_{\max }, \quad \mathrm{v}_{\min }=-\mathrm{v}_{\max }
$$

where $\mathrm{j}_{\min }$ and $\mathrm{j}_{\max }$ are the minimum and maximum value of the jerk, respectively. With these conditions, it is desired to plan a trajectory that, when possible, reaches the maximum (minimum) values for jerk, acceleration and velocity so that the total duration $T$ is minimized (minimum time trajectory). Only the case $q_{1}>q_{0}$ is now considered. The case $q_{1}<q_{0}$ is addressed in the following Sec. 3.4.2. Moreover, for the sake of simplicity, the value $t_{0}=0$ is assumed. The boundary conditions are:

- Generic initial and final values of velocity $\mathrm{v}_{0}, \mathrm{v}_{1}$.
- Initial and final accelerations $\mathrm{a}_{0}, \mathrm{a}_{1}$ set to zero.

Three phases can be distinguished:

1. Acceleration phase, $t \in\left[0, T_{a}\right]$, where the acceleration has a linear profile from the initial value (zero) to the maximum and then back to zero.
2. Maximum velocity phase, $t \in\left[T_{a}, T_{a}+T_{v}\right]$, with a constant velocity.
3. Deceleration phase, $t \in\left[T_{a}+T_{v}, T\right]$, being $T=T_{a}+T_{v}+T_{d}$, with profiles opposite with respect to the acceleration phase.

Given the constraints on the maximum values of jerk, acceleration and velocity, and given the desired displacement $h=q_{1}-q_{0}$, the trajectory is computed by using (3.30a)-(3.30g). However, it is first necessary to verify whether a trajectory can be actually performed or not. As a matter of fact, there are several cases in which a trajectory cannot be computed with the given constraints. For example, if the desired displacement $h$ is small with respect to the difference between the initial and final velocities $\mathrm{v}_{0}$ and $\mathrm{v}_{1}$, it might be not possible

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-02.jpg?height=1208&width=911&top_left_y=164&top_left_x=283)

Fig. 3.13. Typical profiles for position, velocity, acceleration and jerk for the double S trajectory.

to change the velocity (with the given limits on jerk and acceleration), while accomplishing the displacement $h$.

The limit case is represented by a single acceleration (if $\mathrm{v}_{0}<\mathrm{v}_{1}$ ) or deceleration (if $\mathrm{v}_{0}>\mathrm{v}_{1}$ ) phase. Therefore, it is first necessary to check if it is possible to perform the trajectory with a double jerk impulse (one positive and one negative) only. For this purpose, let us define

$$
\begin{equation*}
T_{j}^{\star}=\min \left\{\sqrt{\frac{\left|\mathrm{v}_{1}-\mathrm{v}_{0}\right|}{\mathrm{j}_{\max }}}, \quad \frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }}\right\} \tag{3.17}
\end{equation*}
$$

If $T_{j}^{\star}=\mathrm{a}_{\max } / \mathrm{j}_{\max }$, the acceleration reaches its maximum value and a segment with zero jerk may exist.

Then, the trajectory is feasible if

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-03.jpg?height=205&width=963&top_left_y=164&top_left_x=230)

If this inequality holds, it is possible to compute the trajectory parameters. In this case, by defining the maximum value of the velocity during the motion as $\mathrm{v}_{\text {lim }}=\max (\dot{q}(t))$, there are two possibilities:

Case 1: $\quad \mathrm{v}_{\text {lim }}=\mathrm{v}_{\text {max }}$.

Case 2: $\mathrm{v}_{\text {lim }}<\mathrm{v}_{\text {max }}$.

In the latter case, that can be verified only after the computation of the trajectory's parameters, the maximum velocity is not reached, and there is only an acceleration and a deceleration phase (no segment with constant velocity).

In both Case 1 and Case 2, it is possible that the maximum acceleration (positive or negative) is not reached. This may happen if the displacement is small, if the maximum allowed acceleration $\mathrm{a}_{\max }$ is high ("high dynamic" case), or if the initial (final) velocity is close enough to the maximum allowed speed. In these cases, the constant acceleration segment is not present. In particular, it is worth to notice that, because the different values of the initial and final velocities $v_{0}, v_{1}$, the amounts of time necessary to accelerate (from $\mathrm{v}_{0}$ to $\mathrm{v}_{\text {lim }}$ ) and to decelerate (from $\mathrm{v}_{\text {lim }}$ to $\mathrm{v}_{1}$ ) are in general different, and it may happen that the maximum acceleration $\mathrm{a}_{\max }$ is reached only in one of these phases, while in the other one the maximum acceleration is $\mathrm{a}_{\text {lim }}<\mathrm{a}_{\text {max }}$. Let us define:

$T_{j 1}$ : time-interval in which the jerk is constant $\left(\mathrm{j}_{\max }\right.$ or $\mathrm{j}_{\text {min }}$ ) during the acceleration phase;

$T_{j 2}$ : time-interval in which the jerk is constant $\left(\mathrm{j}_{\text {max }}\right.$ or $\mathrm{j}_{\text {min }}$ ) during the deceleration phase;

$T_{a}$ : acceleration period;

$T_{v}$ : constant velocity period;

$T_{d}$ : deceleration period;

$T$ : total duration of the trajectory $\left(=T_{a}+T_{v}+T_{d}\right)$

Case 1: $\mathrm{v}_{\text {lim }}=\mathrm{v}_{\text {max }}$.

In this case, it is possible to verify if the maximum acceleration ( $\mathrm{a}_{\max }$ or $\left.\mathrm{a}_{\min }=-\mathrm{a}_{\max }\right)$ is reached by means of the following conditions:

$$
\begin{align*}
& \text { if }\left(\mathrm{v}_{\max }-\mathrm{v}_{0}\right) \mathrm{j}_{\max }<\mathrm{a}_{\max }^{2} \Longrightarrow \mathrm{a}_{\max } \quad \text { is not reached; }  \tag{3.19}\\
& \text { if }\left(\mathrm{v}_{\max }-\mathrm{v}_{1}\right) \mathrm{j}_{\max }<\mathrm{a}_{\max }^{2} \Longrightarrow \mathrm{a}_{\min } \text { is not reached. } \tag{3.20}
\end{align*}
$$

Then, the time intervals of the acceleration segment can be computed if (3.19) holds as

$$
\begin{equation*}
T_{j 1}=\sqrt{\frac{\mathrm{v}_{\max }-\mathrm{v}_{0}}{\mathrm{j}_{\max }}}, \quad T_{a}=2 T_{j 1} \tag{3.21}
\end{equation*}
$$

otherwise as

$$
\begin{equation*}
T_{j 1}=\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }}, \quad T_{a}=T_{j 1}+\frac{\mathrm{v}_{\max }-\mathrm{v}_{0}}{\mathrm{a}_{\max }} \tag{3.22}
\end{equation*}
$$

The time intervals of the deceleration segment can be computed if (3.20) holds as

$$
\begin{equation*}
T_{j 2}=\sqrt{\frac{\mathrm{v}_{\max }-\mathrm{v}_{1}}{\mathrm{j}_{\max }}}, \quad T_{d}=2 T_{j 2} \tag{3.23}
\end{equation*}
$$

otherwise as

$$
\begin{equation*}
T_{j 2}=\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }}, \quad T_{d}=T_{j 2}+\frac{\mathrm{v}_{\max }-\mathrm{v}_{1}}{\mathrm{a}_{\max }} \tag{3.24}
\end{equation*}
$$

Finally, it is possible to determine the time duration of the constant velocity segment as

$$
\begin{equation*}
T_{v}=\frac{q_{1}-q_{0}}{\mathrm{v}_{\max }}-\frac{T_{a}}{2}\left(1+\frac{\mathrm{v}_{0}}{\mathrm{v}_{\max }}\right)-\frac{T_{d}}{2}\left(1+\frac{\mathrm{v}_{1}}{\mathrm{v}_{\max }}\right) \tag{3.25}
\end{equation*}
$$

If $T_{v}>0$, then the maximum velocity is actually reached and the values obtained by (3.21)-(3.25) can be used to compute the trajectory.

The condition $T_{v}<0$ simply means that the maximum velocity $\mathrm{v}_{\text {lim }}$ is smaller than $\mathrm{v}_{\max }$ and, therefore, the following Case 2 must be considered.

Example 3.9 Fig. 3.14 reports the position, velocity, acceleration and jerk for a double S trajectory when the constant velocity phase is present. The boundary conditions are

$$
q_{0}=0, \quad q_{1}=10, \quad \mathrm{v}_{0}=1, \quad \mathrm{v}_{1}=0
$$

while the constraints are

$$
\mathrm{v}_{\max }=5, \quad \mathrm{a}_{\max }=10, \quad \mathrm{j}_{\max }=30
$$

The resulting time intervals are

$$
T_{a}=0.7333, \quad T_{v}=1.1433, \quad T_{d}=0.8333, \quad T_{j 1}=0.3333, \quad T_{j 2}=0.3333
$$

Case 2: $\mathrm{v}_{\text {lim }}<\mathrm{v}_{\text {max }}$.

In this case, the constant velocity segment is not present $\left(T_{v}=0\right)$, and the duration of the acceleration and deceleration segments can be easily computed if the maximum/minimum accelerations are reached in both segments. In this case

$$
\begin{equation*}
T_{j 1}=T_{j 2}=T_{j}=\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }} \tag{3.26a}
\end{equation*}
$$

and

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-05.jpg?height=1179&width=827&top_left_y=176&top_left_x=352)

Fig. 3.14. Double S trajectory profiles (position, velocity, acceleration and jerk) including a constant velocity phase.

$$
\begin{align*}
T_{a}= & \frac{\frac{\mathrm{a}_{\max }^{2}}{\mathrm{j}_{\max }}-2 \mathrm{v}_{0}+\sqrt{\Delta}}{2 \mathrm{a}_{\max }}  \tag{3.26~b}\\
T_{d}= & \frac{\frac{\mathrm{a}_{\max }^{2}}{\mathrm{j}_{\max }}-2 \mathrm{v}_{1}+\sqrt{\Delta}}{2 \mathrm{a}_{\max }} \tag{3.26c}
\end{align*}
$$

where

$$
\begin{equation*}
\Delta=\frac{\mathrm{a}_{\max }^{4}}{\mathrm{j}_{\max }^{2}}+2\left(\mathrm{v}_{0}^{2}+\mathrm{v}_{1}^{2}\right)+\mathrm{a}_{\max }\left(4\left(q_{1}-q_{0}\right)-2 \frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }}\left(\mathrm{v}_{0}+\mathrm{v}_{1}\right)\right) \tag{3.27}
\end{equation*}
$$

Example 3.10 Fig. 3.15 reports the position, velocity, acceleration and jerk for a double S trajectory when the constant velocity phase is not present. The
boundary conditions are

$$
q_{0}=0, \quad q_{1}=10, \quad \mathrm{v}_{0}=1, \quad \mathrm{v}_{1}=0
$$

with constraints

$$
\mathrm{v}_{\max }=10, \quad \mathrm{a}_{\max }=10, \quad \mathrm{j}_{\max }=30
$$

The resulting time intervals are

$$
T_{a}=1.0747, \quad T_{v}=0, \quad T_{d}=1.1747, \quad T_{j 1}=0.3333, \quad T_{j 2}=0.3333
$$

and the maximum speed is $\mathrm{v}_{\text {lim }}=8.4136$.

If $T_{a}<2 T_{j}$ or $T_{d}<2 T_{j}$, then the maximum (minimum) acceleration is not reached in at least one of the two segments, and therefore it is not possible to use eq. (3.26a), (3.26b), (3.26c). In this case (indeed rather unusual), the determination of parameters is quite difficult, and it may be more convenient to find an approximated solution that, although not optimal, results acceptable from a computational point of view. A possible way to determine this solution is to progressively decrease the value of $\mathrm{a}_{\max }$ (e.g. by assuming $\mathrm{a}_{\max }=\gamma \mathrm{a}_{\max }$, with $0<\gamma<1$ ) and compute the durations of the segments by means of (3.26a), (3.26b), (3.26c), until the conditions $T_{a}>2 T_{j}$ and $T_{d}>2 T_{j}$ are both true.

Example 3.11 Fig. 3.16 reports the position, velocity, acceleration and jerk of a double S trajectory when the constant velocity segment is not present. In this case, also the maximum acceleration is not reached and the above recursive algorithm is adopted. The boundary conditions are

$$
q_{0}=0, \quad q_{1}=10, \quad \mathrm{v}_{0}=7, \quad \mathrm{v}_{1}=0
$$

while the constraints are

$$
\mathrm{v}_{\max }=10, \quad \mathrm{a}_{\max }=10, \quad \mathrm{j}_{\max }=30
$$

The time intervals defining the double S trajectory result

$$
T_{a}=0.4666, \quad T_{v}=0, \quad T_{d}=1.4718, \quad T_{j 1}=0.2321, \quad T_{j 2}=0.2321
$$

The maximum speed is $\mathrm{v}_{\text {lim }}=8.6329$, and the limit values of the acceleration during the acceleration and deceleration period are respectively

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-06.jpg?height=50&width=562&top_left_y=1825&top_left_x=180)

During this recursive computation, it may happen that $T_{a}$ or $T_{d}$ becomes negative. In this case, only one of the acceleration or deceleration phase is necessary, depending on the values of the initial and final velocities. If $T_{a}<0$

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-07.jpg?height=1181&width=824&top_left_y=173&top_left_x=356)

Fig. 3.15. Double S trajectory profiles (position, velocity, acceleration and jerk) without a constant velocity phase.

(note that in this case necessarily $\mathrm{v}_{0}>\mathrm{v}_{1}$ ), the acceleration phase is not present. Then, $T_{a}$ is set to 0 and the duration of the deceleration segment can be computed according to

$$
\begin{equation*}
T_{j 2}=\frac{\mathrm{j}_{\max }=2 \frac{q_{1}-q_{0}}{\mathrm{v}_{1}+\mathrm{v}_{0}}}{\left.\mathrm{j}_{\max }-q_{0}\right)-\sqrt{\left.\mathrm{j}_{\max }+\mathrm{v}_{0}\right)}} \tag{3.28a}
\end{equation*}
$$

Example 3.12 Fig. 3.17 reports position, velocity, acceleration and jerk of a double S trajectory composed only by the deceleration phase. The boundary conditions are

$$
q_{0}=0, \quad q_{1}=10, \quad \mathrm{v}_{0}=7.5, \quad \mathrm{v}_{1}=0
$$

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-08.jpg?height=1189&width=824&top_left_y=169&top_left_x=354)

Fig. 3.16. Double S trajectory profiles (position, velocity, acceleration and jerk) without the constant velocity phase and with limit acceleration lower than the maximum value.

with constraints

$$
\mathrm{v}_{\max }=10, \quad \mathrm{a}_{\max }=10, \quad \mathrm{j}_{\max }=30
$$

The resulting time intervals are

$$
T_{a}=0, \quad T_{v}=0, \quad T_{d}=2.6667, \quad T_{j 1}=0, \quad T_{j 2}=0.0973
$$

The maximum velocity is $\mathrm{v}_{\text {lim }}=7.5$, and the limit values of the acceleration during the acceleration and deceleration periods are respectively $\mathrm{a}_{\lim _{a}}=0$

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-08.jpg?height=43&width=330&top_left_y=1876&top_left_x=178)

In the dual case, i.e. when $T_{d}<0$ (this case is possible when $\mathrm{v}_{1}>\mathrm{v}_{0}$ ), the deceleration phase is not necessary $\left(T_{d}=0\right)$ and the duration of the

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-09.jpg?height=1179&width=822&top_left_y=174&top_left_x=355)

Fig. 3.17. Double S trajectory profiles (position, velocity, acceleration and jerk) composed by only a deceleration phase.

acceleration period must be computed as

$$
\begin{equation*}
T_{j 1}=\frac{\mathrm{j}_{\max }=2 \frac{q_{1}-q_{0}}{\mathrm{v}_{1}+\mathrm{v}_{0}}}{\mathrm{j}_{\max }\left(\mathrm{v}_{1}+\mathrm{v}_{0}\right)-\sqrt{\mathrm{j}_{\max }\left(\mathrm{j}_{\max }\left(q_{1}-q_{0}\right)^{2}-\left(\mathrm{v}_{1}+\mathrm{v}_{0}\right)^{2}\left(\mathrm{v}_{1}-\mathrm{v}_{0}\right)\right)}} \tag{3.29a}
\end{equation*}
$$

After the duration of each segment of the trajectory has been defined, it is possible to compute the values of the maximum/minimum accelerations ( $\mathrm{a}_{\text {lim }_{a}}$ and $\mathrm{a}_{\text {lim }}$ ) and of the maximum velocity ( $\mathrm{v}_{\text {lim }}$ ) of the trajectory:

$$
\begin{gathered}
\mathrm{a}_{\text {lim }_{a}}=\mathrm{j}_{\max } T_{j 1}, \quad \mathrm{a}_{\text {lim }_{d}}=-\mathrm{j}_{\text {max }} T_{j 2} \\
\mathrm{v}_{\text {lim }}=\mathrm{v}_{0}+\left(T_{a}-T_{j 1}\right) \mathrm{a}_{\text {lim }_{a}}=\mathrm{v}_{1}-\left(T_{d}-T_{j 2}\right) \mathrm{a}_{\text {lim }_{d}}
\end{gathered}
$$

### 3.4.1 Computation of the trajectory for $q_{1}>q_{0}$

Once the time lengths and the other parameters have been defined, the double S trajectory is computed by means of the following equations (one for each segment defined in Fig. 3.13). The case $t_{0}=0$ is assumed, otherwise a translation in time has to be applied, see Sec. 5.1.

#### Acceleration phase

a) $t \in\left[0, T_{j 1}\right]$

$$
\left\{\begin{array}{l}
q(t)=q_{0}+\mathrm{v}_{0} t+\mathrm{j}_{\max } \frac{t^{3}}{6}  \tag{3.30a}\\
\dot{q}(t)=\mathrm{v}_{0}+\mathrm{j}_{\max } \frac{t^{2}}{2} \\
\ddot{q}(t)=\mathrm{j}_{\max } t \\
q^{(3)}(t)=\mathrm{j}_{\max }
\end{array}\right.
$$

b) $t \in\left[T_{j 1}, T_{a}-T_{j 1}\right]$

$$
\left\{\begin{array}{l}
q(t)=q_{0}+\mathrm{v}_{0} t+\frac{\mathrm{a}_{l i m_{a}}}{6}\left(3 t^{2}-3 T_{j 1} t+T_{j 1}^{2}\right)  \tag{3.30b}\\
\dot{q}(t)=\mathrm{v}_{0}+\mathrm{a}_{l i m_{a}}\left(t-\frac{T_{j 1}}{2}\right) \\
\ddot{q}(t)=\mathrm{j}_{\max } T_{j 1}=\mathrm{a}_{l i m_{a}} \\
q^{(3)}(t)=0
\end{array}\right.
$$

c) $t \in\left[T_{a}-T_{j 1}, T_{a}\right]$

$$
\begin{cases}q(t) & =q_{0}+\left(\mathrm{v}_{l i m}+\mathrm{v}_{0}\right) \frac{T_{a}}{2}-\mathrm{v}_{l i m}\left(T_{a}-t\right)-\mathrm{j}_{\min } \frac{\left(T_{a}-t\right)^{3}}{6}  \tag{3.30c}\\ \dot{q}(t)=\mathrm{v}_{l i m}+\mathrm{j}_{\min } \frac{\left(T_{a}-t\right)^{2}}{2} \\ \ddot{q}(t)=-\mathrm{j}_{\min }\left(T_{a}-t\right) \\ q^{(3)}(t)=\mathrm{j}_{\min }=-\mathrm{j}_{\max }\end{cases}
$$

#### Constant velocity phase

a) $t \in\left[T_{a}, T_{a}+T_{v}\right]$

$$
\begin{cases}q(t) & =q_{0}+\left(\mathrm{v}_{l i m}+\mathrm{v}_{0}\right) \frac{T_{a}}{2}+\mathrm{v}_{l i m}\left(t-T_{a}\right)  \tag{3.30d}\\ \dot{q}(t) & =\mathrm{v}_{\text {lim }} \\ \ddot{q}(t) & =0 \\ q^{(3)}(t) & =0\end{cases}
$$

#### Deceleration phase

a) $t \in\left[T-T_{d}, T-T_{d}+T_{j 2}\right]$

$$
\left\{\begin{array}{l}
q(t)=q_{1}-\left(\mathrm{v}_{\text {lim }}+\mathrm{v}_{1}\right) \frac{T_{d}}{2}+\mathrm{v}_{l i m}\left(t-T+T_{d}\right)-\mathrm{j}_{\max } \frac{\left(t-T+T_{d}\right)^{3}}{6}  \tag{3.30e}\\
\dot{q}(t)=\mathrm{v}_{\text {lim }}-\mathrm{j}_{\max } \frac{\left(t-T+T_{d}\right)^{2}}{2} \\
\ddot{q}(t)=-\mathrm{j}_{\max }\left(t-T+T_{d}\right) \\
q^{(3)}(t)=\mathrm{j}_{\min }=-\mathrm{j}_{\max }
\end{array}\right.
$$

b) $t \in\left[T-T_{d}+T_{j 2}, T-T_{j 2}\right]$

$$
\left\{\begin{align*}
q(t)= & q_{1}-\left(\mathrm{v}_{l i m}+\mathrm{v}_{1}\right) \frac{T_{d}}{2}+\mathrm{v}_{l i m}\left(t-T+T_{d}\right)+  \tag{3.30f}\\
& +\frac{\mathrm{a}_{l i m_{d}}}{6}\left(3\left(t-T+T_{d}\right)^{2}-3 T_{j 2}\left(t-T+T_{d}\right)+T_{j 2}^{2}\right) \\
\dot{q}(t)= & \mathrm{v}_{l i m}+\mathrm{a}_{l_{i m_{d}}}\left(t-T+T_{d}-\frac{T_{j 2}}{2}\right) \\
\ddot{q}(t)= & -\mathrm{j}_{\max } T_{j 2}=\mathrm{a}_{l i m_{d}} \\
q^{(3)}(t)= & 0
\end{align*}\right.
$$

c) $t \in\left[T-T_{j 2}, T\right]$

$$
\begin{cases}q(t) & =q_{1}-\mathrm{v}_{1}(T-t)-\mathrm{j}_{\max } \frac{(T-t)^{3}}{6}  \tag{3.30~g}\\ \dot{q}(t) & =\mathrm{v}_{1}+\mathrm{j}_{\max } \frac{(T-t)^{2}}{2} \\ \ddot{q}(t) & =-\mathrm{j}_{\max }(T-t) \\ q^{(3)}(t)=\mathrm{j}_{\max }\end{cases}
$$

### 3.4.2 Computation of the trajectory for q1 < q0

In case $q_{1}<q_{0}$, the parameters of the trajectory can be computed according to the same procedure reported above. It is necessary to consider the initial and final positions/velocities with opposite signs, and, after the computation, to invert the resulting profiles of position, velocity, acceleration, and jerk.

More generally, given any initial and final values for position and velocity $\left(\hat{q}_{0}, \hat{q}_{1}, \hat{\mathrm{v}}_{0}, \hat{\mathrm{v}}_{1}\right)$, in order to compute the trajectory it is necessary to transform these values as

$$
\begin{equation*}
q_{0}=\sigma \hat{q}_{0}, \quad q_{1}=\sigma \hat{q}_{1}, \quad \mathrm{v}_{0}=\sigma \hat{\mathrm{v}}_{0}, \quad \mathrm{v}_{1}=\sigma \hat{\mathrm{v}}_{1} \tag{3.31}
\end{equation*}
$$

where $\sigma=\operatorname{sign}\left(\hat{q}_{0}-\hat{q}_{1}\right)$. Similarly, also the constraints on maximum and minimum velocity, acceleration and jerk $\left(\hat{\mathrm{v}}_{\text {max }}, \hat{\mathrm{v}}_{\text {min }}, \hat{\mathrm{a}}_{\text {max }}, \hat{\mathrm{a}}_{\text {min }}, \hat{\mathrm{j}}_{\text {max }}, \hat{\mathrm{j}}_{\text {min }}\right)$ must be transformed:

$$
\left\{\begin{array}{l}
\mathrm{v}_{\max }=\frac{(\sigma+1)}{2} \hat{\mathrm{v}}_{\text {max }}+\frac{(\sigma-1)}{2} \hat{\mathrm{v}}_{\text {min }}  \tag{3.32}\\
\mathrm{v}_{\min }=\frac{(\sigma+1)}{2} \hat{\mathrm{v}}_{\text {min }}+\frac{(\sigma-1)}{2} \hat{\mathrm{v}}_{\text {max }} \\
\mathrm{a}_{\max }=\frac{(\sigma+1)}{2} \hat{\mathrm{a}}_{\text {max }}+\frac{(\sigma-1)}{2} \hat{\mathrm{a}}_{\text {min }} \\
\mathrm{a}_{\min }=\frac{(\sigma+1)}{2} \hat{\mathrm{a}}_{\text {min }}+\frac{(\sigma-1)}{2} \hat{\mathrm{a}}_{\text {max }} \\
\mathrm{j}_{\max }=\frac{(\sigma+1)}{2} \hat{\mathrm{j}}_{\text {max }}+\frac{(\sigma-1)}{2} \hat{\mathrm{j}}_{\text {min }} \\
\mathrm{j}_{\min }=\frac{(\sigma+1)}{2} \hat{\mathrm{j}}_{\text {min }}+\frac{(\sigma-1)}{2} \hat{\mathrm{j}}_{\text {max }}
\end{array}\right.
$$

Finally, the computed profiles (i.e. $\left.q(t), \dot{q}(t), \ddot{q}(t), q^{(3)}(t)\right)$ must be transformed again as

$$
\left\{\begin{array}{l}
\hat{q}(t)=\sigma q(t)  \tag{3.33}\\
\dot{\dot{q}}(t)=\sigma \dot{q}(t) \\
\ddot{\hat{q}}(t)=\sigma \ddot{q}(t) \\
\hat{q}^{(3)}(t)=\sigma q^{(3)}(t)
\end{array}\right.
$$

#### Flux diagram for the double S computation

Since the synthesis of the double S trajectory is quite articulated, a scheme summarizing the algorithm to determine the trajectory in all possible conditions is shown in Fig. 3.18.

### 3.4.3 Double S with null initial and final velocities

When the initial and final velocities $\mathrm{v}_{0}$ and $\mathrm{v}_{1}$ are null, the computation of the double S trajectory is much simpler, in particular when the constraints are symmetric $\left(\mathrm{j}_{\min }=-\mathrm{j}_{\text {max }}, \mathrm{a}_{\text {min }}=-\mathrm{a}_{\text {max }}, \mathrm{v}_{\text {min }}=-\mathrm{v}_{\text {max }}\right)$.

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-13.jpg?height=1425&width=944&top_left_y=173&top_left_x=289)

Fig. 3.18. Flux diagram for the double S trajectory computation.

As a matter of fact, in this case the acceleration and deceleration segments are symmetric, and then $T_{a}=T_{d}$ and $T_{j 1}=T_{j 2}=T_{j}$. Moreover, it is always possible to find a trajectory joining the initial and the final position, which meets the constraints on velocity, acceleration and jerk.

Let us assume $q_{1}>q_{0}$ (otherwise consider Sec. 3.4.2). Four situations are possible:

1. $\mathrm{v}_{\text {lim }}=\mathrm{v}_{\text {max }}$ :

1.a. $\mathrm{a}_{\text {lim }}=\mathrm{a}_{\max }$

1.b. $\mathrm{a}_{\text {lim }}<\mathrm{a}_{\max }$

2. $\mathrm{v}_{\text {lim }}<\mathrm{v}_{\text {max }}$ :

2.a. $\mathrm{a}_{\text {lim }}=\mathrm{a}_{\max }$

2.b. $\mathrm{a}_{\text {lim }}<\mathrm{a}_{\max }$

where $\mathrm{v}_{\text {lim }}$ and $\mathrm{a}_{\text {lim }}$ are the maximum values of velocity and acceleration actually reached during the trajectory, i.e. $\mathrm{v}_{\text {lim }}=\max _{t}(\dot{q}(t))$ and $\mathrm{a}_{\text {lim }}=$ $\max _{t}(\ddot{q}(t))$.

Case 1. $\mathrm{v}_{\text {lim }}=\mathrm{v}_{\text {max }}$.

In this case, it is necessary to check if the maximum acceleration $\mathrm{a}_{\max }$ is reached or not, and then compute $T_{j}$ and $T_{a}\left(=T_{d}\right)$

$$
\text { a. if } \begin{aligned}
\mathrm{v}_{\max } \mathrm{j}_{\max } \geq \mathrm{a}_{\max }^{2} \Rightarrow \quad T_{j} & =\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }} \\
T_{a} & =T_{j}+\frac{\mathrm{v}_{\max }}{\mathrm{a}_{\max }}
\end{aligned}
$$

$$
\begin{aligned}
& \text { b. if } \mathrm{v}_{\max } \mathrm{j}_{\max }<\mathrm{a}_{\max }^{2} \quad \Rightarrow \quad T_{j}=\sqrt{\frac{\mathrm{v}_{\max }}{\mathrm{j}_{\max }}} \\
& T_{a}=2 T_{j}
\end{aligned}
$$

Then, the duration of the constant velocity segment can be computed as

$$
T_{v}=\frac{q_{1}-q_{0}}{\mathrm{v}_{\max }}-T_{a}
$$

If $T_{v}$ is positive, the maximum velocity is reached, otherwise it is necessary to consider Case 2 (and $T_{v}=0$ ).

Case 2. $\mathrm{v}_{\text {lim }}<\mathrm{v}_{\text {max }}$.

Again, two sub-cases are possible, depending wether the maximum acceleration $\mathrm{a}_{\max }$ is reached or not. Also in this case the solution can be found in a closed form, as
a. if $\begin{aligned}\left(q_{1}-q_{0}\right) \geq 2 \frac{\mathrm{a}_{\max }^{3}}{\mathrm{j}_{\max }^{2}} \Rightarrow T_{j} & =\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }} \\ T_{a} & =\frac{T_{j}}{2}+\sqrt{\left(\frac{T_{j}}{2}\right)^{2}+\frac{q_{1}-q_{0}}{\mathrm{a}_{\max }}} .\end{aligned}$

b. if $\left(q_{1}-q_{0}\right)<2 \frac{\mathrm{a}_{\text {max }}^{3}}{\mathrm{j}_{\text {max }}^{2}} \Rightarrow T_{j}=\sqrt[3]{\frac{q_{1}-q_{0}}{2 \mathrm{j}_{\max }}}$ $T_{a}=2 T_{j}$.

Once $T_{j}, T_{a}$ (and $T_{d}$ ), $T_{v}$ are available, the trajectory can be evaluated according to (3.30a)-(3.30g), with

$$
\begin{aligned}
& \mathrm{a}_{l i m}=\mathrm{j}_{\max } T_{j}=\mathrm{a}_{l i m_{a}}=-\mathrm{a}_{l i m_{d}} \\
& \mathrm{v}_{l i m}=\left(T_{a}-T_{j}\right) \mathrm{a}_{l i m}
\end{aligned}
$$

Example 3.13 Fig. 3.19 shows the position, velocity, acceleration and jerk for a double S trajectory with zero initial and final velocities when the constant velocity phase is not present. In this case, also the maximum acceleration is not reached, however the trajectory parameters are computed in closed form. The boundary conditions are

$$
q_{0}=0, \quad q_{1}=10, \quad \mathrm{v}_{0}=0, \quad \mathrm{v}_{1}=0
$$

while the constraints are

$$
\mathrm{v}_{\max }=10, \quad \mathrm{a}_{\max }=20, \quad \mathrm{j}_{\max }=30
$$

The resulting time intervals are

$$
T_{j}=0.5503, \quad T_{a}=1.1006, \quad T_{v}=0
$$

The maximum velocity is $\mathrm{v}_{l i m}=8.6329$, and the limit values of the acceleration during the acceleration and deceleration period are respectively

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-15.jpg?height=50&width=562&top_left_y=1572&top_left_x=180)

### 3.4.4 On-line computation of the double S trajectory

A simplified approach for the computation of the double S profile is based on a discrete time formulation of the trajectory. This method is suitable when it is necessary to define complex trajectories, composed by several double S segments, and is appropriate for CNC machines, where the trajectory profiles are computed in discrete time.

Let us define

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-16.jpg?height=1189&width=836&top_left_y=169&top_left_x=343)

Fig. 3.19. Double S trajectory profiles with zero initial and final velocities, without a constant velocity phase and with limit acceleration lower than the maximum value.

$$
\begin{cases}q\left(t=k T_{s}\right) & =q_{k} \\ \dot{q}\left(t=k T_{s}\right) & =\dot{q}_{k} \\ \ddot{q}\left(t=k T_{s}\right) & =\ddot{q}_{k} \\ q^{(3)}\left(t=k T_{s}\right) & =q_{k}^{(3)}\end{cases}
$$

the values of position, velocity, acceleration and jerk at the $k$-th time instant, being $T_{s}$ the sampling period. The structure of the trajectory planner is shown in Fig. 3.20. Given the initial and final values of position, velocity and acceleration and the constraints ${ }^{2}\left(\mathrm{v}_{\max }, \mathrm{v}_{\min }, \mathrm{a}_{\max }, \mathrm{a}_{\min }, \mathrm{j}_{\text {max }}, \mathrm{j}_{\text {min }}\right)$, the jerk profile is computed as detailed below and then it is integrated three[^0]

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-17.jpg?height=298&width=1179&top_left_y=163&top_left_x=174)

Fig. 3.20. Block diagram of the trajectory planner for online computation of the double S trajectory.

times to obtain acceleration, velocity and position, respectively. In particular, the trapezoidal integration ${ }^{3}$ is adopted, and accordingly the relations between jerk, acceleration, velocity and position are

$$
\begin{align*}
\ddot{q}_{k} & =\ddot{q}_{k-1}+\frac{T_{s}}{2}\left(q_{k-1}^{(3)}+q_{k}^{(3)}\right) \\
\dot{q}_{k} & =\dot{q}_{k-1}+\frac{T_{s}}{2}\left(\ddot{q}_{k-1}+\ddot{q}_{k}\right)  \tag{3.34}\\
q_{k} & =q_{k-1}+\frac{T_{s}}{2}\left(\dot{q}_{k-1}+\dot{q}_{k}\right)
\end{align*}
$$

The basic idea of this trajectory planner is to perform, with acceleration and jerk compliant with the desired constraints, the acceleration phase and then the constant velocity segment until it is necessary to decelerate in order to reach the final position $q_{1}$ with the desired values of velocity and acceleration $\mathrm{v}_{1}$ and $\mathrm{a}_{1}$. Therefore, the computation of the trajectory is composed by two phases, see Fig. 3.21:

1. An acceleration profile is computed with the classical trapezoidal acceleration, possibly followed by a constant velocity phase $\left(=\mathrm{v}_{\text {max }}\right)$.
2. During the motion, at each time instant $k T_{s}$ it is checked whether it is possible to decelerate from the current velocity $\dot{q}_{k}$ to the final one $\mathrm{v}_{1}$, with the constraints on $\ddot{q}_{k}$ and $q_{k}^{(3)}$, and with the goal to reach exactly $q_{1}$.

#### Phase 1: Acceleration and constant velocity phase

In order to perform a double S trajectory from $q_{0}$ to $q_{1}\left(>q_{0}\right)$, the jerk is kept at its maximum value until $\ddot{q}_{k}<\mathrm{a}_{\max }$. Then, the jerk is set to zero $\left(q_{k}^{(3)}=0\right)$, and therefore the acceleration is constant $\left(\mathrm{a}_{\max }\right.$ ). Finally, the jerk is set to the minimum value $\left(q_{k}^{(3)}=\mathrm{j}_{\min }\right)$ in order to have a null acceleration when the maximum velocity is reached. At this point, the maximum velocity is maintained until the deceleration phase starts.

Mathematically, this can be described by

${ }^{3}$ The discrete time transfer function of an integrator is $G_{I}\left(z^{-1}\right)=\frac{T_{s}}{2} \frac{1+z^{-1}}{1-z^{-1}}$.

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-18.jpg?height=815&width=838&top_left_y=171&top_left_x=340)

Fig. 3.21. Online computation of the double S trajectory.

$$
q_{k}^{(3)}=\left\{\begin{array}{lll}
\mathrm{j}_{\max }, & \text { if } \quad \dot{q}_{k}-\frac{\ddot{q}_{k}^{2}}{2 \mathrm{j}_{\min }}<\mathrm{v}_{\max } & \text { and } \quad \ddot{q}_{k}<\mathrm{a}_{\max }  \tag{3.35}\\
0, & \text { if } \quad \dot{q}_{k}-\frac{\ddot{q}_{k}^{2}}{2 \mathrm{j}_{\min }}<\mathrm{v}_{\max } & \text { and } \quad \ddot{q}_{k} \geq \mathrm{a}_{\max } \\
\mathrm{j}_{\min }, & \text { if } \quad \dot{q}_{k}-\frac{\ddot{q}_{k}^{2}}{2 \mathrm{j}_{\min }} \geq \mathrm{v}_{\max } & \text { and } \quad \ddot{q}_{k}>0 \\
0, & \text { if } \quad \dot{q}_{k}-\frac{\ddot{q}_{k}^{2}}{2 \mathrm{j}_{\min }} \geq \mathrm{v}_{\max } & \text { and } \quad \ddot{q}_{k} \leq 0
\end{array}\right.
$$

The conditions $\dot{q}_{k}-\frac{\ddot{q}_{k}^{2}}{2 \mathrm{j}_{\min }} \geq \mathrm{v}_{\max }, \ddot{q}_{k} \geq \mathrm{a}_{\max }$ and $\ddot{q}_{k} \leq 0$ are considered in lieu of $\dot{q}_{k}-\frac{\ddot{q}_{k}^{2}}{2 \mathrm{j}_{\min }}=\mathrm{v}_{\max }, \ddot{q}_{k}=\mathrm{a}_{\max }$ and $\ddot{q}_{k}=0$, since usually small numerical errors affect the computations.

#### Phase 2: Deceleration phase

At each sampling time ${ }^{4}$, one must compute the time intervals $T_{d}, T_{j 2 a}$ and $T_{j 2 b}$ (refer to Fig. 3.22), necessary to change the acceleration and the velocity from the current values ( $\dot{q}_{k}$ and $\ddot{q}_{k}$ ) to the final ones ( $\mathrm{v}_{1}$ and $\mathrm{a}_{1}$ ) according to a trapezoidal deceleration profile, subject to constraints on maximum/minimim acceleration and jerk. From the expressions of velocity and acceleration varia-

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-18.jpg?height=52&width=1165&top_left_y=1971&top_left_x=183)
equations are not valid.
}

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-19.jpg?height=1195&width=870&top_left_y=175&top_left_x=342)

Fig. 3.22. Typical profiles for position, velocity, acceleration and jerk for the double S trajectory, with initial and final velocities and accelerations $\neq 0$.

tions, in the case that the minimum acceleration $\mathrm{a}_{\text {min }}$ is reached (accordingly $T_{d} \geq T_{j 2 a}+T_{j 2 b}$ ), one obtains

$$
\left\{\begin{align*}
T_{j 2 a} & =\frac{\mathrm{a}_{\min }-\ddot{q}_{k}}{\mathrm{j}_{\min }}  \tag{3.36}\\
T_{j 2 b} & =\frac{\mathrm{a}_{0}-\mathrm{a}_{\min }}{\mathrm{j}_{\max }} \\
T_{d} & =\frac{\mathrm{v}_{1}-\dot{q}_{k}}{\mathrm{a}_{\min }}+T_{j 2 a} \frac{\mathrm{a}_{\min }-\ddot{q}_{k}}{2 \mathrm{a}_{\min }}+T_{j 2 b} \frac{\mathrm{a}_{\min }-\mathrm{a}_{1}}{2 \mathrm{a}_{\min }}
\end{align*}\right.
$$

Otherwise,

$$
\left\{\begin{align*}
T_{j 2 a} & =-\frac{\ddot{q}_{k}}{\mathrm{j}_{\min }}+\frac{\sqrt{\left(\mathrm{j}_{\max }-\mathrm{j}_{\min }\right)\left(\ddot{q}_{k}^{2} \mathrm{j}_{\max }-\mathrm{j}_{\min }\left(\mathrm{a}_{1}^{2}+2 \mathrm{j}_{\max }\left(\dot{q}_{k}-\mathrm{v}_{1}\right)\right)\right)}}{\mathrm{j}_{\min }\left(\mathrm{j}_{\min }-\mathrm{j}_{\max }\right)}  \tag{3.37}\\
T_{j 2 b} & =\frac{\mathrm{a}_{1}}{\mathrm{j}_{\max }}+\frac{\sqrt{\left(\mathrm{j}_{\max }-\mathrm{j}_{\min }\right)\left(\ddot{q}_{k}^{2} \mathrm{j}_{\max }-\mathrm{j}_{\min }\left(\mathrm{a}_{1}^{2}+2 \mathrm{j}_{\max }\left(\dot{q}_{k}-\mathrm{v}_{1}\right)\right)\right)}}{\mathrm{j}_{\max }\left(\mathrm{j}_{\max }-\mathrm{j}_{\min }\right)} \\
T_{d} & =T_{j 2 a}+T_{j 2 b} .
\end{align*}\right.
$$

Note that the periods with maximum and minimum jerk $\left(T_{j 2 b}\right.$ and $\left.T_{j 2 a}\right)$ may be different since the initial and final accelerations of the deceleration phase, $\ddot{q}_{k}$ and $\mathrm{a}_{1}$ respectively, are in general not equal (and in particular are not null).

At this point, it is necessary to compute the position displacement produced by the acceleration and velocity profiles obtained from (3.36) or (3.37)

$$
h_{k}=\frac{1}{2} \ddot{q}_{k} T_{d}^{2}+\frac{1}{6}\left(\mathrm{j}_{\min } T_{j 2 a}\left(3 T_{d}^{2}-3 T_{d} T_{j 2 a}+T_{j 2 a}^{2}\right)+\mathrm{j}_{\max } T_{j 2 b}^{3}\right)+T_{d} \dot{q}_{k}
$$

and check if $h_{k}<q_{1}-q_{k}$. If this condition holds, it is necessary to continue the trajectory computation according to Case 1 (iterating the calculation of the deceleration parameters with the new values of $q_{k}, \dot{q}_{k}$ and $\ddot{q}_{k}$ ), otherwise the deceleration phase must start and the jerk is computed as

$$
q_{k}^{(3)}=\left\{\begin{array}{lll}
\mathrm{j}_{\min }, & \text { if } & (k-\bar{k}) \in\left[0, \frac{T_{j 2 a}}{T_{s}}\right]  \tag{3.38}\\
0, & \text { if } & (k-\bar{k}) \in\left[\frac{T_{j 2 a}}{T_{s}}, \frac{T_{d}-T_{j 2 b}}{T_{s}}\right] \\
\mathrm{j}_{\text {max }}, & \text { if } & (k-\bar{k}) \in\left[\frac{T_{d}-T_{j 2 b}}{T_{s}}, \frac{T_{d}}{T_{s}}\right]
\end{array}\right.
$$

where $\bar{k}$ is the time instant in which Phase 2 starts.

Example 3.14 Fig. 3.23 reports the position, velocity, acceleration and jerk for a double S trajectory computed online. In this case, it is simple to consider non-null initial and final values of velocity and acceleration, and asymmetric constraints on the maximum values of velocity, acceleration and jerk. In particular, the boundary conditions are

$$
q_{0}=0, \quad q_{1}=10, \quad \mathrm{v}_{0}=1, \quad \mathrm{v}_{1}=0, \quad \mathrm{a}_{0}=1, \quad \mathrm{a}_{1}=0
$$

while the constraints are

$$
\begin{array}{lll}
\mathrm{v}_{\max }=5, & \mathrm{a}_{\max }=10, & \mathrm{j}_{\max }=30 \\
\mathrm{v}_{\min }=-5, & \mathrm{a}_{\min }=-8, & \mathrm{j}_{\min }=-40
\end{array}
$$

The sampling period is $T_{s}=0.001 \mathrm{~s}$.

As already mentioned, this algorithm is affected by numerical errors which depend on the sampling period: the larger $T_{s}$ is, the larger the errors are on

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-21.jpg?height=1184&width=827&top_left_y=174&top_left_x=352)

Fig. 3.23. Double S trajectory profiles (position, velocity, acceleration and jerk) computed online.

acceleration, velocity and position. However, if it is necessary for some reasons to have a large value for $T_{s}$, it is possible to assume, only for the computation of the trajectory, a sampling period $N$ times smaller, e.g. $T_{s}^{\star}=\frac{T_{s}}{N}$, and then under-sample the data points so obtained.

Example 3.15 Fig. 3.24 reports the position, velocity, acceleration and jerk for a double S trajectory computed online, with $T_{s}=0.02 \mathrm{~s}$. The same values of boundary and peak conditions of the previous example are considered. The dashed lines have been obtained with $T_{s}=0.02 s$, while the solid lines have been computed with $T_{s}^{\star}=T_{s} / 200=0.0001 s$, and then under-sampling the profiles by considering a point every two hundred samples. Note the errors that affect the dashed curves.

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-22.jpg?height=1193&width=827&top_left_y=167&top_left_x=352)

Fig. 3.24. Double S trajectory profiles (position, velocity, acceleration and jerk) computed online, with $T_{s}=0.0001 \mathrm{~s}$ (solid) and with $T_{s}=0.02 s$ (dashed).

If $q_{1}<q_{0}$ it is possible to adopt the method described in Sec. 3.4.2.

Example 3.16 Fig. 3.25 reports the position, velocity, acceleration and jerk for a double S trajectory computed online for $q_{1}<q_{0}$. In particular, the boundary conditions are

$$
q_{0}=0, \quad q_{1}=-10, \quad \mathrm{v}_{0}=1, \quad \mathrm{v}_{1}=0, \quad \mathrm{a}_{0}=1, \quad \mathrm{a}_{1}=0
$$

with the constraints

$$
\begin{aligned}
& \mathrm{v}_{\max }=5, \quad \mathrm{a}_{\max }=10, \quad \mathrm{j}_{\text {max }}=30 \\
& \mathrm{v}_{\min }=-5, \quad \mathrm{a}_{\min }=-8, \quad \mathrm{j}_{\text {min }}=-40
\end{aligned}
$$

The sampling period is $T_{s}=0.001 \mathrm{~s}$.

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-23.jpg?height=1182&width=827&top_left_y=177&top_left_x=352)

Fig. 3.25. Double S trajectory profiles (position, velocity, acceleration and jerk) computed online with $q_{1}<q_{0}$.

### 3.4.5 Displacement time of a double S trajectory

The computation of the displacement time of a double S trajectory is rather complex, because of the large number of different possible cases. For this reason, only some specific, but common, situations are considered here.

In particular, assuming $h=q_{1}-q_{0}>0$, symmetric constraints $\mathrm{v}_{\text {min }}=$ $-\mathrm{v}_{\text {max }}, \mathrm{a}_{\text {min }}=-\mathrm{a}_{\text {max }}, \mathrm{j}_{\text {min }}=-\mathrm{j}_{\text {max }}$, and that the maximum values of both acceleration and speed ( $\mathrm{a}_{\max }$ and $\mathrm{v}_{\max }$ ) are reached, the total duration of the trajectory can be easily obtained as

$$
\begin{equation*}
T=\frac{h}{\mathrm{v}_{\max }}+\frac{T_{a}}{2}\left(1-\frac{\mathrm{v}_{0}}{\mathrm{v}_{\text {max }}}\right)+\frac{T_{d}}{2}\left(1-\frac{\mathrm{v}_{1}}{\mathrm{v}_{\text {max }}}\right) \tag{3.39}
\end{equation*}
$$

with

$$
T_{a}=\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }}+\frac{\mathrm{v}_{\max }-\mathrm{v}_{0}}{\mathrm{a}_{\max }}, \quad T_{d}=\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }}+\frac{\mathrm{v}_{\max }-\mathrm{v}_{1}}{\mathrm{a}_{\max }}
$$

If both initial and final speeds are zero, (3.39) becomes

$$
\begin{equation*}
T=\frac{h}{\mathrm{v}_{\max }}+\frac{\mathrm{v}_{\max }}{\mathrm{a}_{\max }}+\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }} \tag{3.40}
\end{equation*}
$$

From eq. (3.40), it is straightforward to verify that the time length of the trajectory can be easily modified by properly scaling the values of $\mathrm{v}_{\text {max }}, \mathrm{a}_{\text {max }}$, $\mathrm{j}_{\text {max }}$. As a matter of fact, if the new constraints

$$
\mathrm{v}_{\max }^{\prime}=\lambda \mathrm{v}_{\max }, \quad \mathrm{a}_{\max }^{\prime}=\lambda^{2} \mathrm{a}_{\max }, \quad \mathrm{j}_{\max }^{\prime}=\lambda^{3} \mathrm{j}_{\max }
$$

are considered, the duration $T^{\prime}$ becomes

$$
T^{\prime}=\frac{T}{\lambda}
$$

and therefore it is possible to compute the value of $\lambda$ which leads to a desired duration $T^{\prime}=T_{D}$ :

$$
\lambda=\frac{T}{T_{D}}
$$

The same considerations are valid also if the initial and final velocities are not null, but in this case it is necessary to scale also $\mathrm{v}_{0}$ and $\mathrm{v}_{1}$ (see eq. (3.39)):

$$
\mathrm{v}_{0}^{\prime}=\lambda \mathrm{v}_{0}, \quad \mathrm{v}_{1}^{\prime}=\lambda \mathrm{v}_{1}
$$

For other considerations about the scaling in time of trajectories, see Chapter 5 .

Example 3.17 In Fig. 3.26 the position, velocity, acceleration and jerk of a double S trajectory with a total duration $T_{D}=5 \mathrm{~s}$ are shown. The boundary conditions and the constraints are the same of Example 3.9, which lead to the time length $T=2.71 \mathrm{~s}$. In order to modify the duration of the trajectory and to obtain the desired value $T_{D}$, the constraints and the initial and final velocities are scaled by $\lambda=0.542$.

### 3.4.6 Double S trajectory with assigned duration of the different phases

A general approach for planning a double $S$ trajectory, with a given time length $T$ and with specified durations of the acceleration and of the constant jerk phases, consists in defining the values of $\mathrm{v}_{\max }, \mathrm{a}_{\max }$, and $\mathrm{j}_{\text {max }}$ as a function

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-25.jpg?height=1164&width=817&top_left_y=177&top_left_x=355)

Fig. 3.26. Double S trajectory profiles (position, velocity, acceleration and jerk) properly scaled to impose a desired duration $\left(T_{D}=5 \mathrm{~s}\right)$.

of the desired $T, T_{a}, T_{d}, T_{j}$. In particular, the symmetric case with $\mathrm{v}_{\min }=$ $-\mathrm{v}_{\text {max }}, \mathrm{a}_{\text {min }}=-\mathrm{a}_{\max }, \mathrm{j}_{\text {min }}=-\mathrm{j}_{\text {max }}$ is considered, and the initial and final velocities $\mathrm{v}_{0}, \mathrm{v}_{1}$ are both assumed to be zero (therefore $T_{d}=T_{a}$ ). Moreover, it is supposed that both the maximum speed and the maximum acceleration are reached. Therefore, with reference to equations (3.30a)-(3.30g) which define the trajectory profiles, it results

$$
\mathrm{v}_{\text {lim }}=\mathrm{v}_{\max }, \quad \mathrm{a}_{\lim _{a}}=\mathrm{a}_{\lim _{d}}=\mathrm{a}_{\max }
$$

From the expressions of the total duration, of the time length of acceleration phase and of the constant jerk segment, i.e.

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-26.jpg?height=1222&width=840&top_left_y=175&top_left_x=337)

Fig. 3.27. Double S trajectory profiles with zero initial and final velocities, and with desired durations of the single tracts.

$$
\left\{\begin{align*}
T & =\frac{h}{\mathrm{v}_{\max }}+T_{a}  \tag{3.41}\\
T_{a} & =\frac{\mathrm{v}_{\max }}{\mathrm{a}_{\max }}+T_{j} \\
T_{j} & =\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }}
\end{align*}\right.
$$

it is possible to deduce the corresponding values of $\mathrm{v}_{\text {max }}, \mathrm{a}_{\text {max }}, \mathrm{j}_{\text {max }}$ :

$$
\left\{\begin{aligned}
\mathrm{v}_{\max } & =\frac{h}{T-T_{a}} \\
\mathrm{a}_{\max } & =\frac{h}{\left(T-T_{a}\right)\left(T_{a}-T_{j}\right)} \\
\mathrm{j}_{\max } & =\frac{h}{\left(T-T_{a}\right)\left(T_{a}-T_{j}\right) T_{j}}
\end{aligned}\right.
$$

If one assumes that the acceleration period is a fraction of the entire trajectory duration:

$$
T_{a}=\alpha T, \quad 0<\alpha \leq 1 / 2
$$

and, in a similar manner, that the time length of the constant jerk phase is a fraction of the acceleration period:

$$
T_{j}=\beta T_{a}, \quad 0<\beta \leq 1 / 2
$$

then the values of the maximum speed, acceleration and jerk of the double S trajectory $q(t)$ are obtained as:

$$
\left\{\begin{align*}
\mathrm{v}_{\max } & =\frac{h}{(1-\alpha) T}  \tag{3.42}\\
\mathrm{a}_{\max } & =\frac{h}{\alpha(1-\alpha)(1-\beta) T^{2}} \\
\mathrm{j}_{\max } & =\frac{h}{\alpha^{2} \beta(1-\alpha)(1-\beta) T^{3}}
\end{align*}\right.
$$

By substituting these values in (3.30a)-(3.30g), the trajectory with the given durations is defined.

Example 3.18 A double S trajectory with the boundary conditions

$$
q_{0}=0, \quad q_{1}=10, \quad \mathrm{v}_{0}=0, \quad \mathrm{v}_{1}=0
$$

is computed with the purpose of obtaining a total duration $T=5$. The values

$$
\alpha=1 / 3, \quad \beta=1 / 5
$$

are considered, or equivalently

$$
T_{a}=1.6666, \quad T_{j}=0.3333
$$

The resulting values of the velocity, acceleration and jerk are

$$
\mathrm{v}_{\max }=3.14, \quad \mathrm{a}_{\max }=2.25, \quad \mathrm{j}_{\max }=6.75
$$

They produce the trajectory reported in Fig. 3.28.

![](https://cdn.mathpix.com/cropped/2024_07_22_a9cd1b03b7fbda25e480g-28.jpg?height=1261&width=887&top_left_y=169&top_left_x=320)

Fig. 3.28. Double S trajectory profiles with prescribed duration T , and with the conditions $T_{a}=T / 3$ and $T_{j}=T_{a} / 5$.

Obviously, the equations (3.41) which relate the variables of the double S trajectory ( $\mathrm{v}_{\max }, \mathrm{a}_{\text {max }}, \mathrm{j}_{\text {max }}, h, T, T_{a}, T_{j}$ ) can also be solved with respect to other sets of variables, e.g. ( $\left.\mathrm{v}_{\max }, \mathrm{a}_{\max }, T_{j}\right)$, or $\left(\mathrm{v}_{\max }, T_{a}, T_{j}\right)$, and so on, if the other terms are known. For instance, if one desires a trajectory with a total duration $T$ and with a given maximum acceleration and jerk values $\mathrm{a}_{\max }, \mathrm{j}_{\max }$, it is possible to obtain the remaining coefficients as

$$
\left\{\begin{aligned}
\mathrm{v}_{\max } & =\frac{-\mathrm{a}_{\max }^{2}+\mathrm{a}_{\max } \mathrm{j}_{\max } T-\sqrt{\mathrm{a}_{\max }\left(-4 h \mathrm{j}_{\max }^{2}+\mathrm{a}_{\max }\left(\mathrm{a}_{\max }-\mathrm{j}_{\max } T\right)^{2}\right)}}{2 \mathrm{j}_{\max }} \\
T_{a} & =\frac{\mathrm{a}_{\max }^{2}+\mathrm{a}_{\max } \mathrm{j}_{\max } T-\sqrt{\mathrm{a}_{\max }\left(-4 \mathrm{j}_{\max }^{2}+\mathrm{a}_{\max }\left(\mathrm{a}_{\max }-\mathrm{j}_{\max } T\right)^{2}\right)}}{2 \mathrm{a}_{\max } \mathrm{j}_{\max }} \\
T_{j} & =\frac{\mathrm{a}_{\max }}{\mathrm{j}_{\max }}
\end{aligned}\right.
$$

Example 3.19 A double S trajectory with the boundary conditions

$$
q_{0}=0, \quad q_{1}=10, \quad \mathrm{v}_{0}=0, \quad \mathrm{v}_{1}=0
$$

is computed with the purpose of obtaining a total duration $T=5$. Moreover, the constraints

$$
\mathrm{a}_{\max }=2, \quad \mathrm{j}_{\max }=8
$$

are considered. The resulting values of the velocity, and of duration of the constant acceleration and jerk phases are

$$
\mathrm{v}_{\max }=3, \quad T_{a}=1.82, \quad T_{j}=0.25
$$

They define the trajectory reported in Fig. 3.29.

