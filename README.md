# dr_vehicle
Dead Reackoning... 

 ## 3D Quaternion EKF (Error-State) — Overview

**Frames.**
World/navigation frame $\mathcal{W}$ (ENU), body frame $\mathcal{B}$.

**Nominal state.**
$\mathbf{r}\in\mathbb{R}^3$ (pos, ENU), $\mathbf{v}\in\mathbb{R}^3$ (vel, ENU),
$\mathbf{q}\in\mathbb{H}$ (unit quaternion, world $\to$ body),
$\mathbf{b}_g,\ \mathbf{b}_a\in\mathbb{R}^3$ (gyro/accel biases).
$$
x_{\text{nom}}=[\mathbf{r},\mathbf{v},\mathbf{q},\mathbf{b}_g,\mathbf{b}_a]
$$

**IMU measurements (body frame).**
Angular rate $\boldsymbol{\omega}_m$, specific force $\mathbf{a}_m$.
Bias-corrected: $\boldsymbol{\omega}=\boldsymbol{\omega}_m-\mathbf{b}_g$,
$\mathbf{a}=\mathbf{a}_m-\mathbf{b}_a$.

**Error state (15-dim).**
$\delta \mathbf{x}=[\delta\mathbf{r},\delta\mathbf{v},\delta\boldsymbol{\theta},\delta\mathbf{b}_g,\delta\mathbf{b}_a]$,
where $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ is the small-angle attitude error.

### 1) Continuous-time nominal model
\[
\dot{\mathbf{r}}=\mathbf{v},\qquad
\dot{\mathbf{v}}=R(\mathbf{q})\,\mathbf{a}+\mathbf{g},\qquad
\dot{\mathbf{q}}=\tfrac12\,\Omega(\boldsymbol{\omega})\,\mathbf{q},\ \ \|\mathbf{q}\|=1,
\]
\[
\dot{\mathbf{b}}_g=\mathbf{n}_{wg},\qquad
\dot{\mathbf{b}}_a=\mathbf{n}_{wa}.
\]

### 2) Discrete nominal propagation (IMU period $\Delta t$)
\[
\delta\boldsymbol{\theta}=\boldsymbol{\omega}\,\Delta t,\quad
\delta\mathbf{q}=\begin{bmatrix}\cos(\frac{\|\delta\boldsymbol{\theta}\|}{2})\\ \hat{\delta\boldsymbol{\theta}}\sin(\frac{\|\delta\boldsymbol{\theta}\|}{2})\end{bmatrix},\quad
\mathbf{q}_{k+1}=\text{normalize}(\delta\mathbf{q}\otimes\mathbf{q}_k).
\]
\[
\mathbf{f}_w=R(\mathbf{q}_k)\,\mathbf{a},\quad
\mathbf{v}_{k+1}=\mathbf{v}_k+(\mathbf{f}_w+\mathbf{g})\,\Delta t,\quad
\mathbf{r}_{k+1}=\mathbf{r}_k+\mathbf{v}_k\,\Delta t+\tfrac12(\mathbf{f}_w+\mathbf{g})\,\Delta t^2.
\]
\[
\mathbf{b}_{g,k+1}=\mathbf{b}_{g,k},\qquad \mathbf{b}_{a,k+1}=\mathbf{b}_{a,k}.
\]

### 3) Continuous-time error-state model
Using $S(\cdot)$ as the skew operator ($S(\mathbf{x})\,\mathbf{y}=\mathbf{x}\times\mathbf{y}$),
\[
\begin{aligned}
\delta\dot{\mathbf{r}} &= \delta\mathbf{v},\\
\delta\dot{\mathbf{v}} &= -\,R(\mathbf{q})\,S(\mathbf{a})\,\delta\boldsymbol{\theta}\;-\;R(\mathbf{q})\,\delta\mathbf{b}_a\;+\;\mathbf{n}_{a},\\
\delta\dot{\boldsymbol{\theta}} &= -\,S(\boldsymbol{\omega})\,\delta\boldsymbol{\theta}\;-\;\delta\mathbf{b}_g\;+\;\mathbf{n}_{g},\\
\delta\dot{\mathbf{b}}_g &= \mathbf{n}_{wg},\\
\delta\dot{\mathbf{b}}_a &= \mathbf{n}_{wa}.
\end{aligned}
\]
Compactly, $\dot{\delta\mathbf{x}} = F_c\,\delta\mathbf{x} + G_c\,\mathbf{n}$.

### 4) Discrete covariance propagation
First-order discretization:
\[
\Phi_k \approx I + F_c\,\Delta t,\qquad
Q_k \approx G_c\,Q_c\,G_c^\top\,\Delta t,
\]
\[
P_{k+1}^- = \Phi_k\,P_k^+\,\Phi_k^\top + Q_k.
\]

### 5) Measurements (examples) and Jacobians
- GPS position: $h(\cdot)=\mathbf{r}$, hence $H=[I\ 0\ 0\ 0\ 0]$.
- GPS velocity: $h(\cdot)=\mathbf{v}$, hence $H=[0\ I\ 0\ 0\ 0]$.
- Barometer: $h(\cdot)=r_z$, pick row from position block.
- Magnetometer: $h(\cdot)=R(\mathbf{q})^\top \mathbf{m}_w$, linearize wrt $\delta\boldsymbol{\theta}$.

### 6) Measurement update (error-state)
\[
S_k = H_k P_k^- H_k^\top + R_k,\qquad
K_k = P_k^- H_k^\top S_k^{-1},\qquad
\delta\hat{\mathbf{x}}_k = K_k\big(\mathbf{z}_k - h(x_{\text{nom},k}^-)\big),
\]
\[
P_k^+ = (I - K_k H_k) P_k^-.
\]
**Inject** into the nominal:
\[
\mathbf{r}\leftarrow \mathbf{r}+\delta\mathbf{r},\quad
\mathbf{v}\leftarrow \mathbf{v}+\delta\mathbf{v},\quad
\mathbf{q}\leftarrow \text{normalize}\!\left(\delta\mathbf{q}\otimes\mathbf{q}\right),\ \ \delta\mathbf{q}\approx \begin{bmatrix}1\\ \tfrac12\,\delta\boldsymbol{\theta}\end{bmatrix},
\]
\[
\mathbf{b}_g\leftarrow \mathbf{b}_g+\delta\mathbf{b}_g,\qquad
\mathbf{b}_a\leftarrow \mathbf{b}_a+\delta\mathbf{b}_a,
\]
then reset $\delta\mathbf{x}\!=\!\mathbf{0}$.

