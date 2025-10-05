# dr_vehicle: Dead Reckoning  
## 3D Quaternion EKF (Error-State) — Overview

### Frames
World/navigation frame $\mathcal{W}$ (ENU), body frame $\mathcal{B}$.

### Nominal State
- $\mathbf{r} \in \mathbb{R}^3$ (position, ENU)
- $\mathbf{v} \in \mathbb{R}^3$ (velocity, ENU)
- $\mathbf{q} \in \mathbb{H}$ (unit quaternion, world $\to$ body)
- $\mathbf{b}_g, \mathbf{b}_a \in \mathbb{R}^3$ (gyroscope/accelerometer biases)

Nominal state:  
$x_{\text{nom}} = [\mathbf{r},\ \mathbf{v},\ \mathbf{q},\ \mathbf{b}_g,\ \mathbf{b}_a]$

### IMU Measurements (body frame)
Angular rate $\boldsymbol{\omega}_m$, specific force $\mathbf{a}_m$.  
Bias-corrected: $\boldsymbol{\omega} = \boldsymbol{\omega}_m - \mathbf{b}_g$,  
$\mathbf{a} = \mathbf{a}_m - \mathbf{b}_a$.

### Error State (15-dim)
$\delta \mathbf{x} = [\delta\mathbf{r},\ \delta\mathbf{v},\ \delta\boldsymbol{\theta},\ \delta\mathbf{b}_g,\ \delta\mathbf{b}_a]$  
where $\delta\boldsymbol{\theta} \in \mathbb{R}^3$ is the small-angle attitude error.


## Continuous-Time Nominal Model

$$
\dot{\mathbf{r}} = \mathbf{v} \\
\dot{\mathbf{v}} = R(\mathbf{q})\,\mathbf{a} + \mathbf{g} \\
\dot{\mathbf{q}} = \tfrac{1}{2}\,\Omega(\boldsymbol{\omega})\,\mathbf{q},\quad |\mathbf{q}| = 1 \\
\dot{\mathbf{b}}_{g} = \mathbf{n}_{wg} \\
\dot{\mathbf{b}}_{a} = \mathbf{n}_{wa}
$$


## Discrete Nominal Propagation (IMU period $\Delta t$)

### Quaternion Update

$$
\delta\boldsymbol{\theta} = \boldsymbol{\omega}\,\Delta t \\
\delta\mathbf{q} =
\begin{bmatrix}
\cos\left(\dfrac{|\delta\boldsymbol{\theta}|}{2}\right) \\
\hat{\delta\boldsymbol{\theta}} \sin\left(\dfrac{|\delta\boldsymbol{\theta}|}{2}\right)
\end{bmatrix} \\
\mathbf{q}_{k+1} = \text{normalize}\left( \delta\mathbf{q} \otimes \mathbf{q}_k \right)
$$

---

### Position and Velocity Update

$$
\mathbf{f}_w = R(\mathbf{q}_k)\,\mathbf{a} \\
\mathbf{v}_{k+1} = \mathbf{v}_k + (\mathbf{f}_w + \mathbf{g})\,\Delta t \\
\mathbf{r}_{k+1} = \mathbf{r}_k + \mathbf{v}_k\,\Delta t + \tfrac{1}{2}(\mathbf{f}_w + \mathbf{g})\,\Delta t^2 \\
\mathbf{b}_{g,k+1} = \mathbf{b}_{g,k} \\
\mathbf{b}_{a,k+1} = \mathbf{b}_{a,k}
$$

---

### Continuous-Time Error-State Model

Using the skew-symmetric operator $S(\cdot)$ such that $S(\mathbf{x})\,\mathbf{y} = \mathbf{x} \times \mathbf{y}$:

$$
\delta\dot{\mathbf{r}} = \delta\mathbf{v} \\
\delta\dot{\mathbf{v}} = -\,R(\mathbf{q})\,S(\mathbf{a})\,\delta\boldsymbol{\theta} - R(\mathbf{q})\,\delta\mathbf{b}_a + \mathbf{n}_a \\
\delta\dot{\boldsymbol{\theta}} = -\,S(\boldsymbol{\omega})\,\delta\boldsymbol{\theta} - \delta\mathbf{b}_g + \mathbf{n}_g \\
\delta\dot{\mathbf{b}}_g = \mathbf{n}_{wg} \\
\delta\dot{\mathbf{b}}_a = \mathbf{n}_{wa}
$$

Compactly:

$$
\delta\dot{\mathbf{x}} = F_c\,\delta\mathbf{x} + G_c\,\mathbf{n}
$$

---

### Discrete Covariance Propagation

First-order discretization:

$$
\Phi_k \approx I + F_c\,\Delta t \\
Q_k \approx G_c\,Q_c\,G_c^\top\,\Delta t \\
P_{k+1}^- = \Phi_k\,P_k^+\,\Phi_k^\top + Q_k
$$

---

### Measurement Models (Examples) and Jacobians

- **GPS Position:** $h(\cdot) = \mathbf{r}$  
  $H = [\,I \quad 0 \quad 0 \quad 0 \quad 0\,]$

- **GPS Velocity:** $h(\cdot) = \mathbf{v}$  
  $H = [\,0 \quad I \quad 0 \quad 0 \quad 0\,]$

- **Barometer:** $h(\cdot) = r_z$ (pick the $z$-row from the position block)

- **Magnetometer:** $h(\cdot) = R(\mathbf{q})^\top \mathbf{m}_w$ (linearize w.r.t. $\delta\boldsymbol{\theta}$)

---

### Measurement Update (Error-State)

$$
S_k = H_k\,P_k^-\,H_k^\top + R_k \\
K_k = P_k^-\,H_k^\top\,S_k^{-1} \\
\delta\hat{\mathbf{x}}_k = K_k \left[\,\mathbf{z}_k - h(\mathbf{x}_{\text{nom},k}^-)\,\right] \\
P_k^+ = (I - K_k\,H_k)\,P_k^-
$$

---

### Injecting the Error-State into the Nominal

$$
\mathbf{r} \leftarrow \mathbf{r} + \delta\mathbf{r} \\
\mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v} \\
\mathbf{q} \leftarrow \text{normalize}\left( \delta\mathbf{q} \otimes \mathbf{q} \right), \quad
\delta\mathbf{q} \approx 
\begin{bmatrix}
1 \\[6pt] \tfrac{1}{2}\,\delta\boldsymbol{\theta}
\end{bmatrix} \\
\mathbf{b}_g \leftarrow \mathbf{b}_g + \delta\mathbf{b}_g \\
\mathbf{b}_a \leftarrow \mathbf{b}_a + \delta\mathbf{b}_a
$$
