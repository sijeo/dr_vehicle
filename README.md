\documentclass{article}
\usepackage{amsmath, amssymb}
\usepackage{bm}
\usepackage{hyperref}

\begin{document}

\title{dr\_vehicle: Dead Reckoning \\ 3D Quaternion EKF (Error-State) --- Overview}
\author{}
\date{}
\maketitle

\section*{Frames}
World/navigation frame $\mathcal{W}$ (ENU), body frame $\mathcal{B}$.

\section*{Nominal State}
\begin{itemize}
  \item $\mathbf{r} \in \mathbb{R}^3$ (position, ENU)
  \item $\mathbf{v} \in \mathbb{R}^3$ (velocity, ENU)
  \item $\mathbf{q} \in \mathbb{H}$ (unit quaternion, world $\to$ body)
  \item $\mathbf{b}_g, \mathbf{b}_a \in \mathbb{R}^3$ (gyroscope/accelerometer biases)
\end{itemize}

Nominal state: 
\[
x_{\text{nom}} = [\mathbf{r},\ \mathbf{v},\ \mathbf{q},\ \mathbf{b}_g,\ \mathbf{b}_a]
\]

\section*{IMU Measurements (body frame)}
Angular rate $\boldsymbol{\omega}_m$, specific force $\mathbf{a}_m$.\\
Bias-corrected: $\boldsymbol{\omega} = \boldsymbol{\omega}_m - \mathbf{b}_g$,\\
$\mathbf{a} = \mathbf{a}_m - \mathbf{b}_a$.

\section*{Error State (15-dim)}
\[
\delta \mathbf{x} = [\delta\mathbf{r},\ \delta\mathbf{v},\ \delta\boldsymbol{\theta},\ \delta\mathbf{b}_g,\ \delta\mathbf{b}_a]
\]
where $\delta\boldsymbol{\theta} \in \mathbb{R}^3$ is the small-angle attitude error.

\section{Continuous-Time Nominal Model}
\begin{align*}
\dot{\mathbf{r}} &= \mathbf{v} \\
\dot{\mathbf{v}} &= R(\mathbf{q})\,\mathbf{a} + \mathbf{g} \\
\dot{\mathbf{q}} &= \frac{1}{2}\,\Omega(\boldsymbol{\omega})\,\mathbf{q},\quad \|\mathbf{q}\| = 1 \\
\dot{\mathbf{b}_{g}} &= \mathbf{n}_{wg} \\
\dot{\mathbf{b}_{a}} &= \mathbf{n}_{wa}
\end{align*}

\section{Discrete Nominal Propagation (IMU period $\Delta t$)}
\begin{align*}
\delta\boldsymbol{\theta} &= \boldsymbol{\omega}\,\Delta t, \\
\delta\mathbf{q} &=
\begin{bmatrix}
\cos\left(\frac{\|\delta\boldsymbol{\theta}\|}{2}\right)\\
\hat{\delta\boldsymbol{\theta}}\sin\left(\frac{\|\delta\boldsymbol{\theta}\|}{2}\right)
\end{bmatrix}, \\
\mathbf{q}_{k+1} &= \text{normalize}(\delta\mathbf{q} \otimes \mathbf{q}_k).
\end{align*}

\begin{align*}
\mathbf{f}_w &= R(\mathbf{q}_k)\,\mathbf{a} \\
\mathbf{v}_{k+1} &= \mathbf{v}_k + (\mathbf{f}_w + \mathbf{g})\,\Delta t \\
\mathbf{r}_{k+1} &= \mathbf{r}_k + \mathbf{v}_k\,\Delta t + \frac{1}{2}(\mathbf{f}_w + \mathbf{g})\,\Delta t^2 \\
\mathbf{b}_{g,k+1} &= \mathbf{b}_{g,k} \\
\mathbf{b}_{a,k+1} &= \mathbf{b}_{a,k}
\end{align*}

\section{Continuous-Time Error-State Model}
Using $S(\cdot)$ as the skew operator ($S(\mathbf{x})\,\mathbf{y} = \mathbf{x} \times \mathbf{y}$),
\begin{align*}
\delta\dot{\mathbf{r}} &= \delta\mathbf{v} \\
\delta\dot{\mathbf{v}} &= -\,R(\mathbf{q})\,S(\mathbf{a})\,\delta\boldsymbol{\theta} - R(\mathbf{q})\,\delta\mathbf{b}_a + \mathbf{n}_{a} \\
\delta\dot{\boldsymbol{\theta}} &= -\,S(\boldsymbol{\omega})\,\delta\boldsymbol{\theta} - \delta\mathbf{b}_g + \mathbf{n}_{g} \\
\delta\dot{\mathbf{b}}_g &= \mathbf{n}_{wg} \\
\delta\dot{\mathbf{b}}_a &= \mathbf{n}_{wa}
\end{align*}

Compactly, $\dot{\delta\mathbf{x}} = F_c\,\delta\mathbf{x} + G_c\,\mathbf{n}$.

\section{Discrete Covariance Propagation}
First-order discretization:
\begin{align*}
\Phi_k &\approx I + F_c\,\Delta t \\
Q_k &\approx G_c\,Q_c\,G_c^\top\,\Delta t \\
P_{k+1}^- &= \Phi_k\,P_k^+\,\Phi_k^\top + Q_k
\end{align*}

\section{Measurements (examples) and Jacobians}
\begin{itemize}
    \item GPS position: $h(\cdot)=\mathbf{r}$, hence $H=[I\ 0\ 0\ 0\ 0]$.
    \item GPS velocity: $h(\cdot)=\mathbf{v}$, hence $H=[0\ I\ 0\ 0\ 0]$.
    \item Barometer: $h(\cdot)=r_z$, pick row from position block.
    \item Magnetometer: $h(\cdot)=R(\mathbf{q})^\top \mathbf{m}_w$, linearize wrt $\delta\boldsymbol{\theta}$.
\end{itemize}

\section{Measurement Update (Error-State)}
\begin{align*}
S_k &= H_k P_k^- H_k^\top + R_k \\
K_k &= P_k^- H_k^\top S_k^{-1} \\
\delta\hat{\mathbf{x}}_k &= K_k\left(\mathbf{z}_k - h(x_{\text{nom},k}^-)\right) \\
P_k^+ &= (I - K_k H_k) P_k^-
\end{align*}

\textbf{Inject} into the nominal:
\begin{align*}
\mathbf{r} &\leftarrow \mathbf{r} + \delta\mathbf{r} \\
\mathbf{v} &\leftarrow \mathbf{v} + \delta\mathbf{v} \\
\mathbf{q} &\leftarrow \text{normalize}\left(\delta\mathbf{q} \otimes \mathbf{q}\right),\quad \delta\mathbf{q} \approx \begin{bmatrix}1\\ \frac{1}{2}\,\delta\boldsymbol{\theta}\end{bmatrix} \\
\mathbf{b}_g &\leftarrow \mathbf{b}_g + \delta\mathbf{b}_g \\
\mathbf{b}_a &\leftarrow \mathbf{b}_a + \delta\mathbf{b}_a
\end{align*}
Then reset $\delta\mathbf{x} = \mathbf{0}$.

\end{document}